/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfig.h"

#if HAS_BAFSD

#include "bafsd.h"
#include "../../MarlinCore.h"
#include "../../module/planner.h"
#include "../../module/stepper.h"
#include "../../feature/pause.h"
#include "../../gcode/queue.h"
#include "../../module/temperature.h"
#include "../../gcode/gcode.h"

#define DEBUG_OUT ENABLED(DEBUG_BAFSD)
#include "../../core/debug_out.h"

#define BAFSD_SEND(S) tx_str(F(S "\n"))
#define BAFSD_RECV(S) rx_str(F(S "\n"))

#define NO_PORT 255
#define BAFSD_BAUD 9600

BAFSD bafsd;
uint8_t BAFSD::port;
uint8_t BAFSD::nextPort;
uint8_t BAFSD::response; // 0 = No, 1 = Yes, 2 = Error, 99 = Waiting for response;
int BAFSD::timeOut;
millis_t BAFSD::commandIssueTime;
bool BAFSD::waitingResponse;

char BAFSD::rx_buffer[BAFSD_RX_SIZE], BAFSD::tx_buffer[BAFSD_RX_SIZE];

BAFSD::BAFSD() {
  port = NO_PORT;
  nextPort = NO_PORT;
  rx_buffer[0] = '\0';
}

void BAFSD::init() {
  port = NO_PORT;
  BAFSD_SERIAL.begin(BAFSD_BAUD);

  safe_delay(10);
  reset();
  rx_buffer[0] = '\0';
}

void BAFSD::reset() {
  char msg[22];
  sprintf_P(msg, PSTR("M117 BAFS Reset"));
  queue.inject(msg);

  port = NO_PORT;
  BAFSD_SEND("M709");
}

void BAFSD::trigger_camera(const uint16_t d) {
  char msg[22];
  sprintf_P(msg, PSTR("M117 BAFS Trigger Cam"));
  queue.inject(msg);

  tx_printf(F("M240 D%d\n"), d);
}

uint8_t BAFSD::current_port() {
  return port;
}

uint8_t BAFSD::next_port() {
  return nextPort;
}

bool BAFSD::too_cold(uint8_t toolID){
  if (TERN0(PREVENT_COLD_EXTRUSION, !DEBUGGING(DRYRUN) && thermalManager.targetTooColdToExtrude(toolID))) {
    SERIAL_ECHO_MSG(STR_ERR_HOTEND_TOO_COLD);
    return true;
  }
  return false;
}

/*
Using filament sensor:
1. Filament is expected at parking position, which is just below the sensor
2. Retract to position before extruder gear: BAFSD_SENSOR_TO_GEAR_DISTANCE
3. Send PWM Signal to chosen port and wait: BAFSD_SERVO_DELAY
4. Feed until sensor is triggered, approx: BAFSD_SENSOR_TO_GEAR_DISTANCE, If not triggered, wait for user action
*/
void BAFSD::select_port(const uint8_t e) {
    char msg[40];

    if (port == NO_PORT) {
      port = e;
      nextPort = NO_PORT;
      tx_printf(F("T%d\n"), e);
      get_response(BAFSD_TIMEOUT); // Use default timeout, this should not take long
      if (response == 1) {
        sprintf_P(msg, PSTR("M117 BAFS Port: %u"), e);
        queue.inject(msg);
      } else {
          sprintf_P(msg, PSTR("M117 Failed Switching Port: %u"), e);
          queue.inject(msg);
      }
      return;
    }

    if (e != port){
      nextPort = e;
      if (too_cold(active_extruder)) {
        sprintf_P(msg, PSTR("M117 Extr too cold."));
        queue.inject(msg);
        return;
      }

      // 1. Filament is expected at parking position, which is just below the sensor
      DEBUG_ECHOLNPGM("Unloading to before sensor");
      stepper.enable_extruder();
      uint8_t filPresent = filament_present();
      while(filPresent == 1){
        current_position.e -= 1;
        line_to_current_position(MMM_TO_MMS(BAFSD_UNLOAD_FEEDRATE));
        planner.synchronize();
        filPresent = filament_present();
      };

      // 2. Retract to position before extruder gear: BAFSD_SENSOR_TO_GEAR_DISTANCE
      DEBUG_ECHOLNPGM("Unloading to before gear");
      current_position.e -= BAFSD_SENSOR_TO_GEAR_DISTANCE;
      line_to_current_position(MMM_TO_MMS(BAFSD_UNLOAD_FEEDRATE));
      planner.synchronize();
      stepper.disable_extruder();

      // 3. Send tool change command and wait
      bool toolChangeOk = true;

      DEBUG_ECHOLNPGM("Sending T", e, " to BAFSD");
      tx_printf(F("T%d\n"), e);

      DEBUG_ECHOLNPGM("Move extruder motor to help gripping the filament");
      const uint16_t slowMargin = 1200; // move extruder motor at the last moment
      safe_delay(BAFSD_FIL_CHANGE_DURATION - slowMargin);
      stepper.enable_extruder();
      current_position.e += 20;
      line_to_current_position(MMM_TO_MMS(BAFSD_LOAD_FEEDRATE));
      planner.synchronize();
      stepper.disable_extruder();

      get_response(BAFSD_TIMEOUT);
      if (response != 1) {
        toolChangeOk = false;
      }
      safe_delay(250);

      uint8_t fil_present = 0;
      if (toolChangeOk) {
        // 4. Feed until sensor is triggered, approx: BAFSD_SENSOR_TO_GEAR_DISTANCE, If not triggered, wait for user action
        DEBUG_ECHOLNPGM("Loading to sensor");
        fil_present = load_to_sensor();

        // Sensor is not triggered, possibly because it's not fed properly to the gear, try to feed some and retry
        if (fil_present != 1){
          DEBUG_ECHOLNPGM("Loading to sensor failed.");
          DEBUG_ECHOLNPGM("Retrying (C0).");
          safe_delay(250);
          tx_printf(F("C%d\n"), 0);
          safe_delay(BAFSD_SMALL_FEED_DURATION);
          get_response(BAFSD_TIMEOUT);
          safe_delay(250);
          fil_present = load_to_sensor();
        }
      }

      while (fil_present != 1){
        DEBUG_ECHOLNPGM("Filament change failed: ", fil_present, "-", toolChangeOk);
        constexpr xyz_pos_t park_point = NOZZLE_PARK_POINT;
        if (pause_print(0, park_point, true, 0)) {
          wait_for_confirmation(true, 5);
          resume_print(0, 0, 0, 0, 0);
          safe_delay(250);
          fil_present = load_to_sensor();
        }
        safe_delay(250);
      }

      sprintf_P(msg, PSTR("M117 BAFS Port: %u"), e);
      queue.inject(msg);
      port = e;
      nextPort = NO_PORT;
    }
}

// 0 = not present, 1 = present, 2 = error
uint8_t BAFSD::filament_present(){
  char msg[30];

  DEBUG_ECHOLNPGM("Sending M412 to BAFSD");
  BAFSD_SEND("M412");
  get_response(BAFSD_TIMEOUT);
  if (response == 0) {
    return 0;
  } else if (response == 1) {
    return 1;
  } else {
    sprintf_P(msg, PSTR("M117 BAFSD Error: F. Sensor"));
    queue.inject(msg);
    return 2;
  }
}

uint8_t BAFSD::load_to_sensor(){
  bool fil_loaded = 0;
  for (uint8_t i = 0; i < BAFSD_ATTEMPTS_NR; i++) {
    // Done as soon as filament is present
    fil_loaded = filament_present();
    if (fil_loaded != 0) break;

    // Attempt to load the filament, at a time, for 3s
    stepper.enable_extruder();
    const millis_t expire_ms = millis() + 3000;
    while ((fil_loaded == 0) && PENDING(millis(), expire_ms)) {
      current_position.e += 1;
      line_to_current_position(MMM_TO_MMS(BAFSD_LOAD_FEEDRATE));
      planner.synchronize();
      fil_loaded = filament_present();
      safe_delay(100);
    }
    stepper.disable_extruder();
  }
  DEBUG_ECHOLNPGM("Load to sensor: ", fil_loaded);
  return fil_loaded;
}

void BAFSD::bafsd_loop() {
  const millis_t expire_ms = commandIssueTime + timeOut;
  millis_t now = millis();
  while(waitingResponse && PENDING(now, expire_ms)){
    if (rx_ok()) {
      DEBUG_ECHOLNPGM("Rx: ok");
      response = 1;
      waitingResponse = false;
    } else if (rx_no()) {
      DEBUG_ECHOLNPGM("Rx: no");
      response = 0;
      waitingResponse = false;
    }
    now = millis();
  }

  if (waitingResponse){
    DEBUG_ECHOLNPGM("Time out: ", response, " - ", now, " - ", expire_ms, " - ", waitingResponse);
    response = 2;
    waitingResponse = false;
  }
}

void BAFSD::get_response(const int t) {
  timeOut = t;
  commandIssueTime = millis();
  waitingResponse = true;
  DEBUG_ECHOLNPGM("Waiting for response: ", commandIssueTime, ", t: ", timeOut);

  KEEPALIVE_STATE(PAUSED_FOR_USER);

  while (waitingResponse) {
    idle();
  }
  DEBUG_ECHOLNPGM("Response: ", response);
}

/**
 * Transfer data to BAFSD, no argument
 */
void BAFSD::tx_str(FSTR_P fstr) {
  clear_rx_buffer();
  PGM_P pstr = FTOP(fstr);
  while (const char c = pgm_read_byte(pstr)) { BAFSD_SERIAL.write(c); pstr++; }
}

/**
 * Transfer data to BAFSD, single argument
 */
void BAFSD::tx_printf(FSTR_P format, int argument = -1) {
  clear_rx_buffer();
  const uint8_t len = sprintf_P(tx_buffer, FTOP(format), argument);
  LOOP_L_N(i, len) BAFSD_SERIAL.write(tx_buffer[i]);
}

/**
 * Transfer data to BAFSD, two arguments
 */
void BAFSD::tx_printf(FSTR_P format, int argument1, int argument2) {
  clear_rx_buffer();
  const uint8_t len = sprintf_P(tx_buffer, FTOP(format), argument1, argument2);
  LOOP_L_N(i, len) BAFSD_SERIAL.write(tx_buffer[i]);
}

/**
 * Check if we received 'ok' from BAFSD
 */
bool BAFSD::rx_ok() {
  if (BAFSD_RECV("ok")) {
    return true;
  }
  return false;
}

/**
 * Check if we received 'no' from BAFSD
 */
bool BAFSD::rx_no() {
  if (BAFSD_RECV("no")) {
    return true;
  }
  return false;
}

/**
 * Check if the data received ends with the given string.
 */
bool BAFSD::rx_str(FSTR_P fstr) {
  PGM_P pstr = FTOP(fstr);

  uint8_t i = strlen(rx_buffer);

  while (BAFSD_SERIAL.available()) {
    rx_buffer[i++] = BAFSD_SERIAL.read();

    if (i == sizeof(rx_buffer) - 1) {
      DEBUG_ECHOLNPGM("rx buffer overrun");
      break;
    }
  }
  rx_buffer[i] = '\0';

  uint8_t len = strlen_P(pstr);

  if (i < len) return false;

  pstr += len;

  while (len--) {
    char c0 = pgm_read_byte(pstr--), c1 = rx_buffer[i--];
    if (c0 == c1) continue;
    if (c0 == '\r' && c1 == '\n') continue;  // match cr as lf
    if (c0 == '\n' && c1 == '\r') continue;  // match lf as cr
    return false;
  }
  return true;
}

/**
 * Empty the rx buffer
 */
void BAFSD::clear_rx_buffer() {
  while (BAFSD_SERIAL.available()) BAFSD_SERIAL.read();
  rx_buffer[0] = '\0';
  response = 99;
}

#endif
