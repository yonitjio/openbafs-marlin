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
#include "../../module/servo.h"
#include "../../feature/pause.h"
#include "../../gcode/queue.h"
#include "../../module/temperature.h"

#define DEBUG_OUT ENABLED(DEBUG_BAFSD)
#include "../../core/debug_out.h"

#define FILAMENT_PRESENT() (READ(BAFSD_SENSOR_PIN) == LOW)
#define NO_PORT 255

BAFS_D bafs_d;
uint8_t BAFS_D::port;

BAFS_D::BAFS_D() {
  port = NO_PORT;
}

void BAFS_D::init() {
  port = NO_PORT;
  #if ENABLED(BAFSD_FILAMENT_SENSOR)
    SET_INPUT_PULLDOWN(BAFSD_SENSOR_PIN);
  #endif
}

void BAFS_D::reset() {
  char msg[22];
  sprintf_P(msg, PSTR("M117 BAFS Reset"));
  queue.inject(msg);  
  
  port = NO_PORT;

  servo[BAFSD_SERVO_NR].move(BAFSD_RESET_SERVO_ANGLE);
  safe_delay(BAFSD_RESET_SERVO_DELAY);
  servo[BAFSD_SERVO_NR].detach();
}

bool BAFS_D::too_cold(uint8_t toolID){
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
  #if ENABLED(BAFSD_FILAMENT_SENSOR)
    void BAFS_D::select_port(const uint8_t e) {
      char msg[22];

      sprintf_P(msg, PSTR("M117 BAFS Port: %u"), e);
      queue.inject(msg);  

      if (port == NO_PORT) {
        port = e;
        servo[BAFSD_SERVO_NR].move(bafsd_servo_angles[e]);
        safe_delay(100); // no need to delay too long as this won't actually move the servo
        return;
      }

      if (e != port){
        if (too_cold(active_extruder)) {
          sprintf_P(msg, PSTR("M117 Extr too cold."));
          queue.inject(msg);  
          return;
        }

        // 1. Filament is expected at parking position, which is just below the sensor
        stepper.enable_extruder();
        while(FILAMENT_PRESENT()){
          current_position.e -= 1;
          line_to_current_position(MMM_TO_MMS(BAFSD_UNLOAD_FEEDRATE));
          planner.synchronize();
        };

        // 2. Retract to position before extruder gear: BAFSD_SENSOR_TO_GEAR_DISTANCE
        current_position.e -= BAFSD_SENSOR_TO_GEAR_DISTANCE;
        line_to_current_position(MMM_TO_MMS(BAFSD_UNLOAD_FEEDRATE));
        planner.synchronize();
        stepper.disable_extruder();

        // 3. Send PWM Signal to chosen port and wait: BAFSD_SERVO_DELAY
        servo[BAFSD_SERVO_NR].move(bafsd_servo_angles[e]);
        safe_delay(BAFSD_SERVO_DELAY);

        // 4. Feed until sensor is triggered, approx: BAFSD_SENSOR_TO_GEAR_DISTANCE, If not triggered, wait for user action
        bool fil_present = load_to_sensor();

        // Sensor is not triggered, possibly because it's not fed properly to the gear, try to feed some and retry
        if (!fil_present){
          DEBUG_ECHOLNPGM("Asking BAFSD to feed");
          servo[BAFSD_SERVO_NR].move(BAFSD_FEED_SERVO_ANGLE);
          safe_delay(BAFSD_FEED_SERVO_DELAY);

          fil_present = load_to_sensor();
        }

        while (!fil_present){
          constexpr xyz_pos_t park_point = NOZZLE_PARK_POINT;
          if (pause_print(0, park_point, true, 0)) {
            wait_for_confirmation(true, 0);
            resume_print(0, 0, 0, 0, 0);
            fil_present = load_to_sensor();
          }
        }

        port = e;
      }
      servo[BAFSD_SERVO_NR].detach();
    }

    bool BAFS_D::load_to_sensor(){
      DEBUG_ECHOLNPGM("Loading filament to sensor");
      bool fil_present = 0;
      for (uint8_t i = 0; i < BAFSD_ATTEMPTS_NR; i++) {
        // Done as soon as filament is present
        fil_present = FILAMENT_PRESENT();
        if (fil_present) break;

        // Attempt to load the filament, at a time, for 3s
        stepper.enable_extruder();
        const millis_t expire_ms = millis() + 3000;
        do {
          current_position.e += 1;
          line_to_current_position(MMM_TO_MMS(BAFSD_LOAD_FEEDRATE));
          planner.synchronize();
          fil_present = FILAMENT_PRESENT();
        } while (!fil_present && PENDING(millis(), expire_ms));
        stepper.disable_extruder();
      }
      return fil_present;  
    }
  #else
    void BAFS_D::select_port(const uint8_t e) {
    char msg[22];
    sprintf_P(msg, PSTR("M117 BAFS Port: %u"), e);
    queue.inject(msg);  

    if (port == NO_PORT) {
      port = e;
      servo[BAFSD_SERVO_NR].move(bafsd_servo_angles[e]);
      safe_delay(100); // no need to delay too long as this won't actually move the servo
    } else if (e != port){
      port = e;
      servo[BAFSD_SERVO_NR].move(bafsd_servo_angles[e]);
      safe_delay(BAFSD_SERVO_DELAY);
    }
  }
  #endif // ENABLED(BAFSD_FILAMENT_SENSOR)
#endif // HAS_BAFSD
