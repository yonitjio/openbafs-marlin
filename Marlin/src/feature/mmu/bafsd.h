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
#pragma once

#define BAFSD_RX_SIZE  16
#define BAFSD_TX_SIZE  16

class BAFSD {
public:
  BAFSD();
  static void init();
  static void select_port(const uint8_t e);
  static uint8_t current_port();
  static uint8_t next_port();
  static void reset();
  static void trigger_camera(const uint16_t d);

  static void bafsd_loop();

private:
  static uint8_t port;
  static uint8_t nextPort;
  static millis_t commandIssueTime;
  static int timeOut;
  static uint8_t response;
  static bool waitingResponse;
  static char rx_buffer[BAFSD_RX_SIZE], tx_buffer[BAFSD_TX_SIZE];

  static bool rx_str(FSTR_P fstr);
  static void tx_str(FSTR_P fstr);
  static void tx_printf(FSTR_P ffmt, const int argument);
  static void tx_printf(FSTR_P ffmt, const int argument1, const int argument2);
  static void clear_rx_buffer();

  static bool rx_ok();
  static bool rx_no();

  static void get_response(const int t);

  static uint8_t load_to_sensor();
  static uint8_t filament_present();
  static bool too_cold(const uint8_t e);
};

extern BAFSD bafsd;