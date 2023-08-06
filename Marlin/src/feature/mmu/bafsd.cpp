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

#include "../../MarlinCore.h"
#include "../../module/planner.h"
#include "../../module/stepper.h"
#include "../../module/servo.h"
#include "../../gcode/queue.h"

void bafsd_init() {
}

void select_port(const uint8_t e) {
  char msg[22];

  sprintf_P(msg, PSTR("M117 BAFS Port: %u"), e);
  queue.inject(msg);  

  planner.synchronize();
  stepper.disable_e_steppers();

  servo[BAFSD_SERVO_NR].move(bafsd_servo_angles[e]);
  safe_delay(BAFSD_SERVO_DELAY);
  servo[BAFSD_SERVO_NR].detach();
}

#endif // HAS_BAFSD
