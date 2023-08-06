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

#if ENABLED(EDITABLE_SERVO_ANGLES)

#include "../gcode.h"
#include "../../module/servo.h"

/**
 * M281 - Edit / Report Servo Angles
 *
 *  P<index> - Servo to update
 *  L<angle> - Deploy Angle
 *  U<angle> - Stowed Angle
 */
void GcodeSuite::M281() {
  if (!parser.seen_any()) return M281_report();

  if (!parser.seenval('P')) return;

  const int servo_index = parser.value_int();
  if (WITHIN(servo_index, 0, NUM_SERVOS - 1)) {
    #if ENABLED(BLTOUCH)
      if (servo_index == Z_PROBE_SERVO_NR) {
        SERIAL_ERROR_MSG("BLTouch angles can't be changed.");
        return;
      }
    #endif
    #if !HAS_BAFSD
      if (parser.seenval('L')) servo_angles[servo_index][0] = parser.value_int();
      if (parser.seenval('U')) servo_angles[servo_index][1] = parser.value_int();
    #else
      if (servo_index == BAFSD_SERVO_NR){
        if (parser.seenval('A')) bafsd_servo_angles[0] = parser.value_int();
        if (parser.seenval('B')) bafsd_servo_angles[1] = parser.value_int();
        #if EXTRUDERS > 2
          if (parser.seenval('C')) bafsd_servo_angles[2] = parser.value_int();
          #if EXTRUDERS > 3
            if (parser.seenval('D')) bafsd_servo_angles[3] = parser.value_int();
            #if EXTRUDERS > 4
              if (parser.seenval('E')) bafsd_servo_angles[4] = parser.value_int();
              #if EXTRUDERS > 5
                if (parser.seenval('F')) bafsd_servo_angles[5] = parser.value_int();
                #if EXTRUDERS > 6
                  if (parser.seenval('G')) bafsd_servo_angles[6] = parser.value_int();
                  #if EXTRUDERS > 7
                    if (parser.seenval('H')) bafsd_servo_angles[7] = parser.value_int();
                  #endif
                #endif
              #endif
            #endif
          #endif
        #endif
      } else {
        if (parser.seenval('L')) servo_angles[servo_index][0] = parser.value_int();
        if (parser.seenval('U')) servo_angles[servo_index][1] = parser.value_int();
      }
    #endif  
  }
  else
    SERIAL_ERROR_MSG("Servo ", servo_index, " out of range");
}

void GcodeSuite::M281_report(const bool forReplay/*=true*/) {
  report_heading_etc(forReplay, F(STR_SERVO_ANGLES));
  LOOP_L_N(i, NUM_SERVOS) {
    switch (i) {
      default: break;
      #if ENABLED(SWITCHING_EXTRUDER)
        case SWITCHING_EXTRUDER_SERVO_NR:
        #if EXTRUDERS > 3
          case SWITCHING_EXTRUDER_E23_SERVO_NR:
        #endif
      #elif ENABLED(SWITCHING_NOZZLE)
        case SWITCHING_NOZZLE_SERVO_NR:
      #elif ENABLED(BLTOUCH) || (HAS_Z_SERVO_PROBE && defined(Z_SERVO_ANGLES))
        case Z_PROBE_SERVO_NR:
      #elif HAS_BAFSD
        case BAFSD_SERVO_NR:
      #endif
          report_echo_start(forReplay);
          #if HAS_BAFSD
            if (i == BAFSD_SERVO_NR) {
              SERIAL_ECHOLNPGM("  M281 P", i, 
                " A", bafsd_servo_angles[0], 
                " B", bafsd_servo_angles[1]
                #if EXTRUDERS > 2
                  , " C", bafsd_servo_angles[2]
                  #if EXTRUDERS > 3
                    ," D", bafsd_servo_angles[3]
                    #if EXTRUDERS > 4
                      , " E", bafsd_servo_angles[4]
                      #if EXTRUDERS > 5
                        , " F", bafsd_servo_angles[5]
                        #if EXTRUDERS > 6
                          , " G", bafsd_servo_angles[6]
                          #if EXTRUDERS > 7
                            ," H", bafsd_servo_angles[7]
                          #endif
                        #endif
                      #endif
                    #endif
                  #endif
                #endif
                );
            } else {
              SERIAL_ECHOLNPGM("  M281 P", i, " L", servo_angles[i][0], " U", servo_angles[i][1]);
            }
          #else
              SERIAL_ECHOLNPGM("  M281 P", i, " L", servo_angles[i][0], " U", servo_angles[i][1]);
          #endif
    }
  }
}

#endif // EDITABLE_SERVO_ANGLES
