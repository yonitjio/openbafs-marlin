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

/**
 * module/servo.h
 */

#include "../inc/MarlinConfig.h"
#include "../HAL/shared/servo.h"

#if HAS_SERVO_ANGLES

  #if ENABLED(SWITCHING_EXTRUDER)
    // Switching extruder can have 2 or 4 angles
    #if EXTRUDERS > 3
      #define REQ_ANGLES 4
    #else
      #define REQ_ANGLES 2
    #endif
    constexpr uint16_t sase[] = SWITCHING_EXTRUDER_SERVO_ANGLES;
    static_assert(COUNT(sase) == REQ_ANGLES, "SWITCHING_EXTRUDER_SERVO_ANGLES needs " STRINGIFY(REQ_ANGLES) " angles.");
  #else
    constexpr uint16_t sase[4] = { 0 };
  #endif

  #if ENABLED(SWITCHING_NOZZLE)
    constexpr uint16_t sasn[] = SWITCHING_NOZZLE_SERVO_ANGLES;
    static_assert(COUNT(sasn) == 2, "SWITCHING_NOZZLE_SERVO_ANGLES needs 2 angles.");
  #else
    constexpr uint16_t sasn[2] = { 0 };
  #endif

  #ifdef Z_PROBE_SERVO_NR
    #if ENABLED(BLTOUCH)
      #include "../feature/bltouch.h"
      #undef Z_SERVO_ANGLES
      #define Z_SERVO_ANGLES { BLTOUCH_DEPLOY, BLTOUCH_STOW }
    #endif
    constexpr uint16_t sazp[] = Z_SERVO_ANGLES;
    static_assert(COUNT(sazp) == 2, "Z_SERVO_ANGLES needs 2 angles.");
  #else
    constexpr uint16_t sazp[2] = { 0 };
  #endif

  #ifdef BAFSD_SERVO_NR
    constexpr uint16_t safs[] = BAFSD_SERVO_ANGLES;
    static_assert(COUNT(safs) >= 2, "BAFSD_SERVO_ANGLES needs at least 2 angles.");
  #else
    constexpr uint16_t safs[2] = { 0 };
  #endif

#ifndef SWITCHING_EXTRUDER_SERVO_NR
    #define SWITCHING_EXTRUDER_SERVO_NR -1
  #endif
  #ifndef SWITCHING_EXTRUDER_E23_SERVO_NR
    #define SWITCHING_EXTRUDER_E23_SERVO_NR -1
  #endif
  #ifndef SWITCHING_NOZZLE_SERVO_NR
    #define SWITCHING_NOZZLE_SERVO_NR -1
  #endif
  #ifndef Z_PROBE_SERVO_NR
    #define Z_PROBE_SERVO_NR -1
  #endif
  #ifndef BAFSD_SERVO_NR
    #define BAFSD_SERVO_NR -1
  #endif
  

  #define ASRC(N,I) (                                  \
      N == SWITCHING_EXTRUDER_SERVO_NR     ? sase[I]   \
    : N == SWITCHING_EXTRUDER_E23_SERVO_NR ? sase[I+2] \
    : N == SWITCHING_NOZZLE_SERVO_NR       ? sasn[I]   \
    : N == Z_PROBE_SERVO_NR                ? sazp[I]   \
    : N == BAFSD_SERVO_NR                  ? safs[I]   \
    : 0                                                )

  #if ENABLED(EDITABLE_SERVO_ANGLES)
    #if HAS_BAFSD
      #define NUM_SERVOS_EX NUM_SERVOS - 1
      extern uint16_t servo_angles[NUM_SERVOS_EX][2];
      #define CONST_SERVO_ANGLES base_servo_angles

      extern uint16_t bafsd_servo_angles[EXTRUDERS];
      #define CONST_MULTI_SERVO_ANGLES base_bafsd_servo_angles
    #else
      extern uint16_t servo_angles[NUM_SERVOS][2];
      #define CONST_SERVO_ANGLES base_servo_angles
    #endif
  #else
    #define CONST_SERVO_ANGLES servo_angles
    #if HAS_BAFSD
      #define CONST_MULTI_SERVO_ANGLES bafsd_servo_angles
    #endif
  #endif

  constexpr uint16_t CONST_SERVO_ANGLES [NUM_SERVOS][2] = {
      { ASRC(0,0), ASRC(0,1) }
    #if NUM_SERVOS > 1
      , { ASRC(1,0), ASRC(1,1) }
      #if NUM_SERVOS > 2
        , { ASRC(2,0), ASRC(2,1) }
        #if NUM_SERVOS > 3
          , { ASRC(3,0), ASRC(3,1) }
        #endif
      #endif
    #endif
  };

  #if HAS_BAFSD 
    constexpr uint16_t CONST_MULTI_SERVO_ANGLES[EXTRUDERS] = {
      ASRC(0, 0), ASRC(0, 1)
      #if EXTRUDERS > 2
         , ASRC(0, 2)
        #if EXTRUDERS > 3
          , ASRC(0, 3)
          #if EXTRUDERS > 4
            , ASRC(0, 4)
            #if EXTRUDERS > 5
              , ASRC(0, 5)
              #if EXTRUDERS > 6
                , ASRC(0, 7)
                #if EXTRUDERS > 8
                  , ASRC(0, 8)
                #endif
              #endif
            #endif
          #endif
        #endif
      #endif
    };  
  #endif

  #if HAS_Z_SERVO_PROBE
    #define DEPLOY_Z_SERVO() servo[Z_PROBE_SERVO_NR].move(servo_angles[Z_PROBE_SERVO_NR][0])
    #define STOW_Z_SERVO() servo[Z_PROBE_SERVO_NR].move(servo_angles[Z_PROBE_SERVO_NR][1])
  #endif

#endif // HAS_SERVO_ANGLES

extern hal_servo_t servo[NUM_SERVOS];
void servo_init();
