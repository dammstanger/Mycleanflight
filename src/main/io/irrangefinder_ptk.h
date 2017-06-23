/*
 * irrangefinder_ptk.h
 *
 *  Created on: 2017年6月15日
 *      Author: DammStanger
 */

/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */


#pragma once

#include "platform.h"
#include "sensors/irrangefinder.h"

typedef struct ptkIrData_s {
    uint8_t state;                  // PTKir thread state. Used for detecting cable disconnects and configuring attached devices
    uint8_t baudrateIndex;          // index into auto-detecting or current baudrate
    uint32_t errors;                // PTL error counter - crc error/lost of data/sync etc..
    uint32_t timeouts;
    uint32_t lasttime;           	// last time valid PTK data was received (millis)
    uint16_t dist;					//RAW dis data
} ptkIrData_t;


void ptkIrInit(irrangfd_t *irrangfd);
void ptkWrtCmd(void);
int32_t ptk_get_distance(void);
