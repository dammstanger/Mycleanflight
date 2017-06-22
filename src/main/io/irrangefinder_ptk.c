/*
 * irrangefinder_ptk.c
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
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

#include <platform.h>
#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/utils.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "config/feature.h"

#include "drivers/dma.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"

#include "sensors/sensors.h"
//#include "sensors/irrangefinder.h"

#include "fc/config.h"
#include "fc/runtime_config.h"
#include "fc/fc_serial.h"
#include "fc/fc_debug.h"

#include "io/serial.h"
#include "io/display.h"
#include "io/irrangefinder_ptk.h"

#include "flight/pid.h"
#include "flight/navigation.h"




typedef enum {
	PTK_UNKNOWN,
	PTK_INITIALIZING,
	PTK_CHANGE_BAUD,
	PTK_CONFIGURE,
	PTK_RECEIVING_DATA,
	PTK_LOST_COMMUNICATION,
} ptkState_e;

ptkIrData_t ptkIrData;
static serialPort_t *ptkIrPort;


static void ptkIrSetState(ptkState_e state)
{
	ptkIrData.state = state;
	ptkIrData.lasttime = millis();
}


void ptkRevDat_Callback(uint16_t dat);

void ptkIrInit(void)
{
	ptkIrData.baudrateIndex = BAUD_115200;
	ptkIrData.errors = 0;
	ptkIrData.timeouts = 0;


	// init ptkIrData structure. if we're not actually enabled, don't bother doing anything else
	ptkIrSetState(PTK_UNKNOWN);

    serialPortConfig_t *ptkIrPortConfig = findSerialPortConfig(FUNCTION_IRRANGDF_PTK);			//如果是NULL，则没有接口可用
    if (!ptkIrPortConfig) {
        featureClear(FEATURE_IRRANGFD);
        return;
    }


		portMode_t mode = MODE_RXTX;

		ptkIrPort = openSerialPort(ptkIrPortConfig->identifier,FUNCTION_IRRANGDF_PTK,ptkRevDat_Callback,
				baudRates[ptkIrData.baudrateIndex],mode,SERIAL_NOT_INVERTED);

	    if (!ptkIrPort) {
	        featureClear(FEATURE_IRRANGFD);
	        return;
	    }

	    // signal PTK "thread" to initialize when it gets to it
	    ptkIrSetState(PTK_INITIALIZING);
}


void ptkRevDat_Callback(uint16_t dat)
{
    if (debugMode == DEBUG_GYRO)
        debug[3] = (int16_t)dat;
}


uint8_t dattst[]="GKXN!";
void ptkWrtCmd(void)
{
	serialWriteBuf(ptkIrPort,dattst,(int)strlen(dattst));
}


