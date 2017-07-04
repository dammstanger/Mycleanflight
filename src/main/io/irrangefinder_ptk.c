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


#define REVDATASIZE 4
#define PTK_MAXRANGECM	1200
#define PTK_DETECTION_CONE_DECIDEGREES 30 //  recommended cone angle 3 degrees, 单位0.1度
#define PTK_DETECTION_CONE_EXTENDED_DECIDEGREES 300

typedef enum {
	PTK_UNKNOWN,
	PTK_INITIALIZING,
	PTK_CHANGE_BAUD,
	PTK_CONFIGURE,
	PTK_RECEIVING_DATA,
	PTK_LOST_COMMUNICATION,
} ptkState_e;

#if defined(IRRANGFD)
ptkIrData_t ptkIrData;
static serialPort_t *ptkIrPort;

STATIC_UNIT_TESTED volatile int32_t measurement = -1;
static uint32_t lastMeasurementAt;


void ptkRevDat_Callback(uint16_t dat);

static void ptkIrSetState(ptkState_e state)
{
	ptkIrData.state = state;
	ptkIrData.lasttime = millis();
}

void ptkIrInit(irrangfd_t *irrangfd)
{
	ptkIrData.baudrateIndex = BAUD_115200;
	ptkIrData.errors = 0;
	ptkIrData.timeouts = 0;

	irrangfd->irrangfdMaxRangeCm = PTK_MAXRANGECM;
	irrangfd->irrangfddetectionConeDeciDegrees = PTK_DETECTION_CONE_DECIDEGREES;
	irrangfd->irrangfddetectionConeExtendedDeciDegrees = PTK_DETECTION_CONE_EXTENDED_DECIDEGREES;

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
	    ptkIrSetState(PTK_RECEIVING_DATA);
}


void ptkRevDat_Callback(uint16_t dat)
{
	static u8 check = 0,i = 0;
	static u8 ReceiveData[REVDATASIZE] = {0};

//	if(check==2)
//	{
//		ReceiveData[i] = (u8)dat;
//		i++;
//		if(i==REVDATASIZE)
//		{
//			i = 0;
//			check = 0;
//		    if (debugMode == DEBUG_IRRANGFD)
//		    {
//		        ptkIrData.dist = ((int16_t)ReceiveData[1]<<8)|ReceiveData[0];
//		        debug[0] = ptkIrData.dist;
//		    }
//		}
//	}
//	if((dat==0x59)&&(check==1)) {check = 2;i = 0;}
//	if((dat==0x59)&&(check==0))	check = 1;
	if(check==3)
	{
		ReceiveData[i] = (u8)dat;
		i++;
		if(i==REVDATASIZE)
		{
			i = 0;
			check = 0;
		    if (debugMode == DEBUG_IRRANGFD)
		    {
		        ptkIrData.dist = (((int16_t)ReceiveData[0]<<8|ReceiveData[1]) + 5)/10;			//四舍五入
		        debug[0] = ptkIrData.dist;
		    }
		}
	}
	if((dat==0x02)&&(check==2)) {check = 3;i = 0;}		//first three bytes are 0x01 0x03 0x02
	if((dat==0x03)&&(check==1)) {check = 2;}
	if((dat==0x01)&&(check==0))	check = 1;

//	serialWrite(ptkIrPort,(u8)dat);
}

/**
 * Send measure or read command if it is necessary
 */
void ptkWrtCmd(void)
{
	const uint8_t dattst[8]={0x01,0x03,0xD8,0xD9,0x00,0x01,0x6F,0x51};
    uint32_t now = millis();

	if(ptkIrData.state!=PTK_RECEIVING_DATA){
		return;
	}
    if (now < (lastMeasurementAt + 40)) {
        // PTK 测量周期为25Hz所以应在上一次40ms
        return;
    }

    lastMeasurementAt = now;
	serialWriteBuf(ptkIrPort,dattst,(int)sizeof(dattst));
}


/**
 * Get the distance that was measured by the last cycle, in centimeters.
 */
int32_t ptk_get_distance(void)
{
    int32_t distance = ptkIrData.dist;

    return distance;
}

#endif
