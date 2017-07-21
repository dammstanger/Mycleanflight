/*
 * mwrader_zb.c
 *
 *  Created on: 2017年7月18日
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

#include "fc/config.h"
#include "fc/runtime_config.h"
#include "fc/fc_serial.h"
#include "fc/fc_debug.h"

#include "io/serial.h"
#include "io/display.h"
#include "io/mwrader_zb.h"

#include "flight/pid.h"
#include "flight/navigation.h"


#if defined(MWRADER) && defined(USE_ZB005)

#define REVDATASIZE_MAX 18
#define ZB_MAXRANGECM	1200
#define ZB_DETECTION_CONE_DECIDEGREES 360 //  recommended cone angle 36 degrees, 单位0.1度
#define ZB_DETECTION_CONE_EXTENDED_DECIDEGREES 450

typedef enum {
	ZB_UNKNOWN,
	ZB_INITIALIZING,
	ZB_CHANGE_BAUD,
	ZB_CONFIGURE,
	ZB_RECEIVING_DATA,
	ZB_LOST_COMMUNICATION,
} zbState_e;

static bool workfind = false;					//传感器正常工作标志位
static bool revdatflg = false;					//接收到数据包标志

zbMwData_t zbMwData;
static serialPort_t *zbMwPort;

static uint32_t lastMeasurementAt;


void zbRevDat_Callback(uint16_t dat);

static void zbWmSetState(zbState_e state)
{
	zbMwData.state = state;
	zbMwData.lasttime = millis();
}

void zbWmInit(mwrader_t *mwrader)
{
	zbMwData.baudrateIndex = BAUD_38400;
	zbMwData.errors = 0;
	zbMwData.timeouts = 0;

	mwrader->mwraderMaxRangeCm = ZB_MAXRANGECM;
	mwrader->mwraderdetectionConeDeciDegrees = ZB_DETECTION_CONE_DECIDEGREES;
	mwrader->mwraderdetectionConeExtendedDeciDegrees = ZB_DETECTION_CONE_EXTENDED_DECIDEGREES;

	// init zbMwData structure. if we're not actually enabled, don't bother doing anything else
	zbWmSetState(ZB_UNKNOWN);

    serialPortConfig_t *zbMwPortConfig = findSerialPortConfig(FUNCTION_MWRADER_ZB);			//如果是NULL，则没有接口可用
    if (!zbMwPortConfig) {
        featureClear(FEATURE_MWRADER);
        return;
    }


		portMode_t mode = MODE_RXTX;

		zbMwPort = openSerialPort(zbMwPortConfig->identifier,FUNCTION_MWRADER_ZB,zbRevDat_Callback,
				baudRates[zbMwData.baudrateIndex],mode,SERIAL_NOT_INVERTED);

	    if (!zbMwPort) {
	        featureClear(FEATURE_MWRADER);
	        return;
	    }

	    // signal RADER "thread" to initialize when it gets to it
	    zbWmSetState(ZB_RECEIVING_DATA);
}

void tst_Nra24PakHandle(uint16_t dat);
//对zb005协议包解析 ASCII格式
void zbRevDat_Callback(uint16_t dat)
{
	static u8 check = 0,i = 0;
	static u8 ReceiveData[REVDATASIZE_MAX] = {0};

	if(check==3)
	{
		ReceiveData[i] = (u8)dat;

		if(ReceiveData[i-1]==0x0D && ReceiveData[i]==0x0A )						//到达包尾
		{
			if(ReceiveData[3]=='.')
			{
				zbMwData.dist = (uint16_t)((ReceiveData[2]-'0')*100 +			//解析出距离
										  (ReceiveData[4]-'0')*10 +
										  (ReceiveData[5]-'0'));
				if(ReceiveData[8]=='.')
				{
					zbMwData.vel = (int16_t)((ReceiveData[7]-'0')*100 +			//解析出速度
											(ReceiveData[9]-'0')*10 +
											(ReceiveData[10]-'0'));
				}
				else if(ReceiveData[10]=='.')
				{
					zbMwData.vel = -(int16_t)((ReceiveData[8]-'0')*1000 +		//解析出速度
											 (ReceiveData[9]-'0')*100 +
											 (ReceiveData[11]-'0')*10 +
											 (ReceiveData[12]-'0'));
				}
				else if(ReceiveData[9]=='.')
				{
					if(ReceiveData[7]=='-')
					{
						zbMwData.vel = -(int16_t)((ReceiveData[8]-'0')*100 +	//解析出速度
												 (ReceiveData[10]-'0')*10 +
												 (ReceiveData[11]-'0'));
					}
					else{
						zbMwData.vel = (int16_t)((ReceiveData[7]-'0')*1000 +	//解析出速度
												(ReceiveData[8]-'0')*100 +
												(ReceiveData[10]-'0')*10 +
												(ReceiveData[11]-'0'));
					}
				}
				else {i = 0;check = 0;return ;}
			}//end if(ReceiveData[3]=='.')
			else if(ReceiveData[4]=='.')
			{
				zbMwData.dist = (uint16_t)((ReceiveData[2]-'0')*1000 +			//解析出距离
										  (ReceiveData[3]-'0')*100 +
										  (ReceiveData[5]-'0')*10 +
										  (ReceiveData[6]-'0'));
				if(ReceiveData[9]=='.')
				{
					zbMwData.vel = (int16_t)((ReceiveData[8]-'0')*100 +			//解析出速度
											(ReceiveData[10]-'0')*10 +
											(ReceiveData[1]-'0'));
				}
				else if(ReceiveData[11]=='.')
				{
					zbMwData.vel = -(int16_t)((ReceiveData[9]-'0')*1000 +		//解析出速度
											 (ReceiveData[10]-'0')*100 +
											 (ReceiveData[12]-'0')*10 +
											 (ReceiveData[13]-'0'));
				}
				else if(ReceiveData[10]=='.')
				{
					if(ReceiveData[7]=='-')
					{
						zbMwData.vel = -(int16_t)((ReceiveData[9]-'0')*100 +	//解析出速度
												 (ReceiveData[11]-'0')*10 +
												 (ReceiveData[12]-'0'));
					}
					else{
						zbMwData.vel = (int16_t)((ReceiveData[8]-'0')*1000 +	//解析出速度
												(ReceiveData[9]-'0')*100 +
												(ReceiveData[11]-'0')*10 +
												(ReceiveData[12]-'0'));
					}
				}
				else {i = 0;check = 0;return ;}
			} //end else if(ReceiveData[4]=='.')
			else {i = 0;check = 0;return ;}

			i = 0;
			check = 0;
			revdatflg = true;						//到达这里说明数据有效

		    if (debugMode == DEBUG_IRRANGFD)
		    {
		        debug[0] = zbMwData.dist;
		        debug[1] = zbMwData.vel;
		    }
		}	// end if(ReceiveData[i-1]==0x0D && ReceiveData[i]==0x0A )
		else{
			i++;
		}
	}
	else{
		if((dat==0x0A)&&(check==2)) {check = 3;i = 0;}		//first three bytes are 0x31 0x0D 0x0A
		else if(check==2) check = 0;
		if((dat==0x0D)&&(check==1)) check = 2;
		else if(check==1) check = 0;
		if((dat==0x31)&&(check==0))	check = 1;
	}
//	tst_Nra24PakHandle(dat);
//	serialWrite(zbMwPort,dat);
}

#define SIZENOHEAD 12
void tst_Nra24PakHandle(uint16_t dat)
{
	//NRA24格式 只读取0x70C 和 0x70B两类包
	static u8 check = 0,i = 0;
	static u8 ReceiveData[SIZENOHEAD] = {0};

	if(check==2)
	{
		ReceiveData[i] = (u8)dat;

		if(i==(SIZENOHEAD-1))
		{
			i = 0;
			check = 0;
			if(ReceiveData[0]==0x0B && ReceiveData[1]==0x07)
			{
				revdatflg = true;
				return ;
			}
			else if(ReceiveData[0]==0x0C && ReceiveData[1]==0x07)
			{
				zbMwData.dist = ((uint16_t)ReceiveData[4]<<8|ReceiveData[5]);		//cm
			}
			else return;

		    if (debugMode == DEBUG_IRRANGFD)
		    {
		        debug[0] = zbMwData.dist;
		    }
		}
		else{
			i++;
		}
	}
	else{
		if((dat==0xAA)&&(check==1)) {check = 2;i = 0;}		//first three bytes are 0xAA 0xAA
		else if(check==1) check = 0;
		if((dat==0xAA)&&(check==0))	check = 1;
	}
}


/**
 * Send measure or read command if it is necessary
 */
void zbMwWrtCmd(void)
{
	const uint8_t dattst[8]={0x01,0x03,0xD8,0xD9,0x00,0x01,0x6F,0x51};
    uint32_t now = millis();

	if(zbMwData.state!=ZB_RECEIVING_DATA){
		return;
	}
    if (now < (lastMeasurementAt + 40)) {
        // PTK 测量周期为25Hz所以应在上一次40ms
        return;
    }

    lastMeasurementAt = now;
	serialWriteBuf(zbMwPort,dattst,(int)sizeof(dattst));
}


/**
 * Get the distance that was measured by the last cycle, in centimeters.
 */
int32_t zbMw_get_distance(void)
{
    int32_t distance = zbMwData.dist;

    return distance;
}

void zbMwSensorWorkChk()
{
	static int8_t datrevlosecnt = 0;				//数据丢包计数
	if(revdatflg)
	{
		revdatflg = false;
		workfind = true;
		datrevlosecnt = 0;
		return ;
	}
	if(datrevlosecnt>=5){
		workfind = false;
	}
	else
		datrevlosecnt++;
}

bool isZbMwWorkFind()
{
	return workfind;
}


#endif


