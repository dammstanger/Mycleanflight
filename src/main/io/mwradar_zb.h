/*
 * mwradar_zb.h
 *
 *  Created on: 2017年7月18日
 *      Author: DammStanger
 */

#pragma once

#include "platform.h"
#include "sensors/mwradar.h"

typedef struct zbMwData_s {
    uint8_t state;                  // PTKir thread state. Used for detecting cable disconnects and configuring attached devices
    uint8_t baudrateIndex;          // index into auto-detecting or current baudrate
    uint32_t errors;                // PTL error counter - crc error/lost of data/sync etc..
    uint32_t timeouts;
    uint32_t lasttime;           	// last time valid PTK data was received (millis)
    uint16_t dist;					//RAW dist data
    int16_t vel;					//RAW vel data
} zbMwData_t;


void zbWmInit(mwradar_t *mwradar);
void zbMwWrtCmd(void);
int32_t zbMw_get_distance(void);
void zbMwSensorWorkChk(void);
bool isZbMwWorkFind(void);
