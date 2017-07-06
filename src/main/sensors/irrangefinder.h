/*
 * irrangefinder.h
 *
 *  Created on: 2017年6月22日
 *      Author: DammStanger
 */

#pragma once

#define IRRANGFD_OUT_OF_RANGE (-1)

typedef struct{
    int16_t irrangfddetectionConeDeciDegrees; // detection cone angle as in HC-SR04 device spec
    int16_t irrangfddetectionConeExtendedDeciDegrees; // device spec is conservative, in practice have slightly larger detection cone

	int16_t irrangfdMaxRangeCm;
	int16_t irrangfdMaxAltWithTiltCm;
	int16_t irrangfdCfAltCm; // Complimentary Filter altitude
	int16_t irrangfdMaxTiltDeciDegrees;
	float sonarMaxTiltCos;

	int32_t calculatedAltitude;
}irrangfd_t;

extern irrangfd_t irrangfd;

void irrangfdUpdate(void);
int32_t irrangfdRead(void);
int32_t irrangfdCalculateAltitude(int32_t irrangfdDistance, float cosTiltAngle);
int32_t irrangfdGetLatestAltitude(void);
