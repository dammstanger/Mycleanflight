/*
 * mwradar.h
 *
 *  Created on: 2017年7月18日
 *      Author: DammStanger
 */

#pragma once

#define MWRADAR_OUT_OF_RANGE (-1)

typedef struct{
    int16_t mwradardetectionConeDeciDegrees; // detection cone angle as in mwradar device spec
    int16_t mwradardetectionConeExtendedDeciDegrees; // device spec is conservative, in practice have slightly larger detection cone

	int16_t mwradarMaxRangeCm;
	int16_t mwradarMaxAltWithTiltCm;
	int16_t mwradarCfAltCm; // Complimentary Filter altitude
	int16_t mwradarMaxTiltDeciDegrees;
	float mwradarMaxTiltCos;

	int32_t calculatedAltitude;
}mwradar_t;

extern mwradar_t mwradar;

void mwradarUpdate(void);
int32_t mwradarRead(void);
int32_t mwradarReadRaw(void);
int32_t mwradarCalculateAltitude(int32_t mwradarDistance, float cosTiltAngle);
int32_t mwradarGetLatestAltitude(void);
bool ismwradarWorkFind(void);
