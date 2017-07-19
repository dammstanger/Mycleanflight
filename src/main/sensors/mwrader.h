/*
 * mwrader.h
 *
 *  Created on: 2017年7月18日
 *      Author: DammStanger
 */

#pragma once

#define MWRADER_OUT_OF_RANGE (-1)

typedef struct{
    int16_t mwraderdetectionConeDeciDegrees; // detection cone angle as in mwrader device spec
    int16_t mwraderdetectionConeExtendedDeciDegrees; // device spec is conservative, in practice have slightly larger detection cone

	int16_t mwraderMaxRangeCm;
	int16_t mwraderMaxAltWithTiltCm;
	int16_t mwraderCfAltCm; // Complimentary Filter altitude
	int16_t mwraderMaxTiltDeciDegrees;
	float mwraderMaxTiltCos;

	int32_t calculatedAltitude;
}mwrader_t;

extern mwrader_t mwrader;

void mwraderUpdate(void);
int32_t mwraderRead(void);
int32_t mwraderCalculateAltitude(int32_t irrangfdDistance, float cosTiltAngle);
int32_t mwraderGetLatestAltitude(void);
bool ismwraderWorkFind(void);
