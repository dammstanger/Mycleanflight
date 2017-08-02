/*
 * alt_transcurve.c
 *
 *  Created on: 2017年7月29日
 *      Author: DammStanger
 */

#include <stdbool.h>
#include <stdint.h>

#include "config/parameter_group.h"

#include "sensors/mwradar.h"

#include "fc/alt_transcurve.h"



#define ALTEXPO 11.0f
#define ALTRATE 0.01f

#define ALTTRANS_LOOKUP_LENGTH 10

static float lookupAltTransPar[ALTTRANS_LOOKUP_LENGTH+1];      // lookup table for expo & Altitude Transition Parameter
static int8_t section;

float altLookup(int alt)
{
    const int num = alt / section;
    return lookupAltTransPar[num] + (alt - num * section) * (lookupAltTransPar[num + 1] - lookupAltTransPar[num]) / section;
}


void generateAltTransCurve(int32_t range)
{
	section = (range + 10)/ALTTRANS_LOOKUP_LENGTH;		//+10让范围稍大不至于除后被缩小
    for (int i = 0; i <= ALTTRANS_LOOKUP_LENGTH; i++) {
    	lookupAltTransPar[i] = (100 + ALTEXPO * (i * i - 10)) * i * ALTRATE / 100.0;
    	if(lookupAltTransPar[i]<0) lookupAltTransPar[i] = 0;
    	else if(lookupAltTransPar[i]>1.0f) lookupAltTransPar[i] = 1.0f;
    }
}

