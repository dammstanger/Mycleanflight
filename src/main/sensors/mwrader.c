/*
 * mwrader.c
 *
 *  Created on: 2017年7月18日
 *      Author: DammStanger
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include <platform.h>
#include "build/build_config.h"

#include "common/maths.h"
#include "common/axis.h"

#include "config/parameter_group.h"
#include "config/feature.h"

#include "io/mwrader_zb.h"

#include "fc/runtime_config.h"
#include "fc/config.h"

#include "sensors/irrangefinder.h"
#include "sensors/sensors.h"

#ifdef MWRADER

mwrader_t mwrader;

void mwraderInit(void)
{
#ifdef USE_ZB005
	zbWmInit(&mwrader);
#endif
    sensorsSet(SENSOR_MWRADER);						//使能微波测距传感器
    mwrader.mwraderCfAltCm = mwrader.mwraderMaxRangeCm / 2;
    mwrader.mwraderMaxTiltDeciDegrees =  mwrader.mwraderdetectionConeExtendedDeciDegrees / 2;
    mwrader.mwraderMaxTiltCos = cos_approx(mwrader.mwraderMaxTiltDeciDegrees / 10.0f * RAD);
    mwrader.mwraderMaxAltWithTiltCm = mwrader.mwraderMaxRangeCm * mwrader.mwraderMaxTiltCos;

}


#endif
