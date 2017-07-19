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

#define DISTANCE_SAMPLES_MEDIAN_ZB 5

static int32_t applyMwraderMedianFilter(int32_t newraderReading)
{
    static int32_t raderFilterSamples[DISTANCE_SAMPLES_MEDIAN_ZB];
    static int currentFilterSampleIndex = 0;
    static bool medianFilterReady = false;			//用于滤波器等待足够的有效数据后出发工作
    int nextSampleIndex;

    if (newraderReading > MWRADER_OUT_OF_RANGE) // only accept samples that are in range
    {
        nextSampleIndex = (currentFilterSampleIndex + 1);
        if (nextSampleIndex == DISTANCE_SAMPLES_MEDIAN_ZB) {
            nextSampleIndex = 0;
            medianFilterReady = true;
        }

        raderFilterSamples[currentFilterSampleIndex] = newraderReading;
        currentFilterSampleIndex = nextSampleIndex;
    }
    if (medianFilterReady)
        return quickMedianFilter5(raderFilterSamples);
    else
        return newraderReading;
}

void mwraderUpdate(void)
{
	//对于不能自动连续测量回传数据的模块，使用命令触发
#ifdef USE_ZB005
	zbMwWrtCmd();
	zbMwSensorWorkChk();
#endif
}


bool ismwraderWorkFind()
{
#ifdef USE_ZB005
	return isZbMwWorkFind();
#endif
}

/**
 * Get the last distance measured by the irrangefinder in centimeters. When the ground is too far away, SONAR_OUT_OF_RANGE is returned.
 */
int32_t mwraderRead(void)
{
    int32_t distance = zbMw_get_distance();
    if (distance > mwrader.mwraderMaxRangeCm)
        distance = MWRADER_OUT_OF_RANGE;

    return applyMwraderMedianFilter(distance);
}

/**
 * Apply tilt correction to the given raw sonar reading in order to compensate for the tilt of the craft when estimating
 * the altitude. Returns the computed altitude in centimeters.
 *
 * When the ground is too far away or the tilt is too large, IRRANGFD_OUT_OF_RANGE is returned.
 */
int32_t mwraderCalculateAltitude(int32_t mwraderDistance, float cosTiltAngle)
{
    // calculate sonar altitude only if the ground is in the mwrader cone
    if (cosTiltAngle <= mwrader.mwraderMaxTiltCos)
    	mwrader.calculatedAltitude = MWRADER_OUT_OF_RANGE;
    else
        // altitude = distance * cos(tiltAngle), use approximation
    	mwrader.calculatedAltitude = mwraderDistance * cosTiltAngle;
    return mwrader.calculatedAltitude;
}

/**
 * Get the latest altitude that was computed by a call to mwraderCalculateAltitude(), or IRRANGFD_OUT_OF_RANGE if sonarCalculateAltitude
 * has never been called.
 */
int32_t mwraderGetLatestAltitude(void)
{
    return mwrader.calculatedAltitude;
}


#endif
