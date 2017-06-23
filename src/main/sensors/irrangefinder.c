/*
 * irrangefinder.c
 *
 *  Created on: 2017年6月20日
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

#include "io/irrangefinder_ptk.h"

#include "fc/runtime_config.h"
#include "fc/config.h"

#include "sensors/irrangefinder.h"
#include "sensors/sensors.h"

#ifdef IRRANGFD

irrangfd_t irrangfd;

void irrangfdInit(void)
{
#ifdef USE_PTK
    ptkIrInit(&irrangfd);
#endif
    sensorsSet(SENSOR_IRRANGFD);
    irrangfd.irrangfdCfAltCm = irrangfd.irrangfdMaxRangeCm / 2;
    irrangfd.irrangfdMaxTiltDeciDegrees =  irrangfd.irrangfddetectionConeExtendedDeciDegrees / 2;
    irrangfd.sonarMaxTiltCos = cos_approx(irrangfd.irrangfdMaxTiltDeciDegrees / 10.0f * RAD);
    irrangfd.irrangfdMaxAltWithTiltCm = irrangfd.irrangfdMaxRangeCm * irrangfd.sonarMaxTiltCos;

}

#define DISTANCE_SAMPLES_MEDIAN_PTK 5

static int32_t applyIrrangfdMedianFilter(int32_t newirrangfdReading)
{
    static int32_t irrangfdFilterSamples[DISTANCE_SAMPLES_MEDIAN_PTK];
    static int currentFilterSampleIndex = 0;
    static bool medianFilterReady = false;
    int nextSampleIndex;

    if (newirrangfdReading > IRRANGFD_OUT_OF_RANGE) // only accept samples that are in range
    {
        nextSampleIndex = (currentFilterSampleIndex + 1);
        if (nextSampleIndex == DISTANCE_SAMPLES_MEDIAN_PTK) {
            nextSampleIndex = 0;
            medianFilterReady = true;
        }

        irrangfdFilterSamples[currentFilterSampleIndex] = newirrangfdReading;
        currentFilterSampleIndex = nextSampleIndex;
    }
    if (medianFilterReady)
        return quickMedianFilter5(irrangfdFilterSamples);
    else
        return newirrangfdReading;
}

void irrangfdUpdate(void)
{
	//对于不能自动连续测量回传数据的模块，使用命令触发
#ifdef USE_PTK
	ptkWrtCmd();
#endif
}

/**
 * Get the last distance measured by the irrangefinder in centimeters. When the ground is too far away, SONAR_OUT_OF_RANGE is returned.
 */
int32_t irrangfdRead(void)
{
    int32_t distance = ptk_get_distance();
    if (distance > irrangfd.irrangfdMaxRangeCm)
        distance = IRRANGFD_OUT_OF_RANGE;

    return applyIrrangfdMedianFilter(distance);
}

/**
 * Apply tilt correction to the given raw sonar reading in order to compensate for the tilt of the craft when estimating
 * the altitude. Returns the computed altitude in centimeters.
 *
 * When the ground is too far away or the tilt is too large, IRRANGFD_OUT_OF_RANGE is returned.
 */
int32_t irrangfdCalculateAltitude(int32_t irrangfdDistance, float cosTiltAngle)
{
    // calculate sonar altitude only if the ground is in the irrangefinder cone
    if (cosTiltAngle <= irrangfd.sonarMaxTiltCos)
    	irrangfd.calculatedAltitude = IRRANGFD_OUT_OF_RANGE;
    else
        // altitude = distance * cos(tiltAngle), use approximation
    	irrangfd.calculatedAltitude = irrangfdDistance * cosTiltAngle;
    return irrangfd.calculatedAltitude;
}

/**
 * Get the latest altitude that was computed by a call to irrangfdCalculateAltitude(), or IRRANGFD_OUT_OF_RANGE if sonarCalculateAltitude
 * has never been called.
 */
int32_t irrangfdGetLatestAltitude(void)
{
    return irrangfd.calculatedAltitude;
}

#endif



