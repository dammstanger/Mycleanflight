/*
 * mwradar.c
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
#include "common/filter.h"

#include "config/parameter_group.h"
#include "config/feature.h"

#include "io/mwradar_zb.h"

#include "fc/runtime_config.h"
#include "fc/config.h"

#include "sensors/mwradar.h"
#include "sensors/sensors.h"

#ifdef MWRADAR

mwradar_t mwradar;

void mwradarInit(void)
{
#ifdef USE_ZB005
	zbWmInit(&mwradar);
#endif
    sensorsSet(SENSOR_MWRADAR);						//使能微波测距传感器
    mwradar.mwradarCfAltCm = mwradar.mwradarMaxRangeCm / 2;
    mwradar.mwradarMaxTiltDeciDegrees =  mwradar.mwradardetectionConeExtendedDeciDegrees / 2;
    mwradar.mwradarMaxTiltCos = cos_approx(mwradar.mwradarMaxTiltDeciDegrees / 10.0f * RAD);
    mwradar.mwradarMaxAltWithTiltCm = mwradar.mwradarMaxRangeCm * mwradar.mwradarMaxTiltCos;

}

#define DISTANCE_SAMPLES_MEDIAN_ZB 5

static int32_t applyMwradarMedianFilter(int32_t newradarReading)
{
    static int32_t radarFilterSamples[DISTANCE_SAMPLES_MEDIAN_ZB];
    static int currentFilterSampleIndex = 0;
    static bool medianFilterReady = false;			//用于滤波器等待足够的有效数据后出发工作
    int nextSampleIndex;

    if (newradarReading > MWRADAR_OUT_OF_RANGE) // only accept samples that are in range
    {
        nextSampleIndex = (currentFilterSampleIndex + 1);
        if (nextSampleIndex == DISTANCE_SAMPLES_MEDIAN_ZB) {
            nextSampleIndex = 0;
            medianFilterReady = true;
        }

        radarFilterSamples[currentFilterSampleIndex] = newradarReading;
        currentFilterSampleIndex = nextSampleIndex;
    }
    if (medianFilterReady)
        return quickMedianFilter5(radarFilterSamples);
    else
        return newradarReading;
}

void mwradarUpdate(void)
{
	//对于不能自动连续测量回传数据的模块，使用命令触发
#ifdef USE_ZB005

	zbMwSensorWorkChk();
#endif
}


bool ismwradarWorkFind()
{
#ifdef USE_ZB005
	return isZbMwWorkFind();
#endif
}

//方向选择性低通滤波器，相对高度上升时与相对高度下降时参数2*f不同
#define RISE_2F 1.5f
#define DEC_2F 1.5f	//下降时截止频率高
int32_t sectionlpf(int32_t datin,float dt)
{
	static float dat_lpf;
	float dalta = datin-dat_lpf;
	if(dalta<=0)
		dat_lpf = dat_lpf + (1/(1+1/(DEC_2F*M_PIf*dt)))*dalta;
	else
		dat_lpf = dat_lpf + (1/(1+1/(RISE_2F*M_PIf*dt)))*dalta;
	return (int32_t)(dat_lpf+0.5);											//四舍五入
}

/**
 * Get the last distance measured by the irrangefinder in centimeters. When the ground is too far away, SONAR_OUT_OF_RANGE is returned.
 */
int32_t mwradarRead(void)
{
    int32_t distance = zbMw_get_distance();
    if (distance > mwradar.mwradarMaxRangeCm)
        distance = MWRADAR_OUT_OF_RANGE;

//    pt1Filter_t radarlpf;
//    return (int32_t)(pt1FilterApply4(&radarlpf,distance,1.0,0.05)+0.5);	//4舍5入
    return applyMwradarMedianFilter(distance);
//    if(distance!=MWRADAR_OUT_OF_RANGE)
//    	return sectionlpf(distance, 0.05);
//    else return distance;
}

int32_t mwradarReadRaw(void)
{
    int32_t distance = zbMw_get_distance();
    if (distance > mwradar.mwradarMaxRangeCm)
        distance = MWRADAR_OUT_OF_RANGE;

    if(distance!=MWRADAR_OUT_OF_RANGE)
    	return distance;
}

/**
 * Apply tilt correction to the given raw sonar reading in order to compensate for the tilt of the craft when estimating
 * the altitude. Returns the computed altitude in centimeters.
 *
 * When the ground is too far away or the tilt is too large, IRRANGFD_OUT_OF_RANGE is returned.
 */
int32_t mwradarCalculateAltitude(int32_t mwradarDistance, float cosTiltAngle)
{
    // calculate sonar altitude only if the ground is in the mwradar cone
    if (cosTiltAngle <= mwradar.mwradarMaxTiltCos)
    	mwradar.calculatedAltitude = MWRADAR_OUT_OF_RANGE;
    else
        // altitude = distance * cos(tiltAngle), use approximation
    	mwradar.calculatedAltitude = mwradarDistance * cosTiltAngle;
    return mwradar.calculatedAltitude;
}

/**
 * Get the latest altitude that was computed by a call to mwradarCalculateAltitude(), or IRRANGFD_OUT_OF_RANGE if sonarCalculateAltitude
 * has never been called.
 */
int32_t mwradarGetLatestAltitude(void)
{
    return mwradar.calculatedAltitude;
}


#endif
