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
#include <stdlib.h>
#include <math.h>


#include <platform.h>

#include "build/debug.h"

#include "common/maths.h"
#include "common/axis.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/sonar_hcsr04.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/sonar.h"
#include "sensors/irrangefinder.h"
#include "sensors/mwrader.h"

#include "rx/rx.h"

#include "io/motors.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/fc_debug.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"

#include "flight/altitudehold.h"

int32_t setVelocity = 0;
uint8_t velocityControl = 0;
int32_t errorVelocityI = 0;
int32_t altHoldThrottleAdjustment = 0;
int32_t AltHold;
int32_t vario = 0;                      // variometer in cm/s


#if defined(BARO) || defined(SONAR) || defined(IRRANGFD) || defined(MWRADER)

static int16_t initialRawThrottleHold;
static int16_t initialThrottleHold;
static int32_t EstAlt;                // in cm

PG_REGISTER_WITH_RESET_TEMPLATE(airplaneConfig_t, airplaneConfig, PG_AIRPLANE_ALT_HOLD_CONFIG, 0);

PG_RESET_TEMPLATE(airplaneConfig_t, airplaneConfig,
    .fixedwing_althold_dir = 1,
);

// 40hz update rate (20hz LPF on acc)
#define BARO_UPDATE_FREQUENCY_40HZ (1000 * 25)

#define DEGREES_80_IN_DECIDEGREES 800

static int32_t AltHold_debug = 0;
static int32_t setVelocity_debug = 0;
static uint8_t isAltHoldChanged = 0;
static void applyMultirotorAltHold(void)
{
//#ifdef MWRADER
//    int32_t rela_alt;
//
//#endif
    // multirotor alt hold
    if (rcControlsConfig()->alt_hold_fast_change) {		//本质是死区内使用高度位置保持，死去外直接油门控制
        // rapid alt changes
        if (ABS(rcData[THROTTLE] - initialRawThrottleHold) > rcControlsConfig()->alt_hold_deadband) {
            errorVelocityI = 0;
            isAltHoldChanged = 1;
            rcCommand[THROTTLE] += (rcData[THROTTLE] > initialRawThrottleHold) ? -rcControlsConfig()->alt_hold_deadband : rcControlsConfig()->alt_hold_deadband;
        } else {
            if (isAltHoldChanged) {
                AltHold = EstAlt;
                isAltHoldChanged = 0;
            }
            //
            rcCommand[THROTTLE] = constrain(initialThrottleHold + altHoldThrottleAdjustment, motorConfig()->minthrottle, motorConfig()->maxthrottle);
        }
    } else {
        // slow alt changes, mostly used for aerial photography
        if (ABS(rcData[THROTTLE] - initialRawThrottleHold) > rcControlsConfig()->alt_hold_deadband) {
        // 设置初始目标速度为+-20cm/s
        	if(rcData[THROTTLE]>initialRawThrottleHold)
        		setVelocity = (rcData[THROTTLE] - (initialRawThrottleHold + rcControlsConfig()->alt_hold_deadband)+20);
        	else
        		setVelocity = (rcData[THROTTLE] - (initialRawThrottleHold - rcControlsConfig()->alt_hold_deadband)-20);
            velocityControl = 1;
            isAltHoldChanged = 1;
        } else if (isAltHoldChanged) {									//当油门在死区内时，isAltHoldChanged可使清零只执行一次
            AltHold = EstAlt;
            velocityControl = 0;
            isAltHoldChanged = 0;
        }
        setVelocity_debug = setVelocity;
        AltHold_debug = AltHold;
        //进入高度保持模式的手动油门值 + 高度保持控制器输出的油门控制量
        rcCommand[THROTTLE] = constrain(initialThrottleHold + altHoldThrottleAdjustment, motorConfig()->minthrottle, motorConfig()->maxthrottle);
    }
}

static void applyFixedWingAltHold(void)
{
    // handle fixedwing-related althold. UNTESTED! and probably wrong
    // most likely need to check changes on pitch channel and 'reset' althold similar to
    // how throttle does it on multirotor

    rcCommand[PITCH] += altHoldThrottleAdjustment * airplaneConfig()->fixedwing_althold_dir;
}

void applyAltHold(void)
{
    if (STATE(FIXED_WING)) {
        applyFixedWingAltHold();
    } else {
        applyMultirotorAltHold();
    }
}

void updateAltHoldState(void)
{
    // Baro alt hold activate
    if (!rcModeIsActive(BOXBARO)) {
        DISABLE_FLIGHT_MODE(BARO_MODE);
        return;
    }

    if (!FLIGHT_MODE(BARO_MODE)) {
        ENABLE_FLIGHT_MODE(BARO_MODE);
        AltHold = EstAlt;
        initialRawThrottleHold = rcData[THROTTLE];
        initialThrottleHold = rcCommand[THROTTLE];
        errorVelocityI = 0;
        altHoldThrottleAdjustment = 0;
    }
}

void updateSonarAltHoldState(void)
{
    // Sonar alt hold activate
    if (!rcModeIsActive(BOXSONAR)) {
        DISABLE_FLIGHT_MODE(SONAR_MODE);
        return;
    }

    if (!FLIGHT_MODE(SONAR_MODE)) {
        ENABLE_FLIGHT_MODE(SONAR_MODE);
        AltHold = EstAlt;
        initialRawThrottleHold = rcData[THROTTLE];
        initialThrottleHold = rcCommand[THROTTLE];
        errorVelocityI = 0;
        altHoldThrottleAdjustment = 0;
    }
}

void updateIRrangfdAltHoldState(void)
{
    // IRrangefinder alt hold activate
    if (!rcModeIsActive(BOXIRRANGFD)) {
        DISABLE_FLIGHT_MODE(IRRANGFD_MODE);
        return;
    }

    if (!FLIGHT_MODE(IRRANGFD_MODE)) {
        ENABLE_FLIGHT_MODE(IRRANGFD_MODE);
        AltHold = EstAlt;
        initialRawThrottleHold = rcData[THROTTLE];
        initialThrottleHold = rcCommand[THROTTLE];
        errorVelocityI = 0;
        altHoldThrottleAdjustment = 0;
    }
}

void updateMwraderAltHoldState(void)
{
    // rader alt hold activate
    if (!rcModeIsActive(BOXMWRADER)) {
        DISABLE_FLIGHT_MODE(MWRADER_MODE);
        return;
    }

    if (!FLIGHT_MODE(MWRADER_MODE)) {
        ENABLE_FLIGHT_MODE(MWRADER_MODE);
        AltHold = EstAlt;
        initialRawThrottleHold = rcData[THROTTLE];
        initialThrottleHold = rcCommand[THROTTLE];
        errorVelocityI = 0;
        altHoldThrottleAdjustment = 0;
    }
}


bool isThrustFacingDownwards(attitudeEulerAngles_t *attitude)
{
    return ABS(attitude->values.roll) < DEGREES_80_IN_DECIDEGREES && ABS(attitude->values.pitch) < DEGREES_80_IN_DECIDEGREES;
}

int32_t calculateAltHoldThrottleAdjustment(int32_t vel_tmp, float accZ_tmp, float accZ_old)
{
    int32_t result = 0;
    int32_t error;
    int32_t setVel;

    if (!isThrustFacingDownwards(&attitude)) {			//如果飞行器翻转飞行，油门不是使推力向下，则
        return result;
    }

    // Altitude P-Controller

    if (!velocityControl) {
        error = constrain(AltHold - EstAlt, -500, 500);
        error = applyDeadband(error, 10); // remove small P parameter to reduce noise near zero position
        setVel = constrain((pidProfile()->P8[PIDALT] * error / 128), -300, +300); // limit velocity to +/- 3 m/s
    } else {
        setVel = setVelocity;
    }

    // Velocity PID-Controller
    // P
    error = setVel - vel_tmp;
    result = constrain((pidProfile()->P8[PIDVEL] * error / 32), -300, +300);

    // I
    errorVelocityI += (pidProfile()->I8[PIDVEL] * error);
    errorVelocityI = constrain(errorVelocityI, -(8192 * 200), (8192 * 200));
    result += errorVelocityI / 8192;     // I in range +/-200

    // D
    result -= constrain(pidProfile()->D8[PIDVEL] * (accZ_tmp + accZ_old) / 512, -150, 150);

    return result;
}


static float altimu_debug = 0.0f;
static float velimu_debug = 0.0f;
static float velcf_debug = 0.0f;
static float noneimuVel_debug = 0.0f;
static int32_t BaroAlt_debug = 0;
static int32_t mwraderAlt_debug = 0;

#define WAVERANGE 50				//cm
void calculateEstimatedAltitude(uint32_t currentTime)
{
    static uint32_t previousTime;
    uint32_t dTime;
    static int32_t noneimuvel;
    float dt;
    float vel_acc;
    int32_t vel_tmp;
    float accZ_tmp;
    static float accZ_old = 0.0f;
    static float imuVel = 0.0f;
    static float estVel = 0.0f;
    static float accAlt = 0.0f;
    static int32_t EstAlt_tmp = 0;
    static int32_t EstAlt_tmp_last;
    static uint8_t isAltHoldChanged_last = 0;

    float dx;

#ifdef SONAR
    int32_t sonarAlt = SONAR_OUT_OF_RANGE;
    static int32_t baroAlt_offset = 0;
    float sonarTransition;

#elif defined(IRRANGFD)		// dammstange 20170705
    int32_t irrangfdAlt = IRRANGFD_OUT_OF_RANGE;
    static int32_t baroAlt_offset = 0;
    float irrangfdTransition;
#elif defined(MWRADER)

    int32_t mwraderAlt = MWRADER_OUT_OF_RANGE;
    int32_t mwraderAltRaw = MWRADER_OUT_OF_RANGE;
    static int32_t BaroAlt_rela = 15;			//初始化雷达地面高度
    static int32_t baroAlt_offset = 0;
    static int32_t EstAlt_offset = 0;
    float mwraderTransition;
    static u8 isRaderStartWave = 0;				//初始摆动标记
#endif

    dTime = currentTime - previousTime;			//单位us
    if (dTime < BARO_UPDATE_FREQUENCY_40HZ)
        return;

    previousTime = currentTime;

#ifdef BARO
    if (!isBaroCalibrationComplete()) {
        performBaroCalibrationCycle();
        estVel = 0;
        imuVel = 0;
        accAlt = 0;
    }
	#ifdef MWRADER
	    baroCalculateAltitude();
	    EstAlt_tmp = BaroAlt;
		baroCalculateDaltaAlt(&BaroAlt_rela);
		BaroAlt_debug = BaroAlt;
		if (debugMode == DEBUG_MWRADER)
		{
			debug[0] = BaroAlt_rela;
		}
	#else
		EstAlt_tmp = baroCalculateAltitude();
	#endif
#else
    EstAlt_tmp = 0;
#endif

#ifdef SONAR
    sonarAlt = sonarRead();
    sonarAlt = sonarCalculateAltitude(sonarAlt, getCosTiltAngle());

    if (sonarAlt > 0 && sonarAlt < sonarCfAltCm) {
        // just use the SONAR
        baroAlt_offset = EstAlt_tmp - sonarAlt;
        EstAlt_tmp = sonarAlt;
    } else {
        EstAlt_tmp -= baroAlt_offset;
        if (sonarAlt > 0  && sonarAlt <= sonarMaxAltWithTiltCm) {
            // SONAR in range, so use complementary filter
            sonarTransition = (float)(sonarMaxAltWithTiltCm - sonarAlt) / (sonarMaxAltWithTiltCm - sonarCfAltCm);
            EstAlt_tmp = sonarAlt * sonarTransition + EstAlt_tmp * (1.0f - sonarTransition);
        }
    }
    //dammstanger 20170705
#elif defined(IRRANGFD)
    if(isIRrangfdWorkFind()==true){
		irrangfdAlt = irrangfdRead();
		if (debugMode == DEBUG_IRRANGFD)
		{
			debug[1] = irrangfdAlt;
		}
		irrangfdAltRaw_debug = irrangfdAlt;
		irrangfdAlt = irrangfdCalculateAltitude(irrangfdAlt, getCosTiltAngle());
		irrangfdAlt_debug = irrangfdAlt;
    }
    else{
    	irrangfdAlt = 0;
    }

	if (debugMode == DEBUG_IRRANGFD)
	{
		debug[2] = irrangfdAlt;
	}

    if (irrangfdAlt > 0 && irrangfdAlt < irrangfd.irrangfdCfAltCm) {
        // just use the IRrangefinder
        baroAlt_offset = EstAlt_tmp - irrangfdAlt;
        EstAlt_tmp = irrangfdAlt;
    } else {
        EstAlt_tmp -= baroAlt_offset;
        if (irrangfdAlt > 0  && irrangfdAlt <= irrangfd.irrangfdMaxAltWithTiltCm) {
            // IRrangefinder in range, so use complementary filter
        	irrangfdTransition = (float)(irrangfd.irrangfdMaxAltWithTiltCm - irrangfdAlt) / (irrangfd.irrangfdMaxAltWithTiltCm - irrangfd.irrangfdCfAltCm);
            EstAlt_tmp = irrangfdAlt * irrangfdTransition + EstAlt_tmp * (1.0f - irrangfdTransition);
        }
    }
#elif defined(MWRADER)
    if(ismwraderWorkFind()==true)
    {
    	mwraderAlt = mwraderRead();
    	mwraderAlt_debug = mwraderAlt;												//debug
	    mwraderAlt = mwraderCalculateAltitude(mwraderAlt, getCosTiltAngle());

	    mwraderAltRaw = mwraderReadRaw();
	}
	else{
		mwraderAlt = 0;
	}

    if(isAltHoldChanged)
    	if(mwraderAltRaw<mwrader.mwraderMaxRangeCm){
    		EstAlt_tmp = mwraderAlt;							//回到量程范围且是下降过程需要把高度差消除
    	}
    	isAltHoldChanged_last = isAltHoldChanged;

	if (mwraderAlt > 0 && mwraderAlt < mwrader.mwraderCfAltCm) {
		// just use the IRrangefinder
		baroAlt_offset = EstAlt_tmp - mwraderAlt;
		EstAlt_tmp = mwraderAlt;
	} else {
		EstAlt_tmp -= baroAlt_offset;
		if (mwraderAlt > 0 && mwraderAlt <= mwrader.mwraderMaxAltWithTiltCm) {
			// rader in range, so use complementary filter
			mwraderTransition = (float)(mwrader.mwraderMaxAltWithTiltCm - mwraderAlt) / (mwrader.mwraderMaxAltWithTiltCm - mwrader.mwraderCfAltCm);
			EstAlt_tmp = mwraderAlt * mwraderTransition + EstAlt_tmp * (1.0f - mwraderTransition);
		}
	}

//    if(mwraderAlt>0 && mwraderAlt < mwrader.mwraderMaxAltWithTiltCm){
////    	baroAlt_offset = BaroAlt - mwraderAlt;
//    	EstAlt_offset = EstAlt_tmp - mwraderAltRaw;
//	    if (debugMode == DEBUG_MWRADER)
//	    {
//	        debug[2] = EstAlt_offset;
//	    }
//    	if(EstAlt_offset < -WAVERANGE){
//    		if(isRaderStartWave==0){
//    			isRaderStartWave = 1;
//    			BaroAlt_rela = EstAlt_tmp;
//    		}
//    			EstAlt_tmp = BaroAlt_rela;
//    	}
//
//    	else if(EstAlt_offset>=-WAVERANGE && EstAlt_offset<0){
//    		isRaderStartWave = 0;
//			mwraderTransition = -(float)EstAlt_offset / WAVERANGE;
//			EstAlt_tmp = mwraderAlt * mwraderTransition + EstAlt_tmp  * (1.0f - mwraderTransition);
//    	}
//       	else{
//        		isRaderStartWave = 0;
//        		EstAlt_tmp = mwraderAlt;
//        	}
//    }
//    else{
//    	EstAlt_tmp = BaroAlt_rela;
//    }

    if (debugMode == DEBUG_MWRADER)
    {
        debug[3] = EstAlt_tmp;
    }
#endif

    dt = accTimeSum * 1e-6f; // delta acc reading time in seconds

    // Integrator - velocity, cm/sec
    if (accSumCount) {
        accZ_tmp = (float)accSum[2] / (float)accSumCount;		//多次的平均值
    } else {
        accZ_tmp = 0;
    }
    vel_acc = accZ_tmp * accVelScale * (float)accTimeSum;		//accTimeSum的acc积分出来的垂直速度 dv= a*dt

    // Integrator - Altitude in cm								位移公式由acc计算得到的垂直位置
    dx = (vel_acc * 0.5f) * dt + estVel * dt;            		// integrate velocity to get distance dx = (1/2)*a*dt^2 + v0*dt  alt = alt +dx
    accAlt += dx;
    altimu_debug += dx;
#ifdef BARO
    accAlt = accAlt * barometerConfig()->baro_cf_alt + (float)EstAlt_tmp * (1.0f - barometerConfig()->baro_cf_alt);    // complementary filter for altitude estimation (baro & acc)
#endif															//baro_cf_alt默认0.965
    imuVel += vel_acc;						// v = v + dv 累加出速度
    velimu_debug = imuVel;
#ifdef DEBUG_ALT_HOLD
    debug[1] = accSum[2] / accSumCount; // acceleration
    debug[2] = vel;                     // velocity
    debug[3] = accAlt;                  // height
#endif

    imuResetAccelerationSum();			//使用acc累加的各种量后清零

#ifdef BARO
    if (!isBaroCalibrationComplete()) {
        return;
    }
#endif

#ifdef SONAR
    if (sonarAlt > 0 && sonarAlt < sonarCfAltCm) {
        // the sonar has the best range
        EstAlt = EstAlt_tmp;					//当声纳处于良好测量距离时，垂直位置只使用声呐数据
    } else {
        EstAlt = accAlt;					//否则使用acc积分、气压、声呐三者融合的数据
    }
#elif defined(IRRANGFD)
    if (irrangfdAlt > 0 && irrangfdAlt < irrangfd.irrangfdCfAltCm) {
        // the IRrangefinder has the best range
        EstAlt = EstAlt_tmp;					//当IRrangefinder处于良好测量距离时，垂直位置只使用IRrangefinder数据
    } else {
        EstAlt = accAlt;					//否则使用acc积分、气压、IR三者融合的数据
    }
#elif defined(MWRADER)
    if (mwraderAlt > 0 && mwraderAlt < mwrader.mwraderCfAltCm) {
        // the rader has the best range
        EstAlt = EstAlt_tmp;				//当rader处于良好测量距离时，垂直位置只使用rader数据
    } else {
        EstAlt = accAlt;					//否则使用acc积分、气压、IR三者融合的数据
    }
    BaroAlt = EstAlt;
#else
    EstAlt = accAlt;
#endif

    int32_t d_EstAlt_tmp = EstAlt_tmp - EstAlt_tmp_last;
    EstAlt_tmp_last = EstAlt_tmp;
    if(d_EstAlt_tmp<=20){														//如果高度数据跳跃超过极限，则速度使用历史值
    	noneimuvel = d_EstAlt_tmp * 1000000.0f / dTime;		//单位：cm/s
		noneimuvel = constrain(noneimuvel, -1500, 1500);  // constrain baro velocity +/- 1500cm/s
		noneimuvel = applyDeadband(noneimuvel, 10);       // to reduce noise near zero
    }
    noneimuVel_debug = noneimuvel;
    // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e也就是 without delay
#ifdef BARO
    estVel = imuVel * barometerConfig()->baro_cf_vel + noneimuvel * (1.0f - barometerConfig()->baro_cf_vel);			//baro_cf_vel默认0.985
#endif
    imuVel = estVel;					//
    velcf_debug = estVel;
    vel_tmp = lrintf(estVel);			//4舍5入取整

    // set vario
    vario = applyDeadband(vel_tmp, 5);

    altHoldThrottleAdjustment = calculateAltHoldThrottleAdjustment(vel_tmp, accZ_tmp, accZ_old);

    accZ_old = accZ_tmp;
}

int32_t altitudeHoldGetEstimatedAltitude(void)
{
    return EstAlt;
}

int32_t altitudeGetImuBasedAlt(void)
{
	return (int32_t)altimu_debug;
}


int32_t altitudeGetImuBasedVel(void)
{
	return (int32_t)velimu_debug;
}

int32_t altitudeGetCfVel(void)
{
	return (int32_t)velcf_debug;
}

int16_t altitudeGetMwraderAlt(void)
{
 return mwraderAlt_debug;
}

int32_t altitudeGetNoneImuVel(void)
{
	return noneimuVel_debug;
}
int32_t altitudeGetBaroAlt(void)
{
	return BaroAlt_debug;
}

int32_t altitudeGetAltHold(void)
{
	return AltHold_debug;
}

int32_t altitudeGetsetVel(void)
{
	return setVelocity_debug;
}

#endif

