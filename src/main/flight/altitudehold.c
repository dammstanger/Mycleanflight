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
#include "sensors/mwradar.h"

#include "rx/rx.h"

#include "io/motors.h"

#include "fc/alt_transcurve.h"
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


#if defined(BARO) || defined(SONAR) || defined(IRRANGFD) || defined(MWRADAR)

static int16_t initialRawThrottleHold;
static int16_t initialThrottleHold;
static int32_t EstAlt;                	// in cm
static float estVel = 0.0f;
static int32_t updateAltHoldflg = 0;	//用于中途自动跟新高度保持值

PG_REGISTER_WITH_RESET_TEMPLATE(airplaneConfig_t, airplaneConfig, PG_AIRPLANE_ALT_HOLD_CONFIG, 0);

PG_RESET_TEMPLATE(airplaneConfig_t, airplaneConfig,
    .fixedwing_althold_dir = 1,
);

// 40hz update rate (20hz LPF on acc)
#define BARO_UPDATE_FREQUENCY_40HZ (1000 * 25)

#define DEGREES_80_IN_DECIDEGREES 800

static int32_t AltHold_debug = 0;
static int32_t setVel_debug = 0;
static uint8_t isAltHoldChanged = 0;
static void applyMultirotorAltHold(void)
{
	static float estVel_last =0.0f;
	static u8 issetvelpos = 0;			//目标速度为正标记
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
            else if(updateAltHoldflg){
            	updateAltHoldflg = 0;
            	AltHold = EstAlt;
            }
            //
            rcCommand[THROTTLE] = constrain(initialThrottleHold + altHoldThrottleAdjustment, motorConfig()->minthrottle, motorConfig()->maxthrottle);
        }
    } else {
        // slow alt changes, mostly used for aerial photography
        if (ABS(rcData[THROTTLE] - initialRawThrottleHold) > rcControlsConfig()->alt_hold_deadband) {
        // 设置初始目标速度为+-20cm/s
        	if(rcData[THROTTLE]>initialRawThrottleHold){
        		setVelocity = (rcData[THROTTLE] - (initialRawThrottleHold + rcControlsConfig()->alt_hold_deadband)+20);
        		issetvelpos = 1;
        	}
        	else{
        		setVelocity = (rcData[THROTTLE] - (initialRawThrottleHold - rcControlsConfig()->alt_hold_deadband)-20);
        		issetvelpos = 0;
        	}
            velocityControl = 1;
            isAltHoldChanged = 1;
        } else if (isAltHoldChanged) {									//当油门在死区内时，isAltHoldChanged可使清零只执行一次
        	setVelocity = 0;											//在速度的回到0之前还没进入位置控制的时候 使目标速度为0
        	if((issetvelpos==1 && estVel<0 && estVel_last>0) ||
        	   (issetvelpos==0 && estVel>0 && estVel_last<0) ){			//如果速度过零了，则切换模式
				AltHold = EstAlt;
				velocityControl = 0;
				isAltHoldChanged = 0;
        	}
        }
        else if(updateAltHoldflg){
        	updateAltHoldflg = 0;
        	AltHold = EstAlt;
        }

        estVel_last = estVel;

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

void updateMwradarAltHoldState(void)
{
    // radar alt hold activate
    if (!rcModeIsActive(BOXMWRADAR)) {
        DISABLE_FLIGHT_MODE(MWRADAR_MODE);
        return;
    }

    if (!FLIGHT_MODE(MWRADAR_MODE)) {
        ENABLE_FLIGHT_MODE(MWRADAR_MODE);
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
        error = applyDeadband(error, 5); // remove small P parameter to reduce noise near zero position  default 10
        setVel = constrain((pidProfile()->P8[PIDALT] * error / 128), -20, +20); // limit velocity to + 0.2 m/s  -0.2m  default: +-3m/s 300
    } else {
        setVel = setVelocity;
    }
    setVel_debug = setVel;
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
static int32_t mwradarAlt_debug = 0;
static int32_t BaroAlt_rela = 15;			//初始化雷达地面高度
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
#elif defined(MWRADAR)

    int32_t mwradarAlt = MWRADAR_OUT_OF_RANGE;
    int32_t mwradarAltRaw = MWRADAR_OUT_OF_RANGE;
    static int32_t baroAlt_offset = 0;
    float mwradarTransition;
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

	#ifdef MWRADAR
		if(ARMING_FLAG(ARMED))
			baroCalculateDaltaAlt(&BaroAlt_rela);
		else
			BaroAlt_rela = EstAlt_tmp;

		if (debugMode == DEBUG_MWRADAR)
		{
			debug[0] = BaroAlt_rela;
		}
	#endif

	baroCalculateAltitude();
	BaroAlt_debug = BaroAlt;
	EstAlt_tmp = BaroAlt;

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
#elif defined(MWRADAR)
    if(ismwradarWorkFind()==true)
    {
    	mwradarAlt = mwradarRead();
    	mwradarAlt_debug = mwradarAlt;												//debug
//	    mwradarAlt = mwradarCalculateAltitude(mwradarAlt, getCosTiltAngle());

	    mwradarAltRaw = mwradarReadRaw();
	}
	else{
		mwradarAlt = 0;
	}

	if (mwradarAlt > 0 && mwradarAlt < mwradar.mwradarCfAltCm) {
		// just use the IRrangefinder
		baroAlt_offset = EstAlt_tmp - mwradarAlt;
		EstAlt_tmp = mwradarAlt;
	} else {
		EstAlt_tmp -= baroAlt_offset;
		if (mwradarAlt > 0 && mwradarAlt <= mwradar.mwradarMaxAltWithTiltCm) {
	    	if(isAltHoldChanged){
	    		EstAlt_tmp = mwradarAlt;							//回到量程范围且是下降过程需要把高度差消除
	    		accAlt = mwradarAlt;
	    		baroAlt_offset = 0;
	    	}
	    	else if((EstAlt_tmp - mwradarAlt)>20){
	    		EstAlt_tmp = mwradarAlt;							//回到量程范围且是下降过程需要把高度差消除
	    		accAlt = mwradarAlt;
	    		baroAlt_offset = 0;
	    		updateAltHoldflg = 1;
	    	}

			// radar in range, so use complementary filter
//			mwradarTransition = (float)(mwradar.mwradarMaxAltWithTiltCm - mwradarAlt) / (mwradar.mwradarMaxAltWithTiltCm - mwradar.mwradarCfAltCm);
//	    	mwradarTransition = altLookup(mwradarAlt-mwradar.mwradarCfAltCm);
	    	mwradarTransition = 1.0f;
			EstAlt_tmp = mwradarAlt * (1.0f - mwradarTransition) + EstAlt_tmp * mwradarTransition;
		}
	}

    if (debugMode == DEBUG_MWRADAR)
    {
        debug[2] = EstAlt_tmp;
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
    altimu_debug = accAlt;
#ifdef BARO
    accAlt = accAlt * barometerConfig()->baro_cf_alt + (float)EstAlt_tmp * (1.0f - barometerConfig()->baro_cf_alt);    // complementary filter for altitude estimation (baro & acc)
#endif															//baro_cf_alt默认0.965
    imuVel += vel_acc;						// v = v + dv 累加出速度

    velimu_debug = imuVel;


if (debugMode == DEBUG_ALT_HOLD){
    debug[1] = accSum[2] / accSumCount; // acceleration
    debug[2] = imuVel;                  // velocity
    debug[3] = accAlt;                  // height
}

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
#elif defined(MWRADAR)
    if (mwradarAlt > 0 && mwradarAlt < mwradar.mwradarCfAltCm) {
        // the radar has the best range
        EstAlt = EstAlt_tmp;				//当radar处于良好测量距离时，垂直位置只使用radar数据
        accAlt = EstAlt_tmp;				//
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

    	noneimuVel_debug = noneimuvel;
        if (debugMode == DEBUG_MWRADAR)
        {
            debug[3] = noneimuvel;
        }

		noneimuvel = constrain(noneimuvel, -1500, 1500);  // constrain baro velocity +/- 1500cm/s
		noneimuvel = applyDeadband(noneimuvel, 10);       // to reduce noise near zero
    }

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

int16_t altitudeGetMwradarAlt(void)
{
 return mwradarAlt_debug;
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
	return setVel_debug;
}

int32_t altitudeGetBaroRelaAlt(void)
{
	return BaroAlt_rela;
}

#endif

