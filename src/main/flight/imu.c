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

// Inertial Measurement Unit (IMU)

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "common/maths.h"

#include "build/build_config.h"
#include <platform.h>
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/time.h"

#include "config/parameter_group_ids.h"
#include "config/parameter_group.h"
#include "config/config_reset.h"
#include "config/profile.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/sonar.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"

#include "io/gps.h"

#include "fc/runtime_config.h"
#include "fc/fc_debug.h"

// the limit (in degrees/second) beyond which we stop integrating
// omega_I. At larger spin rates the DCM PI controller can get 'dizzy'
// which results in false gyro drift. See
// http://gentlenav.googlecode.com/files/fastRotations.pdf
#define SPIN_RATE_LIMIT 20

//----iNAV----

#ifdef ASYNC_GYRO_PROCESSING
/* Asynchronous update accumulators */
volatile float imuAccumulatedRate[XYZ_AXIS_COUNT];
volatile timeUs_t imuAccumulatedRateTimeUs;
volatile float imuAccumulatedAcc[XYZ_AXIS_COUNT];
volatile int   imuAccumulatedAccCount;
#endif

t_fp_vector imuMeasuredAccelBF;
t_fp_vector imuMeasuredRotationBF;

//int32_t accSum[XYZ_AXIS_COUNT];

//uint32_t accTimeSum = 0;        // keep track for integration of acc
//int accSumCount = 0;
//float accVelScale;

float throttleAngleScale;
float fc_acc;
float smallAngleCosZ = 0;

static bool isAccelUpdatedAtLeastOnce = false;

static imuRuntimeConfig_t *imuRuntimeConfig;
static accDeadband_t *accDeadband;

PG_REGISTER_WITH_RESET_TEMPLATE(imuConfig_t, imuConfig, PG_IMU_CONFIG, 1);
PG_REGISTER_PROFILE_WITH_RESET_TEMPLATE(throttleCorrectionConfig_t, throttleCorrectionConfig, PG_THROTTLE_CORRECTION_CONFIG, 0);

PG_RESET_TEMPLATE(imuConfig_t, imuConfig,
    .dcm_kp_acc = 2500,                	// 1.0 * 10000
	.dcm_ki_acc = 50,					// 0.005 * 10000
    .dcm_kp_mag = 10000,               	// 1.0 * 10000
	.dcm_ki_mag = 0,					// 1.00 * 10000
    .small_angle = 25,
    .max_angle_inclination = 500,    // 50 degrees
);

PG_RESET_TEMPLATE(throttleCorrectionConfig_t, throttleCorrectionConfig,
    .throttle_correction_value = 0,      // could 10 with althold or 40 for fpv
    .throttle_correction_angle = 800,    // could be 80.0 deg with atlhold or 45.0 for fpv
);

STATIC_UNIT_TESTED float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;    // quaternion of sensor frame relative to earth frame
static float rMat[3][3];

attitudeEulerAngles_t attitude = { { 0, 0, 0 } };     // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800

static float gyroScale_Adc2Rad;

void imuTransformVectorBodyToEarth(t_fp_vector * v)
{
    /* From body frame to earth frame */
    const float x = rMat[0][0] * v->V.X + rMat[0][1] * v->V.Y + rMat[0][2] * v->V.Z;
    const float y = rMat[1][0] * v->V.X + rMat[1][1] * v->V.Y + rMat[1][2] * v->V.Z;
    const float z = rMat[2][0] * v->V.X + rMat[2][1] * v->V.Y + rMat[2][2] * v->V.Z;

    v->V.X = x;
    v->V.Y = -y;
    v->V.Z = z;
}

void imuTransformVectorEarthToBody(t_fp_vector * v)
{
    v->V.Y = -v->V.Y;

    /* From earth frame to body frame */
    const float x = rMat[0][0] * v->V.X + rMat[1][0] * v->V.Y + rMat[2][0] * v->V.Z;
    const float y = rMat[0][1] * v->V.X + rMat[1][1] * v->V.Y + rMat[2][1] * v->V.Z;
    const float z = rMat[0][2] * v->V.X + rMat[1][2] * v->V.Y + rMat[2][2] * v->V.Z;

    v->V.X = x;
    v->V.Y = y;
    v->V.Z = z;
}

STATIC_UNIT_TESTED void imuComputeRotationMatrix(void)
{
    float q1q1 = sq(q1);
    float q2q2 = sq(q2);
    float q3q3 = sq(q3);

    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

void imuConfigure(
    imuRuntimeConfig_t *initialImuRuntimeConfig,
    accDeadband_t *initialAccDeadband,
    float accz_lpf_cutoff,
    uint16_t throttle_correction_angle
)
{
    imuRuntimeConfig = initialImuRuntimeConfig;
    accDeadband = initialAccDeadband;
    fc_acc = calculateAccZLowPassFilterRCTimeConstant(accz_lpf_cutoff);
    throttleAngleScale = calculateThrottleAngleScale(throttle_correction_angle);
}

void imuInit(void)
{
    smallAngleCosZ = cos_approx(degreesToRadians(imuRuntimeConfig->small_angle));
    gyroScale_Adc2Rad = gyro.scale * (M_PIf / 180.0f);  // gyro output scaled to rad per second
//    accVelScale = GRAVITY_MS2 / acc.acc_1G / 10000.0f;

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        imuMeasuredAccelBF.A[axis] = 0;
    }

    imuComputeRotationMatrix();
}

float calculateThrottleAngleScale(uint16_t throttle_correction_angle)
{
    return (1800.0f / M_PIf) * (900.0f / throttle_correction_angle);
}

/*
* Calculate RC time constant used in the accZ lpf.
*/
float calculateAccZLowPassFilterRCTimeConstant(float accz_lpf_cutoff)
{
    return 0.5f / (M_PIf * accz_lpf_cutoff);
}

//void imuResetAccelerationSum(void)
//{
//    accSum[0] = 0;
//    accSum[1] = 0;
//    accSum[2] = 0;
//    accSumCount = 0;
//    accTimeSum = 0;
//}


//// rotate acc into Earth frame and calculate acceleration in it
//void imuCalculateAcceleration(uint32_t deltaT)
//{
//    static int32_t accZoffset = 0;
//    static float accz_smooth = 0;
//    float dT;
//    t_fp_vector accel_ned;
//
//    // deltaT is measured in us ticks
//    dT = (float)deltaT * 1e-6f;
//
//    accel_ned.V.X = accSmooth[0];			//单位依然是LSB
//    accel_ned.V.Y = accSmooth[1];
//    accel_ned.V.Z = accSmooth[2];
//
//    imuTransformVectorBodyToEarth(&accel_ned);
//
//    if (imuRuntimeConfig->acc_unarmedcal == 1) {			//??
//        if (!ARMING_FLAG(ARMED)) {
//            accZoffset -= accZoffset / 64;
//            accZoffset += accel_ned.V.Z;	//减去一个平均值再加上一个新的值，相当于64个值累加平均的效果
//            								//此过程要求水平放置 解锁 才准确
//        }
//        accel_ned.V.Z -= accZoffset / 64;  // compensate for gravitation on z-axis 减去z轴的重力加速度
//    } else
//        accel_ned.V.Z -= acc.acc_1G;
//
//    accz_smooth = accz_smooth + (dT / (fc_acc + dT)) * (accel_ned.V.Z - accz_smooth); // low pass filter fc_acc为时间常数
//
//    // apply Deadband to reduce integration drift and vibration influence
//    accSum[X] += applyDeadband(lrintf(accel_ned.V.X), accDeadband->xy);			//lrintf()四舍五入
//    accSum[Y] += applyDeadband(lrintf(accel_ned.V.Y), accDeadband->xy);
//    accSum[Z] += applyDeadband(lrintf(accz_smooth), accDeadband->z);
//
//    // sum up Values for later integration to get velocity and distance
//    accTimeSum += deltaT;
//    accSumCount++;
//}

static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

static bool imuUseFastGains(void)
{
    return !ARMING_FLAG(ARMED) && millis() < 20000;
}

static float imuGetPGainScaleFactor(void)
{
    if (imuUseFastGains()) {
        return 10.0f;
    }
    else {
        return 1.0f;
    }
}

static void imuResetOrientationQuaternion(const float ax, const float ay, const float az)
{
    const float accNorm = sqrtf(ax * ax + ay * ay + az * az);

    q0 = az + accNorm;
    q1 = ay;
    q2 = -ax;
    q3 = 0.0f;

    const float recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

static void imuCheckAndResetOrientationQuaternion(const float ax, const float ay, const float az)
{
    // Check if some calculation in IMU update yield NAN or zero quaternion
    // Reset quaternion from accelerometer - this might be incorrect, but it's better than no attitude at all

    const bool isNan = (isnan(q0) || isnan(q1) || isnan(q2) || isnan(q3));
    const bool isInf = (isinf(q0) || isinf(q1) || isinf(q2) || isinf(q3));
    const bool isZero = (ABS(q0) < 1e-3f && ABS(q1) < 1e-3f && ABS(q2) < 1e-3f && ABS(q3) < 1e-3f);

    if (isNan || isZero || isInf) {
        imuResetOrientationQuaternion(ax, ay, az);
    }
}

static void imuMahonyAHRSupdate(float dt, float gx, float gy, float gz,
                                bool useAcc, float ax, float ay, float az,
                                bool useMag, float mx, float my, float mz,
                                bool useCOG, float courseOverGround)
{
    static float integralAccX = 0.0f,  integralAccY = 0.0f, integralAccZ = 0.0f;    // integral error terms scaled by Ki
    static float integralMagX = 0.0f,  integralMagY = 0.0f, integralMagZ = 0.0f;    // integral error terms scaled by Ki
    float ex, ey, ez;

    /* Calculate general spin rate (rad/s) */
    const float spin_rate_sq = sq(gx) + sq(gy) + sq(gz);

    /* Step 1: Yaw correction */
    // Use measured magnetic field vector
    if (useMag || useCOG) {
        float kpMag = imuRuntimeConfig->dcm_kp_mag * imuGetPGainScaleFactor();
        const float magMagnitudeSq = mx * mx + my * my + mz * mz;

        if (useMag && magMagnitudeSq > 0.01f) {
            // Normalise magnetometer measurement
            const float magRecipNorm = invSqrt(magMagnitudeSq);
            mx *= magRecipNorm;
            my *= magRecipNorm;
            mz *= magRecipNorm;

            // For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
            // This way magnetic field will only affect heading and wont mess roll/pitch angles

            // (hx; hy; 0) - measured mag field vector in EF (assuming Z-component is zero)
            // (bx; 0; 0) - reference mag field vector heading due North in EF (assuming Z-component is zero)
            const float hx = rMat[0][0] * mx + rMat[0][1] * my + rMat[0][2] * mz;
            const float hy = rMat[1][0] * mx + rMat[1][1] * my + rMat[1][2] * mz;
            const float bx = sqrtf(hx * hx + hy * hy);

            // magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
            const float ez_ef = -(hy * bx);

            // Rotate mag error vector back to BF and accumulate
            ex = rMat[2][0] * ez_ef;
            ey = rMat[2][1] * ez_ef;
            ez = rMat[2][2] * ez_ef;
        }
        else if (useCOG) {
            // Use raw heading error (from GPS or whatever else)
            while (courseOverGround >  M_PIf) courseOverGround -= (2.0f * M_PIf);
            while (courseOverGround < -M_PIf) courseOverGround += (2.0f * M_PIf);

            // William Premerlani and Paul Bizard, Direction Cosine Matrix IMU - Eqn. 22-23
            // (Rxx; Ryx) - measured (estimated) heading vector (EF)
            // (cos(COG), sin(COG)) - reference heading vector (EF)
            // error is cross product between reference heading and estimated heading (calculated in EF)
            const float ez_ef = - sin_approx(courseOverGround) * rMat[0][0] - cos_approx(courseOverGround) * rMat[1][0];

            ex = rMat[2][0] * ez_ef;
            ey = rMat[2][1] * ez_ef;
            ez = rMat[2][2] * ez_ef;
        }
        else {
            ex = 0;
            ey = 0;
            ez = 0;
        }

        // Compute and apply integral feedback if enabled
        if (imuRuntimeConfig->dcm_ki_mag > 0.0f) {
            // Stop integrating if spinning beyond the certain limit
            if (spin_rate_sq < sq(DEGREES_TO_RADIANS(SPIN_RATE_LIMIT))) {
                integralMagX += imuRuntimeConfig->dcm_ki_mag * ex * dt;    // integral error scaled by Ki
                integralMagY += imuRuntimeConfig->dcm_ki_mag * ey * dt;
                integralMagZ += imuRuntimeConfig->dcm_ki_mag * ez * dt;

                gx += integralMagX;
                gy += integralMagY;
                gz += integralMagZ;
            }
        }

        // Calculate kP gain and apply proportional feedback
        gx += kpMag * ex;
        gy += kpMag * ey;
        gz += kpMag * ez;
    }


    /* Step 2: Roll and pitch correction -  use measured acceleration vector */
    if (useAcc) {
        float kpAcc = imuRuntimeConfig->dcm_kp_acc * imuGetPGainScaleFactor();
        const float accRecipNorm = invSqrt(ax * ax + ay * ay + az * az);

        // Just scale by 1G length - That's our vector adjustment. Rather than
        // using one-over-exact length (which needs a costly square root), we already
        // know the vector is enough "roughly unit length" and since it is only weighted
        // in by a certain amount anyway later, having that exact is meaningless. (c) MasterZap
        ax *= accRecipNorm;
        ay *= accRecipNorm;
        az *= accRecipNorm;

        // Error is sum of cross product between estimated direction and measured direction of gravity
        ex = (ay * rMat[2][2] - az * rMat[2][1]);
        ey = (az * rMat[2][0] - ax * rMat[2][2]);
        ez = (ax * rMat[2][1] - ay * rMat[2][0]);

        // Compute and apply integral feedback if enabled
        if (imuRuntimeConfig->dcm_ki_acc > 0.0f) {
            // Stop integrating if spinning beyond the certain limit
            if (spin_rate_sq < sq(DEGREES_TO_RADIANS(SPIN_RATE_LIMIT))) {
                integralAccX += imuRuntimeConfig->dcm_ki_acc * ex * dt;    // integral error scaled by Ki
                integralAccY += imuRuntimeConfig->dcm_ki_acc * ey * dt;
                integralAccZ += imuRuntimeConfig->dcm_ki_acc * ez * dt;

                gx += integralAccX;
                gy += integralAccY;
                gz += integralAccZ;
            }
        }

        // Calculate kP gain and apply proportional feedback
        gx += kpAcc * ex;
        gy += kpAcc * ey;
        gz += kpAcc * ez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    const float qa = q0;
    const float qb = q1;
    const float qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    const float quatRecipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= quatRecipNorm;
    q1 *= quatRecipNorm;
    q2 *= quatRecipNorm;
    q3 *= quatRecipNorm;

    // Check for invalid quaternion
    imuCheckAndResetOrientationQuaternion(ax, ay, az);

    // Pre-compute rotation matrix from quaternion
    imuComputeRotationMatrix();
}

//static void imuMahonyAHRSupdate(float dt, float gx, float gy, float gz,
//                                bool useAcc, float ax, float ay, float az,
//                                bool useMag, float mx, float my, float mz,
//                                bool useYaw, float yawError)
//{
//    static float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;    // integral error terms scaled by Ki
//    float recipNorm;
//    float hx, hy, bx;
//    float ex = 0, ey = 0, ez = 0;
//    float qa, qb, qc;
//
//
//    // Calculate general spin rate (rad/s)
//    float spin_rate = sqrtf(sq(gx) + sq(gy) + sq(gz));
//
//    // Use raw heading error (from GPS or whatever else)
//    if (useYaw) {
//        while (yawError >  M_PIf) yawError -= (2.0f * M_PIf);
//        while (yawError < -M_PIf) yawError += (2.0f * M_PIf);
//
//        ez += sin_approx(yawError / 2.0f);
//    }
//
//    // Use measured magnetic field vector
//    recipNorm = sq(mx) + sq(my) + sq(mz);
//    if (useMag && recipNorm > 0.01f) {
//        // Normalise magnetometer measurement
//        recipNorm = invSqrt(recipNorm);
//        mx *= recipNorm;
//        my *= recipNorm;
//        mz *= recipNorm;
//
//        // For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
//        // This way magnetic field will only affect heading and wont mess roll/pitch angles
//
//        // (hx; hy; 0) - measured mag field vector in EF (assuming Z-component is zero)
//        // (bx; 0; 0) - reference mag field vector heading due North in EF (assuming Z-component is zero)
//        hx = rMat[0][0] * mx + rMat[0][1] * my + rMat[0][2] * mz;
//        hy = rMat[1][0] * mx + rMat[1][1] * my + rMat[1][2] * mz;
//        bx = sqrtf(hx * hx + hy * hy);
//
//        // magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
//        float ez_ef = -(hy * bx);
//
//        // Rotate mag error vector back to BF and accumulate
//        ex += rMat[2][0] * ez_ef;
//        ey += rMat[2][1] * ez_ef;
//        ez += rMat[2][2] * ez_ef;
//    }
//
//    // Use measured acceleration vector
//    recipNorm = sq(ax) + sq(ay) + sq(az);
//    if (useAcc && recipNorm > 0.01f) {
//        // Normalise accelerometer measurement
//        recipNorm = invSqrt(recipNorm);
//        ax *= recipNorm;
//        ay *= recipNorm;
//        az *= recipNorm;
//
//        // Error is sum of cross product between estimated direction and measured direction of gravity
//        ex += (ay * rMat[2][2] - az * rMat[2][1]);
//        ey += (az * rMat[2][0] - ax * rMat[2][2]);
//        ez += (ax * rMat[2][1] - ay * rMat[2][0]);
//    }
//
//    // Compute and apply integral feedback if enabled
//    if(imuRuntimeConfig->dcm_ki > 0.0f) {
//        // Stop integrating if spinning beyond the certain limit
//        if (spin_rate < DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)) {
//            float dcmKiGain = imuRuntimeConfig->dcm_ki;
//            integralFBx += dcmKiGain * ex * dt;    // integral error scaled by Ki
//            integralFBy += dcmKiGain * ey * dt;
//            integralFBz += dcmKiGain * ez * dt;
//        }
//    }
//    else {
//        integralFBx = 0.0f;    // prevent integral windup
//        integralFBy = 0.0f;
//        integralFBz = 0.0f;
//    }
//
//    // Calculate kP gain. If we are acquiring initial attitude (not armed and within 20 sec from powerup) scale the kP to converge faster
//    float dcmKpGain = imuRuntimeConfig->dcm_kp * imuGetPGainScaleFactor();
//
//    // Apply proportional and integral feedback
//    gx += dcmKpGain * ex + integralFBx;
//    gy += dcmKpGain * ey + integralFBy;
//    gz += dcmKpGain * ez + integralFBz;
//
//    // Integrate rate of change of quaternion
//    gx *= (0.5f * dt);
//    gy *= (0.5f * dt);
//    gz *= (0.5f * dt);
//
//    qa = q0;
//    qb = q1;
//    qc = q2;
//    q0 += (-qb * gx - qc * gy - q3 * gz);
//    q1 += (qa * gx + qc * gz - q3 * gy);
//    q2 += (qa * gy - qb * gz + q3 * gx);
//    q3 += (qa * gz + qb * gy - qc * gx);
//
//    // Normalise quaternion
//    recipNorm = invSqrt(sq(q0) + sq(q1) + sq(q2) + sq(q3));
//    q0 *= recipNorm;
//    q1 *= recipNorm;
//    q2 *= recipNorm;
//    q3 *= recipNorm;
//
//    // Pre-compute rotation matrix from quaternion
//    imuComputeRotationMatrix();
//}

STATIC_UNIT_TESTED void imuUpdateEulerAngles(void)
{
#ifndef GPS
    // this local variable should be optimized out when GPS is not used.
    float magneticDeclination = 0.0f;
#endif
    /* Compute pitch/roll angles */
    attitude.values.roll = lrintf(atan2_approx(rMat[2][1], rMat[2][2]) * (1800.0f / M_PIf));
    attitude.values.pitch = lrintf(((0.5f * M_PIf) - acos_approx(-rMat[2][0])) * (1800.0f / M_PIf));
    attitude.values.yaw = lrintf((-atan2_approx(rMat[1][0], rMat[0][0]) * (1800.0f / M_PIf) + magneticDeclination));

    if (attitude.values.yaw < 0)
        attitude.values.yaw += 3600;

    /* Update small angle state */
    if (rMat[2][2] > smallAngleCosZ) {
        ENABLE_STATE(SMALL_ANGLE);
    } else {
        DISABLE_STATE(SMALL_ANGLE);
    }


}


float gyrotst[3]={0};

bool imuIsAircraftArmable(uint8_t arming_angle)
{
    /* Update small angle state */
    
    float armingAngleCosZ = cos_approx(degreesToRadians(arming_angle));
    
    return (rMat[2][2] > armingAngleCosZ);
}

static bool imuIsAccelerometerHealthy(void)
{
    int32_t axis;
    int32_t accMagnitude = 0;

    for (axis = 0; axis < 3; axis++) {
        accMagnitude += (int32_t)accSmooth[axis] * accSmooth[axis];
    }

    accMagnitude = accMagnitude * 100 / (sq((int32_t)acc.acc_1G));

    // Accept accel readings only in range 0.90g - 1.10g
    return (81 < accMagnitude) && (accMagnitude < 121);
}

#ifdef MAG
static bool isMagnetometerHealthy(void)
{
    return (magADC[X] != 0) && (magADC[Y] != 0) && (magADC[Z] != 0);
}
#endif

static void imuCalculateEstimatedAttitude(void)
{
    static uint32_t previousIMUUpdateTime;
    float rawYawError = 0;
    bool useAcc = false;
    bool useMag = false;
    bool useYaw = false;

    uint32_t currentTime = micros();
    uint32_t deltaT = currentTime - previousIMUUpdateTime;
    previousIMUUpdateTime = currentTime;

    if (imuIsAccelerometerHealthy()) {
        useAcc = true;
    }

#ifdef MAG
    if (sensors(SENSOR_MAG) && isMagnetometerHealthy()) {
        useMag = true;
    }
#endif
#if defined(GPS)
    else if (STATE(FIXED_WING) && sensors(SENSOR_GPS) && STATE(GPS_FIX) && GPS_numSat >= 5 && GPS_speed >= 300) {
        // In case of a fixed-wing aircraft we can use GPS course over ground to correct heading
        rawYawError = DECIDEGREES_TO_RADIANS(attitude.values.yaw - GPS_ground_course);
        useYaw = true;
    }
#endif

    imuMahonyAHRSupdate(deltaT * 1e-6f,
    					imuMeasuredRotationBF.A[X], imuMeasuredRotationBF.A[Y], imuMeasuredRotationBF.A[Z],
//    					gyroADC[0] * gyroScale_Adc2Rad, gyroADC[1] * gyroScale_Adc2Rad, gyroADC[2] * gyroScale_Adc2Rad,
//                        useAcc, imuMeasuredAccelBF.A[X], imuMeasuredAccelBF.A[Y], imuMeasuredAccelBF.A[Z],		//对于姿态补偿，ACC做低频量，不必追求短时的精度
						useAcc, accSmooth[X], accSmooth[Y], accSmooth[Z],
                        useMag, magADC[X], magADC[Y], magADC[Z],
                        useYaw, rawYawError);

    imuUpdateEulerAngles();

//    imuCalculateAcceleration(deltaT); // rotate acc vector into earth frame
}

//----iNAV----

#ifdef ASYNC_GYRO_PROCESSING

void imuUpdateGyroscope(timeUs_t gyroUpdateDeltaUs)
{
    float gyroUpdateDelta = gyroUpdateDeltaUs * 1e-6f;

    for (int axis = 0; axis < 3; axis++) {
        imuAccumulatedRate[axis] += gyroADC[axis] * gyroScale_Adc2Rad * gyroUpdateDelta;
    }

    imuAccumulatedRateTimeUs += gyroUpdateDeltaUs;
}
#endif

//float debuggyro[3];
/* Calculate rotation rate in rad/s in body frame */
static void imuUpdateMeasuredRotationRate(void)
{
    int axis;

#ifdef ASYNC_GYRO_PROCESSING
    float imuAccumulatedRateTime = imuAccumulatedRateTimeUs * 1e-6f;
    imuAccumulatedRateTimeUs = 0;

    for (axis = 0; axis < 3; axis++) {
        imuMeasuredRotationBF.A[axis] = imuAccumulatedRate[axis] / imuAccumulatedRateTime;
        imuAccumulatedRate[axis] = 0.0f;
    }
//    if (debugMode == DEBUG_PIDLOOP) {debug[0] = RADIANS_TO_DEGREES(imuMeasuredRotationBF.A[0]);
//									 debug[1] = RADIANS_TO_DEGREES(gyroADC[0] * gyroScale_Adc2Rad);
//									 debug[2] = RADIANS_TO_DEGREES(imuMeasuredRotationBF.A[1]);
//									 debug[3] = RADIANS_TO_DEGREES(gyroADC[1] * gyroScale_Adc2Rad);
//    }
//
//    debuggyro[0] = gyroADC[0] * gyroScale_Adc2Rad;
//    debuggyro[1] = gyroADC[1] * gyroScale_Adc2Rad;
//    debuggyro[2] = gyroADC[2] * gyroScale_Adc2Rad;
#else
    for (axis = 0; axis < 3; axis++) {
        imuMeasuredRotationBF.A[axis] = gyroADCf[axis] * gyroScale_Adc2Rad;
    }
#endif

}


/* Calculate measured acceleration in body frame cm/s/s */
static void imuUpdateMeasuredAcceleration(void)
{
    int axis;

#ifdef ASYNC_GYRO_PROCESSING
    for (axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        imuMeasuredAccelBF.A[axis] = imuAccumulatedAcc[axis] / imuAccumulatedAccCount;
        imuAccumulatedAcc[axis] = 0;
    }
    imuAccumulatedAccCount = 0;;
#else
    /* Convert acceleration to cm/s/s */
    for (axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        imuMeasuredAccelBF.A[axis] = accSmooth[axis] * (GRAVITY_CMS2 / acc.acc_1G);
    }
#endif

//    if (debugMode == DEBUG_PIDLOOP) {debug[3] = imuMeasuredAccelBF.A[2];}

}


void imuUpdateAccelerometer(rollAndPitchTrims_t *accelerometerTrims)			//1khz
{
    if (sensors(SENSOR_ACC)) {
        updateAccelerationReadings(accelerometerTrims);
        isAccelUpdatedAtLeastOnce = true;
    }

#ifdef ASYNC_GYRO_PROCESSING
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        imuAccumulatedAcc[axis] += accSmooth[axis] * (GRAVITY_CMS2 / acc.acc_1G);
    }
    imuAccumulatedAccCount++;
#endif
}

void imuUpdateAttitude(void)
{
    if (sensors(SENSOR_ACC) && isAccelUpdatedAtLeastOnce) {
    	imuUpdateMeasuredRotationRate();		//陀螺仪测量更新
    	imuUpdateMeasuredAcceleration();		//加速度计测量更新
        imuCalculateEstimatedAttitude();		//
    } else {
        accSmooth[X] = 0;
        accSmooth[Y] = 0;
        accSmooth[Z] = 0;
    }
}

float getCosTiltAngle(void)
{
    return rMat[2][2];
}

int16_t calculateThrottleAngleCorrection(uint8_t throttle_correction_value)
{
    /*
    * Use 0 as the throttle angle correction if we are inverted, vertical or with a
    * small angle < 0.86 deg
    * TODO: Define this small angle in config.
    */
    if (rMat[2][2] <= 0.015f) {
        return 0;
    }
    int angle = lrintf(acos_approx(rMat[2][2]) * throttleAngleScale);
    if (angle > 900)
        angle = 900;
    return lrintf(throttle_correction_value * sin_approx(angle / (900.0f * M_PIf / 2.0f)));
}

bool isImuReady(void)
{
	return(isAccelerationCalibrationComplete() && sensors(SENSOR_ACC) && isGyroCalibrationComplete());
}

bool isImuHeadingValid(void)
{
    return (sensors(SENSOR_MAG) && isMagnetometerHealthy()) || (STATE(FIXED_WING) && sensors(SENSOR_GPS) && STATE(GPS_FIX));
}


//
//int16_t debugGetGyroX100()
//{
//	return debuggyro[0]*100;
//}
//int16_t debugGetGyroY100()
//{
//	return debuggyro[1]*100;
//}
//int16_t debugGetGyroZ100()
//{
//	return debuggyro[2]*100;
//}
//
//int16_t debugGetGyroAcclX100()
//{
//	return imuMeasuredRotationBF.A[0]*100;
//}
//int16_t debugGetGyroAcclY100()
//{
//	return imuMeasuredRotationBF.A[1]*100;
//}
//int16_t debugGetGyroAcclZ100()
//{
//	return imuMeasuredRotationBF.A[2]*100;
//}

