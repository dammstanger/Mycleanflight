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

#pragma once

#define PID_MAX_I 256
#define PID_MAX_D 512
#define PID_MAX_TOTAL_PID 1000

#define GYRO_I_MAX 256                      // Gyro I limiter
#define YAW_P_LIMIT_MIN 100                 // Maximum value for yaw P limiter
#define YAW_P_LIMIT_MAX 500                 // Maximum value for yaw P limiter

//typedef enum {
//    PIDROLL,
//    PIDPITCH,
//    PIDYAW,
//    PIDALT,
//    PIDPOS,
//    PIDPOSR,
//    PIDNAVR,
//    PIDLEVEL,
//    PIDMAG,
//    PIDVEL,
//    PID_ITEM_COUNT
//} pidIndex_e;

typedef enum {
    PID_CONTROLLER_MW23 = 0,
    PID_CONTROLLER_MWREWRITE,
    PID_CONTROLLER_LUX_FLOAT,
    PID_COUNT
} pidControllerType_e;

typedef enum {
    PID_DELTA_FROM_MEASUREMENT = 0,
    PID_DELTA_FROM_ERROR
} pidDeltaMethod_e;

typedef enum {
    HORIZON_TILT_MODE_SAFE = 0,
    HORIZON_TILT_MODE_EXPERT
} horizonTiltMode_e;

enum {
    HEADING_HOLD_DISABLED = 0,
    HEADING_HOLD_UPDATE_HEADING,
    HEADING_HOLD_ENABLED
};

typedef enum {
    /* PID              MC      FW  */
    PID_ROLL,       //   +       +
    PID_PITCH,      //   +       +
    PID_YAW,        //   +       +
    PID_POS_Z,      //   +       +
    PID_POS_XY,     //   +       +
    PID_VEL_XY,     //   +       n/a
    PID_SURFACE,    //   n/a     n/a
    PID_LEVEL,      //   +       +
    PID_HEADING,    //   +       +
    PID_VEL_Z,      //   +       n/a
    PID_ITEM_COUNT
} pidIndex_e;

typedef struct pid8_s {
    uint8_t P;
    uint8_t I;
    uint8_t D;
} pid8_t;

typedef struct pidBank_s {
    pid8_t  pid[PID_ITEM_COUNT];
} pidBank_t;

typedef struct pidProfile_s {
//    uint8_t  P8[PID_ITEM_COUNT];
//    uint8_t  I8[PID_ITEM_COUNT];
//    uint8_t  D8[PID_ITEM_COUNT];
    pidBank_t bank_fw;
    pidBank_t bank_mc;

    uint8_t  pidController;
    uint16_t yaw_p_limit;                   // set P term limit (fixed value was 300)
    uint16_t dterm_lpf_hz;                  // Delta Filter in hz
    uint16_t yaw_lpf_hz;                    // additional yaw filter when yaw axis too noisy
    uint8_t  deltaMethod;

    uint8_t horizon_tilt_effect;            // inclination factor for Horizon mode
    uint8_t horizon_tilt_mode;              // SAFE or EXPERT

    uint8_t dterm_filter_type;              // Filter selection for dterm		CLI中使用dterm_lowpass_level项进行设置，NORMAL代表lpf滤波器，HIGH代表biquad(双二阶滤波器)
    uint16_t dterm_notch_hz;                // Biquad dterm notch hz
    uint16_t dterm_notch_cutoff;            // Biquad dterm notch low cutoff
} pidProfile_t;

PG_DECLARE_PROFILE(pidProfile_t, pidProfile);

struct controlRateConfig_s;
union rollAndPitchTrims_u;
struct rxConfig_s;
typedef void (*pidControllerFuncPtr)(const pidProfile_t *pidProfile, const struct controlRateConfig_s *controlRateConfig,
        uint16_t max_angle_inclination, const union rollAndPitchTrims_u *angleTrim, const struct rxConfig_s *rxConfig);            // pid controller function prototype

extern int16_t axisPID[FD_INDEX_COUNT];
extern int32_t axisPID_P[FD_INDEX_COUNT], axisPID_I[FD_INDEX_COUNT], axisPID_D[FD_INDEX_COUNT];
extern uint32_t targetPidLooptime;

float pidScaleITermToRcInput(int axis);
void pidFilterIsSetCheck(const pidProfile_t *pidProfile);

float getdT(void);
void pidInit(uint32_t pidLooptime);
void pidInitFilters(const pidProfile_t *pidProfile);
void pidSetController(pidControllerType_e type);
void pidSetTargetLooptime(uint32_t pidLooptime);
void pidResetITermAngle(void);
void pidResetITerm(void);

int calcHorizonLevelStrength(uint16_t rxConfigMidrc, int horizonTiltEffect,
        uint8_t horizonTiltMode, int horizonSensitivity);

int16_t pidAngleToRcCommand(float angleDeciDegrees, int16_t maxInclination);

void updateHeadingHoldTarget(int16_t heading);
void resetHeadingHoldTarget(int16_t heading);
int16_t getHeadingHoldTarget();

//debug

s16 debug_GetantiWindupScaler10(void);
s16 debug_GetITerm(void);
s16 debug_GetnewOutputLimited(void);
