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

extern int32_t AltHold;
extern int32_t vario;

typedef struct airplaneConfig_s {
    int8_t fixedwing_althold_dir;           // +1 or -1 for pitch/althold gain. later check if need more than just sign
} airplaneConfig_t;

PG_DECLARE(airplaneConfig_t, airplaneConfig);

void calculateEstimatedAltitude(uint32_t currentTime);

void applyAltHold(void);
void updateAltHoldState(void);
void updateSonarAltHoldState(void);
void updateIRrangfdAltHoldState(void);

int32_t altitudeHoldGetEstimatedAltitude(void);
int32_t altitudeGetCfVel(void);
int32_t altitudeGetImuBasedVel(void);
int32_t altitudeGetImuBasedAlt(void);
int32_t altitudeGetBaroAlt(void);
int32_t altitudeGetNoneImuVel(void);
int32_t altitudeGetAltHold(void);
int32_t altitudeGetsetVel(void);
int16_t altitudeGetMwradarAlt(void);
int32_t altitudeGetBaroRelaAlt(void);
