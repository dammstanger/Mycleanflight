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

#include <stdint.h>

#include "platform.h"

#define PERIOD_HZ(hz) (1000000 / (hz))
#define PERIOD_MS(ms) ((ms) * 1000)
#define PERIOD_US(us) (us)

#define HZ_FROM_PERIOD(period) (1000000 / (period))

#define US_FROM_HZ(hz) (1000000 / (hz))
#define US_FROM_MS(ms) ((ms) * 1000)
#define US_FROM_US(us) (us)

// time difference, 32 bits always sufficient
typedef int32_t timeDelta_t;
// millisecond time
typedef uint32_t timeMs_t ;
// microsecond time
#ifdef USE_64BIT_TIME
typedef uint64_t timeUs_t;
#define TIMEUS_MAX UINT64_MAX
#else
typedef uint32_t timeUs_t;
#define TIMEUS_MAX UINT32_MAX
#endif

static inline timeDelta_t cmpTimeUs(timeUs_t a, timeUs_t b) { return (timeDelta_t)(a - b); }
