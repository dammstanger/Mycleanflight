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

#define SRC_MAIN_SCHEDULER_C_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "scheduler/scheduler.h"
#include "build/debug.h"
#include "build/build_config.h"

#include "common/maths.h"

#include "drivers/system.h"
#include "config/config_unittest.h"

static cfTask_t *currentTask = NULL;

//#define USE_REALTIME_GUARD_INTERVAL
#define REALTIME_GUARD_INTERVAL_MIN     10
#define REALTIME_GUARD_INTERVAL_MAX     300
#define REALTIME_GUARD_INTERVAL_MARGIN  25

static uint32_t totalWaitingTasks;
static uint32_t totalWaitingTasksSamples;
static uint32_t realtimeGuardInterval = REALTIME_GUARD_INTERVAL_MAX;

uint32_t currentTime = 0;
uint16_t averageSystemLoadPercent = 0;


static int taskQueuePos = 0;
static unsigned int taskQueueSize = 0;

STATIC_UNIT_TESTED void queueClear(void)
{
    memset(taskQueueArray, 0, taskQueueArraySize * sizeof(cfTask_t *));
    taskQueuePos = 0;
    taskQueueSize = 0;
}

#ifdef UNIT_TEST
STATIC_UNIT_TESTED int queueSize(void)
{
    return taskQueueSize;
}
#endif

//检查队列中是否包含该任务。
STATIC_UNIT_TESTED bool queueContains(cfTask_t *task)
{
    for (unsigned int ii = 0; ii < taskQueueSize; ++ii) {
        if (taskQueueArray[ii] == task) {
            return true;
        }
    }
    return false;
}

STATIC_UNIT_TESTED bool queueAdd(cfTask_t *task)
{
    if ((taskQueueSize >= taskCount) || queueContains(task)) {		//如果队列任务数已超过任务总数，或者队列中已经有该任务，
        return false;
    }
    for (unsigned int ii = 0; ii <= taskQueueSize; ++ii) {
        if (taskQueueArray[ii] == NULL || taskQueueArray[ii]->staticPriority < task->staticPriority) {
            memmove(&taskQueueArray[ii+1], &taskQueueArray[ii], sizeof(task) * (taskQueueSize - ii));
            taskQueueArray[ii] = task;
            ++taskQueueSize;
            return true;
        }
    }
    return false;
}

STATIC_UNIT_TESTED bool queueRemove(cfTask_t *task)
{
    for (unsigned int ii = 0; ii < taskQueueSize; ++ii) {
        if (taskQueueArray[ii] == task) {
            memmove(&taskQueueArray[ii], &taskQueueArray[ii+1], sizeof(task) * (taskQueueSize - ii));
            --taskQueueSize;
            return true;
        }
    }
    return false;
}

/*
 * Returns first item queue or NULL if queue empty
 */
STATIC_INLINE_UNIT_TESTED cfTask_t *queueFirst(void)
{
    taskQueuePos = 0;
    return taskQueueArray[0]; // guaranteed to be NULL if queue is empty
}

/*
 * Returns next item in queue or NULL if at end of queue
 */
STATIC_INLINE_UNIT_TESTED cfTask_t *queueNext(void)
{
    return taskQueueArray[++taskQueuePos]; // guaranteed to be NULL at end of queue
}

//系统任务
void taskSystem(void)
{
    // Calculate system load
    if (totalWaitingTasksSamples > 0) {
        averageSystemLoadPercent = 100 * totalWaitingTasks / totalWaitingTasksSamples;
        totalWaitingTasksSamples = 0;
        totalWaitingTasks = 0;
    }

    realtimeGuardInterval = 0;

#ifdef USE_REALTIME_GUARD_INTERVAL
    // Calculate guard interval
    uint32_t maxNonRealtimeTaskTime = 0;
    for (const cfTask_t *task = queueFirst(); task != NULL; task = queueNext()) {
        if (task->staticPriority != TASK_PRIORITY_REALTIME) {
            maxNonRealtimeTaskTime = MAX(maxNonRealtimeTaskTime, task->averageExecutionTime);
        }
    }

    realtimeGuardInterval = constrain(maxNonRealtimeTaskTime, REALTIME_GUARD_INTERVAL_MIN, REALTIME_GUARD_INTERVAL_MAX) + REALTIME_GUARD_INTERVAL_MARGIN;
#if defined SCHEDULER_DEBUG
    debug[2] = realtimeGuardInterval;
#endif
#endif
}

#ifndef SKIP_TASK_STATISTICS
void getTaskInfo(const int taskId, cfTaskInfo_t * taskInfo)
{
    taskInfo->taskName = cfTasks[taskId].taskName;
    taskInfo->isEnabled = queueContains(&cfTasks[taskId]);		//如果由此任务，说明任务已使能
    taskInfo->desiredPeriod = cfTasks[taskId].desiredPeriod;
    taskInfo->staticPriority = cfTasks[taskId].staticPriority;
    taskInfo->maxExecutionTime = cfTasks[taskId].maxExecutionTime;
    taskInfo->totalExecutionTime = cfTasks[taskId].totalExecutionTime;
    taskInfo->averageExecutionTime = cfTasks[taskId].averageExecutionTime;
    taskInfo->latestDeltaTime = cfTasks[taskId].taskLatestDeltaTime;
}
#endif

void rescheduleTask(const int taskId, uint32_t newPeriodMicros)
{
    if (taskId == TASK_SELF || taskId < (int)taskCount) {
        cfTask_t *task = taskId == TASK_SELF ? currentTask : &cfTasks[taskId];
        task->desiredPeriod = MAX(100, newPeriodMicros);  // Limit delay to 100us (10 kHz) to prevent scheduler clogging 任务的周期限制在大于等于100us即10KHz上,
    }
}

void setTaskEnabled(const int taskId, bool enabled)
{
    if (taskId == TASK_SELF || taskId < (int)taskCount) {
        cfTask_t *task = taskId == TASK_SELF ? currentTask : &cfTasks[taskId];
        if (enabled && task->taskFunc) {
            queueAdd(task);
        } else {
            queueRemove(task);
        }
    }
}

uint32_t getTaskDeltaTime(const int taskId)
{
    if (taskId == TASK_SELF || taskId < (int)taskCount) {
        cfTask_t *task = taskId == TASK_SELF ? currentTask : &cfTasks[taskId];
        return task->taskLatestDeltaTime;
    } else {
        return 0;
    }
}

void schedulerInit(void)
{
    queueClear();
}

void scheduler(void)
{
    // Cache currentTime
    currentTime = micros();

    // Check for realtime tasks
    uint32_t timeToNextRealtimeTask = UINT32_MAX;
    for (const cfTask_t *task = queueFirst(); task != NULL && task->staticPriority >= TASK_PRIORITY_REALTIME; task = queueNext()) {
        const uint32_t nextExecuteAt = task->lastExecutedAt + task->desiredPeriod;
        if ((int32_t)(currentTime - nextExecuteAt) >= 0) {
            timeToNextRealtimeTask = 0;
        } else {
            const uint32_t newTimeInterval = nextExecuteAt - currentTime;
            timeToNextRealtimeTask = MIN(timeToNextRealtimeTask, newTimeInterval);
        }
    }
    //
    const bool outsideRealtimeGuardInterval = (timeToNextRealtimeTask > realtimeGuardInterval);

    // The task to be invoked
    cfTask_t *selectedTask = NULL;
    uint16_t selectedTaskDynamicPriority = 0;

    // Update task dynamic priorities
    uint16_t waitingTasks = 0;
    for (cfTask_t *task = queueFirst(); task != NULL; task = queueNext()) {
        // Task has checkFunc - event driven	事件驱动的任务
        if (task->checkFunc != NULL) {
            // Increase priority for event driven tasks
            if (task->dynamicPriority > 0) {
                task->taskAgeCycles = 1 + ((currentTime - task->lastSignaledAt) / task->desiredPeriod);
                task->dynamicPriority = 1 + task->staticPriority * task->taskAgeCycles;
                waitingTasks++;
            } else if (task->checkFunc(currentTime - task->lastExecutedAt)) {	//上次运行后首次被轮询，问任务自己是否需要运行
                task->lastSignaledAt = currentTime;
                task->taskAgeCycles = 1;
                task->dynamicPriority = 1 + task->staticPriority;
                waitingTasks++;
            } else {
                task->taskAgeCycles = 0;
            }
        } else {
            // Task is time-driven, dynamicPriority is last execution age (measured in desiredPeriods)
            // Task age is calculated from last execution
            task->taskAgeCycles = ((currentTime - task->lastExecutedAt) / task->desiredPeriod);
            if (task->taskAgeCycles > 0) {
                task->dynamicPriority = 1 + task->staticPriority * task->taskAgeCycles;
                waitingTasks++;
            }
        }
        //循环结束便有最需要的任务进入调度器
        if (task->dynamicPriority > selectedTaskDynamicPriority) {
            const bool taskCanBeChosenForScheduling =
                (outsideRealtimeGuardInterval) ||
                (task->taskAgeCycles > 1) ||			//距上次执行已有2个以上期望周期的时间了
                (task->staticPriority == TASK_PRIORITY_REALTIME);
            if (taskCanBeChosenForScheduling) {
                selectedTaskDynamicPriority = task->dynamicPriority;
                selectedTask = task;
            }
        }
    }

    totalWaitingTasksSamples++;
    totalWaitingTasks += waitingTasks;

    currentTask = selectedTask;

    if (selectedTask != NULL) {
        // Found a task that should be run
        selectedTask->taskLatestDeltaTime = currentTime - selectedTask->lastExecutedAt;
        selectedTask->lastExecutedAt = currentTime;
        selectedTask->dynamicPriority = 0;

        // Execute task
        const uint32_t currentTimeBeforeTaskCall = micros();
        selectedTask->taskFunc();
        const uint32_t taskExecutionTime = micros() - currentTimeBeforeTaskCall;

        selectedTask->averageExecutionTime = ((uint32_t)selectedTask->averageExecutionTime * 31 + taskExecutionTime) / 32;
#ifndef SKIP_TASK_STATISTICS
        selectedTask->totalExecutionTime += taskExecutionTime;   // time consumed by scheduler + task
        selectedTask->maxExecutionTime = MAX(selectedTask->maxExecutionTime, taskExecutionTime);
#endif
#if defined SCHEDULER_DEBUG
        debug[3] = (micros() - currentTime) - taskExecutionTime;
    } else {
        debug[3] = (micros() - currentTime);
#endif
    }
    GET_SCHEDULER_LOCALS();
}
