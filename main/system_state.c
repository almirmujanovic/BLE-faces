#include "include/system_state.h"

static volatile bool s_idle = false;
static volatile bool s_pause_for_wifi = false;
static volatile uint32_t s_paused_mask = 0;

void system_set_idle(bool idle)
{
    s_idle = idle;
}

bool system_is_idle(void)
{
    return s_idle;
}

void system_set_pause_for_wifi(bool pause)
{
    s_pause_for_wifi = pause;
}

bool system_is_pause_for_wifi(void)
{
    return s_pause_for_wifi;
}

void system_mark_task_paused(system_task_id_t task, bool paused)
{
    if (paused) {
        s_paused_mask |= (uint32_t)task;
    } else {
        s_paused_mask &= ~(uint32_t)task;
    }
}

bool system_are_tasks_paused(uint32_t task_mask)
{
    return (s_paused_mask & task_mask) == task_mask;
}
