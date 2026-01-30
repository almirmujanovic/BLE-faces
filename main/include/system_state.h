#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SYSTEM_TASK_FACE    = 1u << 0,
    SYSTEM_TASK_GESTURE = 1u << 1,
} system_task_id_t;

void system_set_idle(bool idle);
bool system_is_idle(void);
// Pause gesture processing before WiFi starts to avoid ML teardown races
void system_set_pause_for_wifi(bool pause);
bool system_is_pause_for_wifi(void);
void system_mark_task_paused(system_task_id_t task, bool paused);
bool system_are_tasks_paused(uint32_t task_mask);

#ifdef __cplusplus
}
#endif
