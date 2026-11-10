#pragma once
#include "rb_choreo.h"

/* UI/application boundary. Only an explicit Play submits a motor request. */
typedef struct {
    const char *state, *error;
    float elapsed_s, phase;
    unsigned number;
    bool bridge, active, finished;
} rb_choreo_motion_status_t;
typedef struct {
    const char *(*play)(const rb_choreo_work_t *work, float *duration_s);
    bool (*status)(rb_choreo_motion_status_t *status);
    void (*stop)(void);
    void (*home)(void);
} rb_choreo_motion_backend_t;
void rb_choreo_ui_set_motion(const rb_choreo_motion_backend_t *backend);
