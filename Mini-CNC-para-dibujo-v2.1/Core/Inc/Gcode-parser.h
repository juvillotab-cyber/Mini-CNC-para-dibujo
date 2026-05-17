#pragma once

#include <stdint.h>
#include <stdbool.h>

#define GCODE_STEPS_PER_REV      4096
#define GCODE_MM_PER_REV         20.0f
#define GCODE_STEPS_PER_MM       (GCODE_STEPS_PER_REV / GCODE_MM_PER_REV)
#define GCODE_Z_UP_THRESHOLD      1.0f
#define GCODE_Z_DOWN_THRESHOLD    1.0f

typedef enum {
    Z_STATE_IDLE = 0,
    Z_STATE_UP,
    Z_STATE_DOWN
} Z_State_t;

typedef struct {
    float     target_x_mm;
    float     target_y_mm;
    Z_State_t target_z_state;
    bool      has_xy;
    bool      has_z;
} GCodeCommand_t;

void  GCode_Parse(const char *line, GCodeCommand_t *cmd);
float GCode_MmToSteps(float mm);
