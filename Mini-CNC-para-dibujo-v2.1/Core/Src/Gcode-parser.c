#include "Gcode-parser.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

static char *find_param(char *line, char param)
{
    char *p = strchr(line, param);
    if (!p) return NULL;
    return p + 1;
}

void GCode_Parse(const char *line, GCodeCommand_t *cmd)
{
    char work[64];
    strncpy(work, line, sizeof(work) - 1);
    work[sizeof(work) - 1] = '\0';

    cmd->target_x_mm     = 0.0f;
    cmd->target_y_mm     = 0.0f;
    cmd->target_z_state  = Z_STATE_IDLE;
    cmd->has_xy          = false;
    cmd->has_z           = false;

    char *px = find_param(work, 'X');
    char *py = find_param(work, 'Y');
    char *pz = find_param(work, 'Z');

    if (px || py) cmd->has_xy = true;

    if (px) cmd->target_x_mm = (float)atof(px);
    if (py) cmd->target_y_mm = (float)atof(py);

    if (pz)
    {
        cmd->has_z = true;
        float z_val = (float)atof(pz);

        if (z_val > GCODE_Z_UP_THRESHOLD)
            cmd->target_z_state = Z_STATE_UP;
        else if (z_val < GCODE_Z_DOWN_THRESHOLD)
            cmd->target_z_state = Z_STATE_DOWN;
        else
            cmd->has_z = false;
    }
}

float GCode_MmToSteps(float mm)
{
    return mm * GCODE_STEPS_PER_MM;
}
