#include "Backlash.h"

void Backlash_Init(Backlash_t *bl)
{
    bl->last_dir_x = 0;
    bl->last_dir_y = 0;
    bl->pending_x  = 0;
    bl->pending_y  = 0;
}

void Backlash_Update(Backlash_t *bl, int8_t new_dir_x, int8_t new_dir_y)
{
    if (bl->last_dir_x != 0 && bl->last_dir_x != new_dir_x)
        bl->pending_x += (new_dir_x > 0) ? BACKLASH_X_STEPS : -BACKLASH_X_STEPS;

    if (bl->last_dir_y != 0 && bl->last_dir_y != new_dir_y)
        bl->pending_y += (new_dir_y > 0) ? BACKLASH_Y_STEPS : -BACKLASH_Y_STEPS;

    bl->last_dir_x = new_dir_x;
    bl->last_dir_y = new_dir_y;
}

bool Backlash_HasPending(Backlash_t *bl)
{
    return (bl->pending_x != 0 || bl->pending_y != 0);
}

int8_t Backlash_ConsumeX(Backlash_t *bl)
{
    if (bl->pending_x > 0) { bl->pending_x--; return  1; }
    if (bl->pending_x < 0) { bl->pending_x++; return -1; }
    return 0;
}

int8_t Backlash_ConsumeY(Backlash_t *bl)
{
    if (bl->pending_y > 0) { bl->pending_y--; return  1; }
    if (bl->pending_y < 0) { bl->pending_y++; return -1; }
    return 0;
}
