#pragma once

#include <stdint.h>
#include <stdbool.h>

#define BACKLASH_X_STEPS  0
#define BACKLASH_Y_STEPS  0

typedef struct {
    int8_t  last_dir_x;
    int8_t  last_dir_y;
    int16_t pending_x;
    int16_t pending_y;
} Backlash_t;

void Backlash_Init(Backlash_t *bl);
void Backlash_Update(Backlash_t *bl, int8_t new_dir_x, int8_t new_dir_y);
bool Backlash_HasPending(Backlash_t *bl);

int8_t Backlash_ConsumeX(Backlash_t *bl);
int8_t Backlash_ConsumeY(Backlash_t *bl);
