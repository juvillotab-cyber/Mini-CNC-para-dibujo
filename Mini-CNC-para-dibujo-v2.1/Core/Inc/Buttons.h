#pragma once

#include "main.h"
#include <stdbool.h>

#define BUTTON_CAL_STEP_DELAY_MS  4

typedef enum {
    BTN_NONE = 0,
    BTN_XP,
    BTN_XN,
    BTN_YP,
    BTN_YN,
    BTN_Z1,
    BTN_Z2,
} Button_t;

Button_t Buttons_GetActive(void);
bool     Buttons_AnyActive(void);
