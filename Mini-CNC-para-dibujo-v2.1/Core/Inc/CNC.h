#pragma once

#include "Stepping-sequence.h"
#include "Gcode-parser.h"
#include "Backlash.h"
#include <stdint.h>
#include <stdbool.h>

#define CNC_RX_BUF_SIZE    64
#define CNC_STEP_DELAY_MS  4
#define CNC_Z_STEPS        100

typedef enum {
    CNC_STATE_IDLE = 0,
    CNC_STATE_MOVING_Z,
    CNC_STATE_MOVING_XY,
    CNC_STATE_CALIBRATING,
} CNC_State_t;

typedef struct {
    float dx;
    float dy;
    float sx;
    float sy;
    float err;
    float steps_left;
} CNC_Bresenham_t;

typedef struct {
    StepperMotor_t  motor_x;
    StepperMotor_t  motor_y;
    StepperMotor_t  motor_z;

    CNC_State_t     state;
    CNC_Bresenham_t bres;

    float           cur_x_steps;
    float           cur_y_steps;
    float           target_x_steps;
    float           target_y_steps;

    Z_State_t       current_z_state;
    Z_State_t       target_z_state;
    int32_t         z_steps_left;

    Backlash_t      backlash;

    uint32_t        last_tick;

    char            rx_buf[CNC_RX_BUF_SIZE];
    uint8_t         rx_byte;
    uint8_t         rx_idx;
    bool            rx_ready;
    bool            rx_terminate;
} CNC_t;

void CNC_Init(CNC_t *cnc);
void CNC_Run(CNC_t *cnc);
void CNC_UART_ISR(CNC_t *cnc);
