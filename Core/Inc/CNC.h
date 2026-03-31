#pragma once

#include "stm32f0xx_hal.h"
#include "usart.h"
#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

/* ─────────────────────────────────────────
 *  CONFIGURACIÓN
 * ───────────────────────────────────────── */
#define CNC_STEP_MODE_4     4
#define CNC_STEP_MODE_8     8
#define CNC_STEP_DELAY_MS   5u
#define CNC_RX_BUF_SIZE     32
#define CNC_STEPS_PER_REV   4096
#define CNC_MM_PER_REV      20
#define CNC_STEPS_PER_MM    (CNC_STEPS_PER_REV / CNC_MM_PER_REV)  // 204

/* ─────────────────────────────────────────
 *  SECUENCIAS 28BYJ-48  (IN1 IN2 IN3 IN4)
 * ───────────────────────────────────────── */
static const uint8_t SEQ_4[4][4] = {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1},
};

static const uint8_t SEQ_8[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1},
};

/* ─────────────────────────────────────────
 *  MOTOR
 * ───────────────────────────────────────── */
typedef struct {
    GPIO_TypeDef *port[4];
    uint16_t      pin[4];
    uint8_t       step_mode;
    uint8_t       seq_idx;
    int8_t        direction;
    int32_t       position;
} StepperMotor_t;

/* ─────────────────────────────────────────
 *  ESTADOS Y BRESENHAM
 * ───────────────────────────────────────── */
typedef enum { CNC_IDLE = 0, CNC_MOVING } CNC_State_t;

typedef struct {
    int32_t dx, dy, sx, sy, err, steps_left;
} CNC_Bresenham_t;

/* ─────────────────────────────────────────
 *  CONTEXTO GLOBAL
 * ───────────────────────────────────────── */
typedef struct {
    StepperMotor_t  motor_x;
    StepperMotor_t  motor_y;
    StepperMotor_t  motor_z;

    CNC_State_t     state;
    CNC_Bresenham_t bres;

    int32_t         cur_x, cur_y;
    uint32_t        last_tick;

    /* Buffer UART */
    char     rx_buf[CNC_RX_BUF_SIZE];   /* línea gcode completa   */
    uint8_t  rx_byte;                    /* byte actual ISR        */
    uint8_t  rx_idx;                     /* índice escritura       */
    bool     rx_ready;                   /* línea lista para usar  */
} CNC_t;

/* ─────────────────────────────────────────
 *  API
 * ───────────────────────────────────────── */
void CNC_Init    (CNC_t *cnc, uint8_t step_mode);
void CNC_Run     (CNC_t *cnc);   /* while(1)            */
void CNC_UART_ISR(CNC_t *cnc);   /* HAL_UART_RxCpltCallback */

