#pragma once

#include "main.h"
#include <stdint.h>

#define STEPPER_SEQ_SIZE  8
#define STEPPER_DELAY_MS  4

#define STEPPER_DIR_FORWARD    1
#define STEPPER_DIR_BACKWARD  -1

extern const uint8_t STEPPER_SEQ[STEPPER_SEQ_SIZE][4];

typedef struct {
    GPIO_TypeDef *port[4];
    uint16_t      pin[4];
    int8_t        direction;
    int32_t       position;
    int8_t        last_dir;
    uint8_t       seq_idx;
} StepperMotor_t;

void StepperMotor_Init(StepperMotor_t *m,
    GPIO_TypeDef *p0, uint16_t pin0,
    GPIO_TypeDef *p1, uint16_t pin1,
    GPIO_TypeDef *p2, uint16_t pin2,
    GPIO_TypeDef *p3, uint16_t pin3);

void StepperMotor_SetDirection(StepperMotor_t *m, int8_t dir);
void StepperMotor_Step(StepperMotor_t *m);
void StepperMotor_Release(StepperMotor_t *m);
