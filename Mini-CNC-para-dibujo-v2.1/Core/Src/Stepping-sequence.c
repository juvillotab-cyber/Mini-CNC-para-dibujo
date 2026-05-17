#include "Stepping-sequence.h"

const uint8_t STEPPER_SEQ[STEPPER_SEQ_SIZE][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1},
};

static void stepper_apply(StepperMotor_t *m)
{
    for (int i = 0; i < 4; i++)
    {
        HAL_GPIO_WritePin(m->port[i], m->pin[i],
            STEPPER_SEQ[m->seq_idx][i] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

void StepperMotor_Init(StepperMotor_t *m,
    GPIO_TypeDef *p0, uint16_t pin0,
    GPIO_TypeDef *p1, uint16_t pin1,
    GPIO_TypeDef *p2, uint16_t pin2,
    GPIO_TypeDef *p3, uint16_t pin3)
{
    m->port[0] = p0; m->pin[0] = pin0;
    m->port[1] = p1; m->pin[1] = pin1;
    m->port[2] = p2; m->pin[2] = pin2;
    m->port[3] = p3; m->pin[3] = pin3;
    m->direction = STEPPER_DIR_FORWARD;
    m->position  = 0;
    m->last_dir  = 0;
    m->seq_idx   = 0;
}

void StepperMotor_SetDirection(StepperMotor_t *m, int8_t dir)
{
    m->last_dir  = m->direction;
    m->direction = (dir >= 0) ? STEPPER_DIR_FORWARD : STEPPER_DIR_BACKWARD;
}

void StepperMotor_Step(StepperMotor_t *m)
{
    m->seq_idx = (uint8_t)((m->seq_idx + STEPPER_SEQ_SIZE + m->direction) % STEPPER_SEQ_SIZE);
    stepper_apply(m);
    m->position += m->direction;
}

void StepperMotor_Release(StepperMotor_t *m)
{
    for (int i = 0; i < 4; i++)
        HAL_GPIO_WritePin(m->port[i], m->pin[i], GPIO_PIN_RESET);
}
