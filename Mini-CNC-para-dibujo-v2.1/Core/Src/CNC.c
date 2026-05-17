#include "CNC.h"
#include "usart.h"
#include "Buttons.h"
#include <string.h>
#include <math.h>

static void bres_load(CNC_t *cnc, float tx, float ty)
{
    CNC_Bresenham_t *b = &cnc->bres;

    int8_t old_dir_x = cnc->motor_x.direction;
    int8_t old_dir_y = cnc->motor_y.direction;

    float dx = tx - cnc->cur_x_steps;
    float dy = ty - cnc->cur_y_steps;

    int8_t new_dir_x = (dx >= 0.0f) ? 1 : -1;
    int8_t new_dir_y = (dy >= 0.0f) ? 1 : -1;

    if (old_dir_x != 0 && old_dir_x != new_dir_x)
    {
        tx += (float)new_dir_x * (float)BACKLASH_X_STEPS;
        dx = tx - cnc->cur_x_steps;
    }
    if (old_dir_y != 0 && old_dir_y != new_dir_y)
    {
        ty += (float)new_dir_y * (float)BACKLASH_Y_STEPS;
        dy = ty - cnc->cur_y_steps;
    }

    b->sx = (dx >= 0.0f) ? 1.0f : -1.0f;
    b->sy = (dy >= 0.0f) ? 1.0f : -1.0f;
    b->dx = fabsf(dx);
    b->dy = fabsf(dy);

    StepperMotor_SetDirection(&cnc->motor_x, (int8_t)b->sx);
    StepperMotor_SetDirection(&cnc->motor_y, (int8_t)b->sy);

    if (b->dx >= b->dy)
    {
        b->err        = 2.0f * b->dy - b->dx;
        b->steps_left = b->dx;
    }
    else
    {
        b->err        = 2.0f * b->dx - b->dy;
        b->steps_left = b->dy;
    }
}

static bool bres_step(CNC_t *cnc)
{
    CNC_Bresenham_t *b = &cnc->bres;

    if (b->steps_left <= 0.0f)
        return true;

    if (b->dx >= b->dy)
    {
        StepperMotor_Step(&cnc->motor_x);
        cnc->cur_x_steps += b->sx;
        if (b->err >= 0.0f)
        {
            StepperMotor_Step(&cnc->motor_y);
            cnc->cur_y_steps += b->sy;
            b->err -= 2.0f * b->dx;
        }
        b->err += 2.0f * b->dy;
    }
    else
    {
        StepperMotor_Step(&cnc->motor_y);
        cnc->cur_y_steps += b->sy;
        if (b->err >= 0.0f)
        {
            StepperMotor_Step(&cnc->motor_x);
            cnc->cur_x_steps += b->sx;
            b->err -= 2.0f * b->dy;
        }
        b->err += 2.0f * b->dx;
    }

    b->steps_left -= 1.0f;
    return (b->steps_left <= 0.0f);
}

static void process_command(CNC_t *cnc, const char *line)
{
    if (line[0] == 'f' && (line[1] == '\0' || line[1] == '\r' || line[1] == '\n'))
    {
        cnc->rx_terminate = true;
        return;
    }

    GCodeCommand_t cmd;
    GCode_Parse(line, &cmd);
    bool need_move = false;

    if (cmd.has_z && cmd.target_z_state != cnc->current_z_state)
    {
        cnc->target_z_state = cmd.target_z_state;
        cnc->z_steps_left   = CNC_Z_STEPS;
        cnc->motor_z.direction = (cmd.target_z_state == Z_STATE_UP)
                                 ? STEPPER_DIR_FORWARD
                                 : STEPPER_DIR_BACKWARD;
        cnc->state = CNC_STATE_MOVING_Z;
        need_move = true;
    }

    if (cmd.has_xy)
    {
        cnc->target_x_steps = GCode_MmToSteps(cmd.target_x_mm);
        cnc->target_y_steps = GCode_MmToSteps(cmd.target_y_mm);

        if (!need_move)
        {
            bres_load(cnc, cnc->target_x_steps, cnc->target_y_steps);
            cnc->state = CNC_STATE_MOVING_XY;
            need_move = true;
        }
    }

    if (!need_move)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)"ok\r\n", 4, 100);
    }
}

static void run_calibration(CNC_t *cnc)
{
    Button_t btn = Buttons_GetActive();

    if (btn == BTN_NONE)
    {
        cnc->state = CNC_STATE_IDLE;
        StepperMotor_Release(&cnc->motor_x);
        StepperMotor_Release(&cnc->motor_y);
        StepperMotor_Release(&cnc->motor_z);
        return;
    }

    uint32_t now = HAL_GetTick();
    if ((now - cnc->last_tick) < BUTTON_CAL_STEP_DELAY_MS)
        return;
    cnc->last_tick = now;

    switch (btn)
    {
        case BTN_XP:
            StepperMotor_SetDirection(&cnc->motor_x, STEPPER_DIR_FORWARD);
            StepperMotor_Step(&cnc->motor_x);
            cnc->cur_x_steps += 1.0f;
            break;
        case BTN_XN:
            StepperMotor_SetDirection(&cnc->motor_x, STEPPER_DIR_BACKWARD);
            StepperMotor_Step(&cnc->motor_x);
            cnc->cur_x_steps -= 1.0f;
            break;
        case BTN_YP:
            StepperMotor_SetDirection(&cnc->motor_y, STEPPER_DIR_FORWARD);
            StepperMotor_Step(&cnc->motor_y);
            cnc->cur_y_steps += 1.0f;
            break;
        case BTN_YN:
            StepperMotor_SetDirection(&cnc->motor_y, STEPPER_DIR_BACKWARD);
            StepperMotor_Step(&cnc->motor_y);
            cnc->cur_y_steps -= 1.0f;
            break;
        case BTN_Z1:
            StepperMotor_SetDirection(&cnc->motor_z, STEPPER_DIR_FORWARD);
            StepperMotor_Step(&cnc->motor_z);
            cnc->current_z_state = Z_STATE_UP;
            break;
        case BTN_Z2:
            StepperMotor_SetDirection(&cnc->motor_z, STEPPER_DIR_BACKWARD);
            StepperMotor_Step(&cnc->motor_z);
            cnc->current_z_state = Z_STATE_DOWN;
            break;
        default:
            break;
    }
}

void CNC_Init(CNC_t *cnc)
{
    StepperMotor_Init(&cnc->motor_x,
        M_X1_GPIO_Port, M_X1_Pin,
        M_X2_GPIO_Port, M_X2_Pin,
        M_X3_GPIO_Port, M_X3_Pin,
        M_X4_GPIO_Port, M_X4_Pin);

    StepperMotor_Init(&cnc->motor_y,
        M_Y1_GPIO_Port, M_Y1_Pin,
        M_Y2_GPIO_Port, M_Y2_Pin,
        M_Y3_GPIO_Port, M_Y3_Pin,
        M_Y4_GPIO_Port, M_Y4_Pin);

    StepperMotor_Init(&cnc->motor_z,
        M_Z1_GPIO_Port, M_Z1_Pin,
        M_Z2_GPIO_Port, M_Z2_Pin,
        M_Z3_GPIO_Port, M_Z3_Pin,
        MZ_4_GPIO_Port, MZ_4_Pin);

    cnc->state           = CNC_STATE_IDLE;
    cnc->cur_x_steps     = 0.0f;
    cnc->cur_y_steps     = 0.0f;
    cnc->target_x_steps  = 0.0f;
    cnc->target_y_steps  = 0.0f;
    cnc->current_z_state = Z_STATE_UP;
    cnc->target_z_state  = Z_STATE_IDLE;
    cnc->z_steps_left    = 0;
    cnc->last_tick       = 0;

    Backlash_Init(&cnc->backlash);

    memset(cnc->rx_buf, 0, CNC_RX_BUF_SIZE);
    cnc->rx_idx       = 0;
    cnc->rx_ready     = false;
    cnc->rx_terminate = false;

    StepperMotor_Release(&cnc->motor_x);
    StepperMotor_Release(&cnc->motor_y);
    StepperMotor_Release(&cnc->motor_z);

    HAL_UART_Receive_IT(&huart1, &cnc->rx_byte, 1);
}

void CNC_Run(CNC_t *cnc)
{
    if (cnc->rx_terminate)
    {
        StepperMotor_Release(&cnc->motor_x);
        StepperMotor_Release(&cnc->motor_y);
        StepperMotor_Release(&cnc->motor_z);
        cnc->state = CNC_STATE_IDLE;
        return;
    }

    if (Buttons_AnyActive())
    {
        cnc->state = CNC_STATE_CALIBRATING;
    }

    switch (cnc->state)
    {
        case CNC_STATE_IDLE:
        {
            if (cnc->rx_ready)
            {
                process_command(cnc, cnc->rx_buf);
                memset(cnc->rx_buf, 0, CNC_RX_BUF_SIZE);
                cnc->rx_ready = false;
            }
            break;
        }

        case CNC_STATE_MOVING_Z:
        {
            uint32_t now = HAL_GetTick();
            if ((now - cnc->last_tick) < CNC_STEP_DELAY_MS)
                return;
            cnc->last_tick = now;

            if (cnc->z_steps_left > 0)
            {
                StepperMotor_Step(&cnc->motor_z);
                cnc->z_steps_left--;
            }

            if (cnc->z_steps_left <= 0)
            {
                StepperMotor_Release(&cnc->motor_z);
                cnc->current_z_state = cnc->target_z_state;

                if (cnc->target_x_steps != cnc->cur_x_steps ||
                    cnc->target_y_steps != cnc->cur_y_steps)
                {
                    bres_load(cnc, cnc->target_x_steps, cnc->target_y_steps);
                    cnc->state = CNC_STATE_MOVING_XY;
                }
                else
                {
                    cnc->state = CNC_STATE_IDLE;
                    HAL_UART_Transmit(&huart1, (uint8_t *)"ok\r\n", 4, 100);
                }
            }
            break;
        }

        case CNC_STATE_MOVING_XY:
        {
            uint32_t now = HAL_GetTick();
            if ((now - cnc->last_tick) < CNC_STEP_DELAY_MS)
                return;
            cnc->last_tick = now;

            bool done = bres_step(cnc);
            if (done)
            {
                StepperMotor_Release(&cnc->motor_x);
                StepperMotor_Release(&cnc->motor_y);
                cnc->state = CNC_STATE_IDLE;
                HAL_UART_Transmit(&huart1, (uint8_t *)"ok\r\n", 4, 100);
            }
            break;
        }

        case CNC_STATE_CALIBRATING:
        {
            run_calibration(cnc);
            break;
        }

        default:
            cnc->state = CNC_STATE_IDLE;
            break;
    }
}

void CNC_UART_ISR(CNC_t *cnc)
{
    char c = (char)cnc->rx_byte;

    if (c == '\n' || c == '\r')
    {
        if (cnc->rx_idx > 0)
        {
            cnc->rx_buf[cnc->rx_idx] = '\0';
            cnc->rx_ready = true;
            cnc->rx_idx   = 0;
        }
    }
    else
    {
        if (cnc->rx_idx < CNC_RX_BUF_SIZE - 1)
            cnc->rx_buf[cnc->rx_idx++] = c;
    }

    HAL_UART_Receive_IT(&huart1, &cnc->rx_byte, 1);
}
