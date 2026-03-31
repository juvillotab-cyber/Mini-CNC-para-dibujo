#include "CNC.h"

/* ═══════════════════════════════════════════
 *  HELPERS MOTOR
 * ═══════════════════════════════════════════ */

static void stepper_apply(StepperMotor_t *m)
{
    const uint8_t (*seq)[4] = (m->step_mode == CNC_STEP_MODE_8) ? SEQ_8 : SEQ_4;
    for (int i = 0; i < 4; i++)
        HAL_GPIO_WritePin(m->port[i], m->pin[i],
                          seq[m->seq_idx][i] ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void stepper_step(StepperMotor_t *m)
{
    m->seq_idx = (uint8_t)((m->seq_idx + m->step_mode + m->direction) % m->step_mode);
    stepper_apply(m);
    m->position += m->direction;
}

static void stepper_release(StepperMotor_t *m)
{
    for (int i = 0; i < 4; i++)
        HAL_GPIO_WritePin(m->port[i], m->pin[i], GPIO_PIN_RESET);
}

/* ═══════════════════════════════════════════
 *  BRESENHAM
 * ═══════════════════════════════════════════ */

static void bres_load(CNC_t *cnc, int32_t tx, int32_t ty)
{
    CNC_Bresenham_t *b = &cnc->bres;
    int32_t dx = tx - cnc->cur_x;
    int32_t dy = ty - cnc->cur_y;

    b->sx = (dx >= 0) ? 1 : -1;
    b->sy = (dy >= 0) ? 1 : -1;
    b->dx = (dx >= 0) ? dx : -dx;
    b->dy = (dy >= 0) ? dy : -dy;

    cnc->motor_x.direction = (int8_t)b->sx;
    cnc->motor_y.direction = (int8_t)b->sy;

    if (b->dx >= b->dy) {
        b->err        = 2 * b->dy - b->dx;
        b->steps_left = b->dx;
    } else {
        b->err        = 2 * b->dx - b->dy;
        b->steps_left = b->dy;
    }
}

static bool bres_step(CNC_t *cnc)
{
    CNC_Bresenham_t *b = &cnc->bres;
    if (b->steps_left <= 0) return true;

    if (b->dx >= b->dy) {
        stepper_step(&cnc->motor_x);
        cnc->cur_x += b->sx;
        if (b->err >= 0) {
            stepper_step(&cnc->motor_y);
            cnc->cur_y += b->sy;
            b->err -= 2 * b->dx;
        }
        b->err += 2 * b->dy;
    } else {
        stepper_step(&cnc->motor_y);
        cnc->cur_y += b->sy;
        if (b->err >= 0) {
            stepper_step(&cnc->motor_x);
            cnc->cur_x += b->sx;
            b->err -= 2 * b->dy;
        }
        b->err += 2 * b->dx;
    }

    b->steps_left--;
    return (b->steps_left == 0);
}

/* ═══════════════════════════════════════════
 *  PARSER GCODE  (solo G0/G1 por ahora)
 *  Formato esperado: "G0 X10 Y20\n"
 * ═══════════════════════════════════════════ */

static void gcode_parse(CNC_t *cnc, char *line)
{
    float x = cnc->cur_x;
    float y = cnc->cur_y;

    char *px = strstr(line, "X");
    char *py = strstr(line, "Y");

    if (px) x = (int32_t)atof(px + 1) * CNC_STEPS_PER_MM;
    if (py) y = (int32_t)atof(py + 1) * CNC_STEPS_PER_MM;

    bres_load(cnc, x, y);
    cnc->state = CNC_MOVING;
}

/* ═══════════════════════════════════════════
 *  API PÚBLICA
 * ═══════════════════════════════════════════ */

void CNC_Init(CNC_t *cnc, uint8_t step_mode)
{
    /* Motor X */
    cnc->motor_x.port[0] = M_X1_GPIO_Port; cnc->motor_x.pin[0] = M_X1_Pin;
    cnc->motor_x.port[1] = M_X2_GPIO_Port; cnc->motor_x.pin[1] = M_X2_Pin;
    cnc->motor_x.port[2] = M_X3_GPIO_Port; cnc->motor_x.pin[2] = M_X3_Pin;
    cnc->motor_x.port[3] = M_X4_GPIO_Port; cnc->motor_x.pin[3] = M_X4_Pin;
    cnc->motor_x.step_mode = step_mode;
    cnc->motor_x.seq_idx   = 0;
    cnc->motor_x.direction = 1;
    cnc->motor_x.position  = 0;

    /* Motor Y */
    cnc->motor_y.port[0] = M_Y1_GPIO_Port; cnc->motor_y.pin[0] = M_Y1_Pin;
    cnc->motor_y.port[1] = M_Y2_GPIO_Port; cnc->motor_y.pin[1] = M_Y2_Pin;
    cnc->motor_y.port[2] = M_Y3_GPIO_Port; cnc->motor_y.pin[2] = M_Y3_Pin;
    cnc->motor_y.port[3] = M_Y4_GPIO_Port; cnc->motor_y.pin[3] = M_Y4_Pin;
    cnc->motor_y.step_mode = step_mode;
    cnc->motor_y.seq_idx   = 0;
    cnc->motor_y.direction = 1;
    cnc->motor_y.position  = 0;

    /* Motor Z (reservado) */
    cnc->motor_z.port[0] = M_Z1_GPIO_Port; cnc->motor_z.pin[0] = M_Z1_Pin;
    cnc->motor_z.port[1] = M_Z2_GPIO_Port; cnc->motor_z.pin[1] = M_Z2_Pin;
    cnc->motor_z.port[2] = M_Z3_GPIO_Port; cnc->motor_z.pin[2] = M_Z3_Pin;
    cnc->motor_z.port[3] = MZ_4_GPIO_Port; cnc->motor_z.pin[3] = MZ_4_Pin;
    cnc->motor_z.step_mode = step_mode;
    cnc->motor_z.seq_idx   = 0;
    cnc->motor_z.direction = 1;
    cnc->motor_z.position  = 0;

    /* Estado */
    cnc->state     = CNC_IDLE;
    cnc->cur_x     = 0;
    cnc->cur_y     = 0;
    cnc->last_tick = 0;

    /* Buffer UART */
    memset(cnc->rx_buf, 0, CNC_RX_BUF_SIZE);
    cnc->rx_idx   = 0;
    cnc->rx_ready = false;

    stepper_release(&cnc->motor_x);
    stepper_release(&cnc->motor_y);
    stepper_release(&cnc->motor_z);

    /* Arranca recepción byte a byte */
    HAL_UART_Receive_IT(&huart1, &cnc->rx_byte, 1);
}

/*
 * CNC_UART_ISR — llamar desde HAL_UART_RxCpltCallback en main.c:
 *
 *   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 *       if (huart->Instance == USART2) CNC_UART_ISR(&cnc);
 *   }
 */
void CNC_UART_ISR(CNC_t *cnc)
{
    char c = (char)cnc->rx_byte;

    if (c == '\n' || c == '\r') {
        if (cnc->rx_idx > 0) {
            cnc->rx_buf[cnc->rx_idx] = '\0';
            cnc->rx_ready = true;          /* señal para CNC_Run */
        }
    } else {
        if (cnc->rx_idx < CNC_RX_BUF_SIZE - 1)
            cnc->rx_buf[cnc->rx_idx++] = c;
    }

    /* Reactiva la interrupción para el siguiente byte */
    HAL_UART_Receive_IT(&huart1, &cnc->rx_byte, 1);
}

/*
 * CNC_Run — while(1)
 */
void CNC_Run(CNC_t *cnc)
{
    switch (cnc->state)
    {
        case CNC_IDLE:
        {
            if (cnc->rx_ready) {
                gcode_parse(cnc, cnc->rx_buf);        /* carga movimiento   */
                memset(cnc->rx_buf, 0, CNC_RX_BUF_SIZE);
                cnc->rx_idx   = 0;
                cnc->rx_ready = false;
            }
            break;
        }

        case CNC_MOVING:
        {
            uint32_t now = HAL_GetTick();
            if ((now - cnc->last_tick) < CNC_STEP_DELAY_MS) return;
            cnc->last_tick = now;

            bool done = bres_step(cnc);
            if (done) {
                stepper_release(&cnc->motor_x);
                stepper_release(&cnc->motor_y);
                cnc->state = CNC_IDLE;

                HAL_UART_Transmit(&huart1, (uint8_t*)"ok\r\n", 4, 100);
            }
            break;
        }

        default:
            cnc->state = CNC_IDLE;
            break;
    }
}