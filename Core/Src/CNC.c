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

static void bres_load(CNC_t *cnc, float tx, float ty)
{
    CNC_Bresenham_t *b = &cnc->bres;
    float dx = tx - cnc->cur_x;
    float dy = ty - cnc->cur_y;

    b->sx = (dx >= 0) ? 1.0f : -1.0f;
    b->sy = (dy >= 0) ? 1.0f : -1.0f;
    b->dx = fabsf(dx);
    b->dy = fabsf(dy);

    cnc->motor_x.direction = (int8_t)b->sx;
    cnc->motor_y.direction = (int8_t)b->sy;

    if (b->dx >= b->dy) {
        b->err        = 2.0f * b->dy - b->dx;
        b->steps_left = b->dx;
    } else {
        b->err        = 2.0f * b->dx - b->dy;
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
            b->err -= 2.0f * b->dx;
        }
        b->err += 2.0f * b->dy;
    } else {
        stepper_step(&cnc->motor_y);
        cnc->cur_y += b->sy;
        if (b->err >= 0) {
            stepper_step(&cnc->motor_x);
            cnc->cur_x += b->sx;
            b->err -= 2.0f * b->dy;
        }
        b->err += 2.0f * b->dx;
    }

    b->steps_left--;
    return (b->steps_left == 0);
}

/* ═══════════════════════════════════════════
 *  PARSER GCODE
 *  Formato: "G0 X10.5 Y20.3 Z-2.0"
 * ═══════════════════════════════════════════ */

static void gcode_parse(CNC_t *cnc, char *line)
{
    char *px = strstr(line, "X");
    char *py = strstr(line, "Y");
    char *pz = strstr(line, "Z");

    /* Coordenadas XY en pasos */
    if (px) cnc->target_x = (float)atof(px + 1) * CNC_STEPS_PER_MM;
    else    cnc->target_x = cnc->cur_x;

    if (py) cnc->target_y = (float)atof(py + 1) * CNC_STEPS_PER_MM;
    else    cnc->target_y = cnc->cur_y;

    /* Z: positivo sube, negativo baja */
    if (pz) {
        float z_mm = (float)atof(pz + 1);
        cnc->z_direction        = (z_mm > 0) ? 1 : -1;
        cnc->z_steps_left       = fabsf(z_mm) * CNC_STEPS_PER_MM;
        cnc->motor_z.direction  = cnc->z_direction;
        cnc->state              = CNC_MOVING_Z;   /* Z primero */
    } else if (px || py) {
        bres_load(cnc, cnc->target_x, cnc->target_y);
        cnc->state = CNC_MOVING_XY;
    }
    /* si no hay nada, se queda en IDLE */
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

    /* Motor Z */
    cnc->motor_z.port[0] = M_Z1_GPIO_Port; cnc->motor_z.pin[0] = M_Z1_Pin;
    cnc->motor_z.port[1] = M_Z2_GPIO_Port; cnc->motor_z.pin[1] = M_Z2_Pin;
    cnc->motor_z.port[2] = M_Z3_GPIO_Port; cnc->motor_z.pin[2] = M_Z3_Pin;
    cnc->motor_z.port[3] = MZ_4_GPIO_Port; cnc->motor_z.pin[3] = MZ_4_Pin;
    cnc->motor_z.step_mode = step_mode;
    cnc->motor_z.seq_idx   = 0;
    cnc->motor_z.direction = 1;
    cnc->motor_z.position  = 0;

    /* Estado */
    cnc->state        = CNC_IDLE;
    cnc->cur_x        = 0;
    cnc->cur_y        = 0;
    cnc->target_x     = 0;
    cnc->target_y     = 0;
    cnc->z_steps_left = 0;
    cnc->z_direction  = 1;
    cnc->last_tick    = 0;

    /* Buffer UART */
    memset(cnc->rx_buf, 0, CNC_RX_BUF_SIZE);
    cnc->rx_idx   = 0;
    cnc->rx_ready = false;

    stepper_release(&cnc->motor_x);
    stepper_release(&cnc->motor_y);
    stepper_release(&cnc->motor_z);

    HAL_UART_Receive_IT(&huart1, &cnc->rx_byte, 1);
}

/*
 * Llamar desde main.c:
 *
 *   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 *       if (huart->Instance == USART1) CNC_UART_ISR(&cnc);
 *   }
 */
void CNC_UART_ISR(CNC_t *cnc)
{
    char c = (char)cnc->rx_byte;

    if (c == '\n' || c == '\r') {
        if (cnc->rx_idx > 0) {
            cnc->rx_buf[cnc->rx_idx] = '\0';
            cnc->rx_ready = true;
            cnc->rx_idx   = 0;   /* listo para la siguiente línea */
        }
    } else {
        if (cnc->rx_idx < CNC_RX_BUF_SIZE - 1)
            cnc->rx_buf[cnc->rx_idx++] = c;
    }

    HAL_UART_Receive_IT(&huart1, &cnc->rx_byte, 1);
}

/*
 * CNC_Run — while(1)
 */
void CNC_Run(CNC_t *cnc)
{
    switch (cnc->state)
    {
        /* ── IDLE: espera línea nueva ── */
        case CNC_IDLE:
        {
            if (cnc->rx_ready) {
                gcode_parse(cnc, cnc->rx_buf);
                memset(cnc->rx_buf, 0, CNC_RX_BUF_SIZE);
                cnc->rx_ready = false;
            }
            break;
        }

        /* ── MOVING_Z: mueve Z, luego pasa a XY si hay destino ── */
        case CNC_MOVING_Z:
        {
            uint32_t now = HAL_GetTick();
            if ((now - cnc->last_tick) < CNC_STEP_DELAY_MS) return;
            cnc->last_tick = now;

            if (cnc->z_steps_left > 0) {
                stepper_step(&cnc->motor_z);
                cnc->z_steps_left--;
            }

            if (cnc->z_steps_left <= 0) {
                stepper_release(&cnc->motor_z);

                /* ¿hay movimiento XY pendiente? */
                if (cnc->target_x != cnc->cur_x || cnc->target_y != cnc->cur_y) {
                    bres_load(cnc, cnc->target_x, cnc->target_y);
                    cnc->state = CNC_MOVING_XY;
                } else {
                    cnc->state = CNC_IDLE;
                    HAL_UART_Transmit(&huart1, (uint8_t*)"ok\r\n", 4, 100);
                }
            }
            break;
        }

        /* ── MOVING_XY: Bresenham paso a paso ── */
        case CNC_MOVING_XY:
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