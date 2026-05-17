#include "Buttons.h"

static volatile Button_t active_button = BTN_NONE;

Button_t Buttons_GetActive(void)
{
    return active_button;
}

bool Buttons_AnyActive(void)
{
    return active_button != BTN_NONE;
}

/*
 * HAL_GPIO_EXTI_Callback is a __weak function defined by the HAL.
 * The stm32f0xx_it.c handlers call HAL_GPIO_EXTI_IRQHandler(pin)
 * which clears the pending bit and then calls this callback.
 * We override it here to detect button press/release purely by interrupt.
 *
 * On falling edge (press):  set active_button, switch EXTI to rising edge.
 * On rising edge (release): clear active_button, switch EXTI back to falling.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    Button_t       btn  = BTN_NONE;
    GPIO_TypeDef  *port = NULL;
    uint32_t       line = (uint32_t)GPIO_Pin;

    if (GPIO_Pin == B_XP_Pin)      { btn = BTN_XP; port = B_XP_GPIO_Port; }
    else if (GPIO_Pin == B_XN_Pin) { btn = BTN_XN; port = B_XN_GPIO_Port; }
    else if (GPIO_Pin == B_YP_Pin) { btn = BTN_YP; port = B_YP_GPIO_Port; }
    else if (GPIO_Pin == B_YN_Pin) { btn = BTN_YN; port = B_YN_GPIO_Port; }
    else if (GPIO_Pin == B1_Z_Pin) { btn = BTN_Z1; port = B1_Z_GPIO_Port; }
    else if (GPIO_Pin == B2_Z_Pin) { btn = BTN_Z2; port = B2_Z_GPIO_Port; }
    else return;

    if (HAL_GPIO_ReadPin(port, GPIO_Pin) == GPIO_PIN_RESET)
    {
        active_button = btn;
        EXTI->RTSR |=  line;
        EXTI->FTSR &= ~line;
    }
    else
    {
        if (active_button == btn)
            active_button = BTN_NONE;
        EXTI->FTSR |=  line;
        EXTI->RTSR &= ~line;
    }
}
