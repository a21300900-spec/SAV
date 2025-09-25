#include "stm32f4xx_hal.h"
#include "tim3_pwm.h"

static void SystemClock_Config(void);
static void Error_Handler(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    if (TIM3_PWM_Init(TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    if (TIM3_PWM_SetDutyCycle(TIM_CHANNEL_1, 50.0f) != HAL_OK)
    {
        Error_Handler();
    }

    while (1)
    {
        /* Application code */
    }
}

static void SystemClock_Config(void)
{
    /* Configure system clock as required by the application */
}

static void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
