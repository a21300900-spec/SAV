#include "tim3_pwm.h"

#include <stdbool.h>

static TIM_HandleTypeDef s_tim3_handle;
static bool s_initialized = false;

static void TIM3_PWM_EnableClocks(void);
static void TIM3_PWM_InitGPIO(uint32_t channel);
static uint32_t TIM3_PWM_CalculatePeriod(uint32_t target_frequency_hz);
TIM_HandleTypeDef *TIM3_PWM_GetHandle(void)
{
    return &s_tim3_handle;
}

HAL_StatusTypeDef TIM3_PWM_Init(uint32_t channel)
{
    TIM_OC_InitTypeDef pwm_config = {0};
    HAL_StatusTypeDef status;

    TIM3_PWM_EnableClocks();
    TIM3_PWM_InitGPIO(channel);

    s_tim3_handle.Instance = TIM3;
    s_tim3_handle.Init.Prescaler = 0;
    s_tim3_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    s_tim3_handle.Init.Period = TIM3_PWM_CalculatePeriod(TIM3_PWM_FREQUENCY_HZ);
    s_tim3_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    s_tim3_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    status = HAL_TIM_PWM_Init(&s_tim3_handle);
    if (status != HAL_OK)
    {
        return status;
    }

    pwm_config.OCMode = TIM_OCMODE_PWM1;
    pwm_config.Pulse = 0;
    pwm_config.OCPolarity = TIM_OCPOLARITY_HIGH;
    pwm_config.OCFastMode = TIM_OCFAST_DISABLE;

    status = HAL_TIM_PWM_ConfigChannel(&s_tim3_handle, &pwm_config, channel);
    if (status != HAL_OK)
    {
        return status;
    }

    s_initialized = true;

    return TIM3_PWM_SetDutyCycle(channel, TIM3_PWM_MIN_DUTY_PERCENT);
}

HAL_StatusTypeDef TIM3_PWM_SetDutyCycle(uint32_t channel, float duty_percent)
{
    uint32_t period;
    float clamped;
    uint32_t pulse;

    if (!s_initialized)
    {
        return HAL_ERROR;
    }

    if (duty_percent < TIM3_PWM_MIN_DUTY_PERCENT)
    {
        clamped = TIM3_PWM_MIN_DUTY_PERCENT;
    }
    else if (duty_percent > TIM3_PWM_MAX_DUTY_PERCENT)
    {
        clamped = TIM3_PWM_MAX_DUTY_PERCENT;
    }
    else
    {
        clamped = duty_percent;
    }

    period = __HAL_TIM_GET_AUTORELOAD(&s_tim3_handle);
    pulse = (uint32_t)(((float)(period + 1U) * clamped) / 100.0f);

    __HAL_TIM_SET_COMPARE(&s_tim3_handle, channel, pulse);

    return HAL_TIM_PWM_Start(&s_tim3_handle, channel);
}

static void TIM3_PWM_EnableClocks(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
}

static void TIM3_PWM_InitGPIO(uint32_t channel)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    switch (channel)
    {
        case TIM_CHANNEL_1:
            gpio.Pin = GPIO_PIN_6;
            gpio.Alternate = GPIO_AF2_TIM3;
            HAL_GPIO_Init(GPIOA, &gpio);
            break;
        case TIM_CHANNEL_2:
            gpio.Pin = GPIO_PIN_7;
            gpio.Alternate = GPIO_AF2_TIM3;
            HAL_GPIO_Init(GPIOA, &gpio);
            break;
        case TIM_CHANNEL_3:
            gpio.Pin = GPIO_PIN_0;
            gpio.Alternate = GPIO_AF2_TIM3;
            HAL_GPIO_Init(GPIOB, &gpio);
            break;
        case TIM_CHANNEL_4:
            gpio.Pin = GPIO_PIN_1;
            gpio.Alternate = GPIO_AF2_TIM3;
            HAL_GPIO_Init(GPIOB, &gpio);
            break;
        default:
            break;
    }
}

static uint32_t TIM3_PWM_CalculatePeriod(uint32_t target_frequency_hz)
{
    RCC_ClkInitTypeDef clock_config;
    uint32_t flash_latency;
    uint32_t pclk1;
    uint32_t timer_clock;
    uint32_t period;

    HAL_RCC_GetClockConfig(&clock_config, &flash_latency);
    pclk1 = HAL_RCC_GetPCLK1Freq();

    if (clock_config.APB1CLKDivider != RCC_HCLK_DIV1)
    {
        timer_clock = pclk1 * 2U;
    }
    else
    {
        timer_clock = pclk1;
    }

    if (target_frequency_hz == 0U)
    {
        return 0U;
    }

    period = (uint32_t)((timer_clock + (target_frequency_hz / 2U)) / target_frequency_hz) - 1U;

    if (period > 0xFFFFU)
    {
        period = 0xFFFFU;
    }

    return period;
}

