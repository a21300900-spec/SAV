#ifndef TIM3_PWM_H
#define TIM3_PWM_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TIM3_PWM_FREQUENCY_HZ 1800U
#define TIM3_PWM_MIN_DUTY_PERCENT 7.0f
#define TIM3_PWM_MAX_DUTY_PERCENT 94.0f

HAL_StatusTypeDef TIM3_PWM_Init(uint32_t channel);
HAL_StatusTypeDef TIM3_PWM_SetDutyCycle(uint32_t channel, float duty_percent);
TIM_HandleTypeDef *TIM3_PWM_GetHandle(void);

#ifdef __cplusplus
}
#endif

#endif /* TIM3_PWM_H */
