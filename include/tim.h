
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"

    extern TIM_HandleTypeDef htim6;
    extern TIM_HandleTypeDef htim7;

    void MX_TIM6_Init(void);
    void MX_TIM7_Init(void);

    void Set_Timer7_Period(uint32_t new_period); // Функция для изменения периода таймера TIM7
    // void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin);  // Callback-функция, которая вызывается при срабатывании прерывания
    // void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin); // Callback-функция, которая вызывается при срабатывании прерывания
    void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin); // Callback-функция, которая вызывается при срабатывании прерывания STM32F4

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */
