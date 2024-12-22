
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

    // Установите DEBUG для включения отладочной информации
#define DEBUG 1 // Поставьте 0 для отключения отладочной информации

#if DEBUG
    #define DEBUG_PRINTF(...) printf(__VA_ARGS__)
    // #define DEBUG_PRINTF(fmt,...) printf("GREEN" fmt , ##__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...) (void)0 // Приведение 0 к типу void, ничего не делает
#endif


#include "stm32f4xx_hal.h"

void Error_Handler(void);

#define Led2_Pin GPIO_PIN_2
#define Led2_GPIO_Port GPIOC
#define Led1_Pin GPIO_PIN_15
#define Led1_GPIO_Port GPIOA

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
