
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
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

    struct dataUART
    {
        uint8_t flag;      // Флаг готовности данных
        uint8_t num;       // Номер UART
        uint32_t status;   // Статус данных
        uint8_t statusDMA; // Статус вызова нового DMA
        uint32_t distance; // Дистанция по последнему хорошему измерению
        uint16_t quality;  // Качество сигнала
        float angle;       // Угол в котором находился мотор в момент когда пришли данные по измерению
        uint32_t time;     // Время измерения от начала запуска программы
        uint32_t timeSend; // Время отправки запроса к датчику
        uint8_t *adr;      // Адрес буфера
        uint16_t len;      // Длинна данных в буфере
        UART_HandleTypeDef *huart;
    };

    void Error_Handler(void);

#define Led2_Pin GPIO_PIN_2
#define Led2_GPIO_Port GPIOC
#define Led1_Pin GPIO_PIN_15
#define Led1_GPIO_Port GPIOA

#define Analiz_Pin GPIO_PIN_2
#define Analiz_GPIO_Port GPIOB

#define En_Motor_Pin GPIO_PIN_5
#define En_Motor_GPIO_Port GPIOC

#define Dir_Motor0_Pin GPIO_PIN_11
#define Dir_Motor0_GPIO_Port GPIOC
#define Step_Motor0_Pin GPIO_PIN_10
#define Step_Motor0_GPIO_Port GPIOC

#define Dir_Motor1_Pin GPIO_PIN_12
#define Dir_Motor1_GPIO_Port GPIOA
#define Step_Motor1_Pin GPIO_PIN_11
#define Step_Motor1_GPIO_Port GPIOA

#define Dir_Motor2_Pin GPIO_PIN_1
#define Dir_Motor2_GPIO_Port GPIOC
#define Step_Motor2_Pin GPIO_PIN_3
#define Step_Motor2_GPIO_Port GPIOC

#define Dir_Motor3_Pin GPIO_PIN_1
#define Dir_Motor3_GPIO_Port GPIOB
#define Step_Motor3_Pin GPIO_PIN_0
#define Step_Motor3_GPIO_Port GPIOB

#define laserEn_Pin GPIO_PIN_13
#define laserEn_GPIO_Port GPIOB

#define micMotor0_Pin GPIO_PIN_14
#define micMotor0_GPIO_Port GPIOC
#define micMotor0_EXTI_IRQn EXTI15_10_IRQn

#define micMotor1_Pin GPIO_PIN_13
#define micMotor1_GPIO_Port GPIOC
#define micMotor1_EXTI_IRQn EXTI15_10_IRQn

#define micMotor2_Pin GPIO_PIN_0
#define micMotor2_GPIO_Port GPIOC
#define micMotor2_EXTI_IRQn EXTI0_IRQn

#define micMotor3_Pin GPIO_PIN_15
#define micMotor3_GPIO_Port GPIOC
#define micMotor3_EXTI_IRQn EXTI15_10_IRQn


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
