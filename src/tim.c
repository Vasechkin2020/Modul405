
#include "tim.h"
#include "config.h"

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim13;

extern void timer6();
extern void timer7();
extern void timer10();
extern void timer11();
extern void timer13();

extern void isrMicMotor0();
extern void isrMicMotor1();
extern void isrMicMotor2();
extern void isrMicMotor3();
// extern uint32_t millis();
extern volatile uint32_t millisCounter;

/* TIM6 init function */
void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 83; // 83+1 получается делитель для 168 МГц в 1 микросекунду делить в 2 раза 
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/* TIM7 init function */
void MX_TIM7_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 83; //APB1 : Поддерживает "медленные" периферийные устройства.Частота обычно ниже (например, 42 МГц для STM32F4).К этой шине подключаются таймеры: TIM2–TIM7 , TIM12–TIM14 .
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/* TIM10 init function */
void MX_TIM10_Init(void)
{
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 167; // Подключен к быстрой шине APB2 APB2 :Поддерживает "быстрые" периферийные устройства. Частота обычно выше (например, 84 МГц для STM32F4).К этой шине подключаются таймеры: TIM1 , TIM8–TIM11 .
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 1000;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
}
/* TIM11 init function */
void MX_TIM11_Init(void)
{
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 167;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 1000;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
}
/* TIM13 init function */
void MX_TIM13_Init(void)
{
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 83;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 1000;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
}


void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *tim_baseHandle)
{
  if (tim_baseHandle->Instance == TIM6)
  {
    /* TIM6 clock enable */
    __HAL_RCC_TIM6_CLK_ENABLE();

    /* TIM6 interrupt Init */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  }
  else if (tim_baseHandle->Instance == TIM7)
  {
    /* TIM7 clock enable */
    __HAL_RCC_TIM7_CLK_ENABLE();

    /* TIM7 interrupt Init */
    HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
  }
  else if(tim_baseHandle->Instance==TIM10)
  {
    /* TIM10 clock enable */
    __HAL_RCC_TIM10_CLK_ENABLE();

    /* TIM10 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  }
  else if(tim_baseHandle->Instance==TIM11)
  {
    /* TIM11 clock enable */
    __HAL_RCC_TIM11_CLK_ENABLE();

    /* TIM11 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
  }
  else if(tim_baseHandle->Instance==TIM13)
  {
    /* TIM13 clock enable */
    __HAL_RCC_TIM13_CLK_ENABLE();

    /* TIM13 interrupt Init */
    HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *tim_baseHandle)
{

  if (tim_baseHandle->Instance == TIM6)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM6_CLK_DISABLE();

    /* TIM6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
  }
  else if (tim_baseHandle->Instance == TIM7)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM7_CLK_DISABLE();

    /* TIM7 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM7_IRQn);
  }
  else if(tim_baseHandle->Instance==TIM10)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM10_CLK_DISABLE();

    /* TIM10 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
  }
  else if(tim_baseHandle->Instance==TIM11)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM11_CLK_DISABLE();

    /* TIM11 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM1_TRG_COM_TIM11_IRQn);
  }
  else if(tim_baseHandle->Instance==TIM13)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM13_CLK_DISABLE();

    /* TIM13 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
  }
}
//************************************************

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    timer6();
  }
  else if (htim->Instance == TIM7)
  {
    timer7();
  }
  else if (htim->Instance == TIM10)
  {
    timer10();
  }
  else if (htim->Instance == TIM11)
  {
    timer11();
  }
  else if (htim->Instance == TIM13)
  {
    timer13();
  }
}
// Callback-функция, которая вызывается при срабатывании прерывания STM32F4
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  HAL_GPIO_TogglePin(Led2_GPIO_Port, Led2_Pin);     // Инвертирование состояния выхода.
  if (GPIO_Pin == micMotor0_Pin) // Действия при нажатии кнопки (например, переключение светодиода)
  {
    isrMicMotor0();
  }
  else if (GPIO_Pin == micMotor1_Pin) // Действия при нажатии кнопки (например, переключение светодиода)
  {
    isrMicMotor1();
  }
  else if (GPIO_Pin == micMotor2_Pin) // Действия при нажатии кнопки (например, переключение светодиода)
  {
    isrMicMotor2();
  }
  else if (GPIO_Pin == micMotor3_Pin) // Действия при нажатии кнопки (например, переключение светодиода)
  {
    isrMicMotor3();
  }
}

// // Callback-функция, которая вызывается при срабатывании прерывания По спаду уровня
// void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
// {
//   // if (millisCounter > 50) // Если с момента запуска прошло более 500 милисекунд то реагируем на прерывания
//   HAL_GPIO_TogglePin(Led2_GPIO_Port, Led2_Pin); // Инвертирование состояния выхода.

//   if (1) // Если с момента запуска прошло более 500 милисекунд то реагируем на прерывания
//   {

//     if (GPIO_Pin == micMotor0_Pin) // Действия при нажатии кнопки (например, переключение светодиода)
//     {
//       isrMicMotor0();
//     }
//     else if (GPIO_Pin == micMotor1_Pin) // Действия при нажатии кнопки (например, переключение светодиода)
//     {
//       isrMicMotor1();
//     }
//     else if (GPIO_Pin == micMotor2_Pin) // Действия при нажатии кнопки (например, переключение светодиода)
//     {
//       isrMicMotor2();
//     }
//     else if (GPIO_Pin == micMotor3_Pin) // Действия при нажатии кнопки (например, переключение светодиода)
//     {
//       isrMicMotor3();
//     }
//   }
// }
// // Callback-функция, которая вызывается при срабатывании прерывания По подьему уровня
// void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
// {
//   HAL_GPIO_TogglePin(Led2_GPIO_Port, Led2_Pin); // Инвертирование состояния выхода.

//   if (GPIO_Pin == micMotor0_Pin) // Действия при нажатии кнопки (например, переключение светодиода)
//   {
//   }
//   else if (GPIO_Pin == micMotor1_Pin) // Действия при нажатии кнопки (например, переключение светодиода)
//   {
//   }
//   else if (GPIO_Pin == micMotor2_Pin) // Действия при нажатии кнопки (например, переключение светодиода)
//   {
//   }
//   else if (GPIO_Pin == micMotor3_Pin) // Действия при нажатии кнопки (например, переключение светодиода)
//   {
//   }
// }