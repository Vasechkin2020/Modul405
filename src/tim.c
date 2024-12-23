
#include "tim.h"
#include "config.h"

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

extern void timer6();
extern void timer7();
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

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 167; // 167+1 получается делитель для 168 МГц в 1 микросекунду
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

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 167;
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