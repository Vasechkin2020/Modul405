
#include "i2c.h"

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration    PB6     ------> I2C1_SCL    PB7     ------> I2C1_SDA    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration     PB6     ------> I2C1_SCL    PB7     ------> I2C1_SDA    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);
        /* I2C1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
  }
}

/* USER CODE BEGIN 1 */

// Функция восстановления шины I2C (Software Reset)
// Вызывать ТОЛЬКО из основного цикла (main), не из прерывания!
void I2C1_ClearBusyFlagErratum(void)
{
    printf("+++ I2C1_ClearBusyFlagErratum.\n");
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. Сохраняем настройки пинов (PB6=SCL, PB7=SDA)
    GPIO_TypeDef* SCL_PORT = GPIOB;
    uint16_t SCL_PIN = GPIO_PIN_6;
    GPIO_TypeDef* SDA_PORT = GPIOB;
    uint16_t SDA_PIN = GPIO_PIN_7;

    // 2. Отключаем периферию I2C и прерывания, чтобы не мешали
    __HAL_I2C_DISABLE(&hi2c1);
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);

    // 3. Настраиваем пины как обычные выходы (Output Open Drain)
    // Чтобы мы могли вручную "дёргать" ногами
    GPIO_InitStruct.Pin = SCL_PIN | SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 4. Процедура "9 импульсов" (ручная генерация тактов)
    // Это заставляет датчик отдать бит и отпустить шину SDA
    HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_SET); // SDA High
    
    for (uint8_t i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET);   // SCL High
        HAL_Delay(1); 
        HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_RESET); // SCL Low
        HAL_Delay(1);
    }

    // 5. Генерируем STOP условие (SCL High, затем SDA Low -> High)
    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_SET);
    HAL_Delay(1);

    // 6. Возвращаем пины обратно в режим I2C (Alternate Function)
    GPIO_InitStruct.Pin = SCL_PIN | SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1; // Важно: AF4 для F405 I2C1
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 7. Сброс самого модуля I2C через RCC (Software Reset)
    __HAL_RCC_I2C1_FORCE_RESET();
    HAL_Delay(2);
    __HAL_RCC_I2C1_RELEASE_RESET();

    // 8. Включаем прерывания обратно и инициализируем I2C заново
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    
    // Повторная инициализация HAL
    MX_I2C1_Init(); 
}
/* USER CODE END 1 */