/* Includes ------------------------------------------------------------------*/
#include "sdio.h"

SD_HandleTypeDef hsd;

void MX_SDIO_SD_Init(void) // SDIO Initialization Function
{
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 118; // Начальная частота ~400 кГц
  // hsd.Init.ClockDiv = 118; //  // Частота SDIO SDIO_CK = SDIOCLK / [CLKDIV + 2]. The output clock frequency can vary between 187 KHz and 24 MHz. It is advised to keep the default ClockDiv value (0) to have a maximum SDIO_CK frequency of 24 MHz.

  if (HAL_SD_Init(&hsd) != HAL_OK) // Ошибка инициализации контроллера SDIO
  {
    uint32_t sdError = HAL_SD_GetError(&hsd); // Получаем код ошибки SDIO
    uint32_t sdError2 = hsd.ErrorCode;
    
    printf("Error init SDIO (0x%08lX)  = %lu Error2= %lu \r\n", sdError, sdError, sdError2);

    switch (sdError) // Расшифровываем ошибку (используем SDMMC_ERROR вместо SDIO_ERROR)
    {
    case SDMMC_ERROR_CMD_CRC_FAIL:
      DEBUG_PRINTF("SD ERROR: CMD CRC FAIL (неправильный CRC команды)\r\n");
      break;
    case SDMMC_ERROR_DATA_CRC_FAIL:
      DEBUG_PRINTF("SD ERROR: DATA CRC FAIL (ошибка CRC данных)\r\n");
      break;
    case SDMMC_ERROR_CMD_RSP_TIMEOUT:
      DEBUG_PRINTF("SD ERROR: CMD RESPONSE TIMEOUT (карта не ответила)\r\n");
      DEBUG_PRINTF("Проверьте:\r\n");
      DEBUG_PRINTF("1. Карта вставлена?\r\n");
      DEBUG_PRINTF("2. Частота SDIO слишком высокая? Попробуйте ClockDiv=206\r\n");
      DEBUG_PRINTF("3. Контакты SDIO подключены правильно?\r\n");
      break;
    case SDMMC_ERROR_DATA_TIMEOUT:
      DEBUG_PRINTF("SD ERROR: DATA TIMEOUT (таймаут передачи данных)\r\n");
      break;
    case SDMMC_ERROR_TX_UNDERRUN:
      DEBUG_PRINTF("SD ERROR: TX UNDERRUN (буфер передачи пуст)\r\n");
      break;
    case SDMMC_ERROR_RX_OVERRUN:
      DEBUG_PRINTF("SD ERROR: RX OVERRUN (буфер приёма переполнен)\r\n");
      break;
    default:
      DEBUG_PRINTF("SD ERROR: Unknown error (0x%08lX)\r\n", sdError);
      break;

      Error_Handler();
    }
  }
}

void HAL_SD_MspInit(SD_HandleTypeDef *sdHandle) // SDIO MSP Initialization Function
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (sdHandle->Instance == SDIO)
  {
    /* SDIO clock enable */
    __HAL_RCC_SDIO_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**SDIO GPIO Configuration
    PC8     ------> SDIO_D0
    PC12     ------> SDIO_CK
    PD2     ------> SDIO_CMD
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // Подтяжка для стабильности
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // Подтяжка для стабильности
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  }
}

void HAL_SD_MspDeInit(SD_HandleTypeDef *sdHandle) // SDIO MSP De-Initialization Function
{

  if (sdHandle->Instance == SDIO)
  {
    /* Peripheral clock disable */
    __HAL_RCC_SDIO_CLK_DISABLE();

    /**SDIO GPIO Configuration
    PC8     ------> SDIO_D0
    PC12     ------> SDIO_CK
    PD2     ------> SDIO_CMD
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_8 | GPIO_PIN_12);
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);
  }
}