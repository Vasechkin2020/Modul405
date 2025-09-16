// ver 1.1

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>
//----
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"

#include "config.h"

#include "tim.h"
#include "usart.h"
#include "sdio.h"
#include "gpio.h"
#include "myfat.h"
#include "..\lib\FATFS\fatfs.h"

#include "code.h"
#include "motor.h"
#include "laser80M.h"
#include "slaveSPI.h"
#include "bno055.h"
#include "icm20948.h"
#include "MadgwickAHRS.h"

void SystemClock_Config(void);
volatile uint32_t millisCounter = 0;
volatile uint64_t microsCounter = 0;
volatile uint32_t overflow_count = 0; // Счётчик переполнений

int main(void)
{
  HAL_Init();           // Инициализация HAL библиотеки
  SystemClock_Config(); // Настройка системного тактирования

  MX_GPIO_Init_Only_Clock(); // Инициализация ТОЛЬКО тактирования GPIO
  MX_USART1_UART_Init();     // Инициализация USART1
  HAL_Delay(3000);
  printf("\r\n *** Modul ver 1.6 01-09-25 *** printBIM.ru *** 2025 *** \r\n");

#if MDEBUG == 1
  printf("debug.\n");
#else
  printf("non-debug !!!\n");
#endif

  initFirmware(); // Заполнение данными Прошивки
  EnableFPU();    // Включение FPU (CP10 и CP11: полный доступ) Работа с плавающей точкой

  //******************** CD Card start ********************
  MX_SDIO_SD_Init(); // Инициализация SDIO для работы с SD картой
  MX_FATFS_Init();   // Инициализация файловой системы FATFS
  mountFilesystem(); // Функция для монтирования файловой системы

  MX_I2C1_Init(); // Инициализация I2C1

  I2C_ScanDevices(&hi2c1); // Сканирование I2C шины
  BNO055_Init();
  icm20948_init(); // Инициализация ICM-20948

  float laserOffSet[4] = {1.23f, 4.56f, 7.89f, 3.1415f}; // Массив с 4 значениями калибровки лазеров
  writeFloatToFile(laserOffSet, 4, "laser.cfg");
  readFloatFromFile(laserOffSet, 4, "laser.cfg");

  unmountFilesystem();    // Функция для демонтирования файловой системы
  HAL_SD_MspDeInit(&hsd); // SDIO MSP De-Initialization Function
  // while (1)
  //   ;
  // HAL_Delay(999999);
  //******************** CD Card end ********************

  MX_GPIO_Init();

  MX_DMA_Init();

  MX_SPI1_Init();

  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();

  //-------------------- Таймеры --------------------------------------
  MX_TIM2_Init(); // Таймер на микросекунды
  MX_TIM6_Init(); // Таймер на милисекунды
  MX_TIM7_Init(); // Таймеры на моторы
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM13_Init();

  HAL_TIM_Base_Start_IT(&htim2);  // Таймер для общего цикла
  HAL_TIM_Base_Start_IT(&htim6);  // Таймер для общего цикла
  HAL_TIM_Base_Start_IT(&htim7);  // Таймер для моторов шаговых для датчиков
  HAL_TIM_Base_Start_IT(&htim10); // Таймер для моторов шаговых для датчиков
  HAL_TIM_Base_Start_IT(&htim11); // Таймер для моторов шаговых для датчиков
  HAL_TIM_Base_Start_IT(&htim13); // Таймер для моторов шаговых для датчиков

  //-------------------- Таймеры --------------------------------------

  initSPI_slave(); // Закладываем начальноы значения и инициализируем буфер DMA //  // Запуск обмена данными по SPI с использованием DMA

  initMotor(); // Начальная инициализация и настройка шаговых моторов
  // while (1)
  //   ;
  // testMotorRun();
  
  // setZeroMotor(); // Установка в ноль

  // initLaser(); // Инициализация лазеров в зависимости от типа датчкика. определяем переменные буфер приема для каждого UART

  printf("%lli LOOP !!!!!!!!!!!!!!!!!!!!!!!!!!! \r\n", timeSpi);

  while (1)
  {
    workingSPI();   // Отработка действий по обмену по шине SPI
    workingLaser(); // Отработка действий по лазерным датчикам
    workingFlag();  // Обработка флагов
    workingMotor(); // Отработка действий по шаговым моторам
    workingI2C();   // Отработка по датчикам I2C

    workingTimer(); // Отработка действий по таймеру в 1, 50, 60 милисекунд
  }
}

#if DEBUG
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
#endif

/**  * @brief System Clock Configuration  */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0}; // Настройка осцилляторов
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */