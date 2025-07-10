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
#include "gpio.h"

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
  HAL_Init();
  SystemClock_Config();

  EnableFPU(); // Включение FPU (CP10 и CP11: полный доступ) Работа с плавающей точкой

  MX_GPIO_Init();
  HAL_GPIO_WritePin(ledGreen_GPIO_Port, ledGreen_Pin, GPIO_PIN_SET); // Сразу включаем светодиод что началась загрузка
  MX_DMA_Init();

  MX_I2C1_Init();

  MX_SPI1_Init();

  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();

  // for (int i = 0; i < 10; i++)
  // {
  //   HAL_GPIO_TogglePin(Dir_Motor3_GPIO_Port, Dir_Motor3_Pin);
  //   HAL_GPIO_TogglePin(Step_Motor3_GPIO_Port, Step_Motor3_Pin);
  //   HAL_Delay(100);
  // }
  MX_TIM2_Init(); // Таймер на микросекунды
  MX_TIM6_Init(); // Таймер на милисекунды
  MX_TIM7_Init(); // Таймеры на моторы
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM13_Init();

  HAL_TIM_Base_Start_IT(&htim2); // Таймер для общего цикла
  HAL_TIM_Base_Start_IT(&htim6); // Таймер для общего цикла

  HAL_TIM_Base_Start_IT(&htim7);  // Таймер для моторов шаговых для датчиков
  HAL_TIM_Base_Start_IT(&htim10); // Таймер для моторов шаговых для датчиков
  HAL_TIM_Base_Start_IT(&htim11); // Таймер для моторов шаговых для датчиков
  HAL_TIM_Base_Start_IT(&htim13); // Таймер для моторов шаговых для датчиков

  HAL_Delay(4000);
  printf("\r\n *** Modul *** printBIM.ru *** 2025 *** \r\n");
  initFirmware();

  initSPI_slave(); // Закладываем начальноы значения и инициализируем буфер DMA //  // Запуск обмена данными по SPI с использованием DMA


   if ((SCB->CPACR & (0xF << 20)) != (0xF << 20))
    {
        printf("FPU отключена!\n");
    }
    else
    {
        printf("FPU включена!\n");
    }

    uint32_t cpacr = SCB->CPACR; // Чтение регистра CPACR
    if ((cpacr & ((3UL << 20) | (3UL << 22))) == ((3UL << 20) | (3UL << 22)))
    {
        printf("FPU2 включён\n");
    }
    else
    {
        printf("FPU2 отключён\n");
    }

    float result = 0.0f;
    uint32_t start = HAL_GetTick();
    for (int i = 0; i < 10000; i++) {
        result += sinf((float)i / 100.0f);
    }
    uint32_t end = HAL_GetTick();
    uint32_t rez = end - start;
    printf("Time: %lu ms, Result: %f\n", rez, result);
    if (rez > 100)
      printf(" SOFT FPU !!!\n");
      else
      printf(" +++ HARD FPU !!!\n");

    // HAL_Delay(4000);
    
    // while (1);


  // initMotor(); // Начальная инициализация и настройка шаговых моторов
  // // testMotorRun();
  // setZeroMotor(); // Установка в ноль

  // initLaser(); // Инициализация лазеров в зависимости от типа датчкика. определяем переменные буфер приема для каждого UART

  // I2C_ScanDevices(&hi2c1);// Сканирование I2C шины
  icm20948_init(); // Инициализация ICM-20948

  // I2C_ScanDevices(&hi2c1);
  ak09916_init(); // Инициализация магнитометра

  // BNO055_Init(); // Инициализация датчика на шине I2C

  DEBUG_PRINTF("%lli LOOP !!!!!!!!!!!!!!!!!!!!!!!!!!! \r\n", timeSpi);
  HAL_GPIO_WritePin(ledGreen_GPIO_Port, ledGreen_Pin, GPIO_PIN_RESET); // Выключаем светодиод что прошло выполнение кода до цикла

  while (1)
  {
    // workingSPI(); // Отработка действий по обмену по шине SPI
    // workingLaser();  // Отработка действий по лазерным датчикам
    // workingFlag();  // Остановка драйверов и моторов при обрыве связи
    // workingMotor(); // Отработка действий по таймеру в 1, 50, 60 милисекунд
    // workingBNO055(); // Отработка по датчику BNO055

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
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
  RCC_OscInitStruct.PLL.PLLQ = 4;
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