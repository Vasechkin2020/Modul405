#ifndef CODE_H
#define CODE_H

#define MOTOR yes
#define SPI_protocol yes
#define LASER yes

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <limits.h> // для CHAR_BIT


#include "gpio.h" // Добавьте в начало файла, где будете использовать

#include "config.h"
#include "motor.h"
#include "laser80M.h"
#include "sk60plus.h"
#include "slaveSPI.h"
#include "bno055.h"
#include "icm20948.h"
#include "MadgwickAHRS.h"

//********************************* ПЕРЕМЕННЫЕ ***************************************************************************

bool flag_timer_10millisec = false;
bool flag_timer_50millisec = false;
bool flag_timer_1sec = false;

GPIO_TypeDef *myPort;

void timer6();       // Обработчик прерывания таймера TIM6	1 раз в 1 милисекунду
void workingTimer(); // Отработка действий по таймеру в 1, 50, 60 милисекунд
void workingLaser(); // Отработка действий по лазерным датчикам
void workingSPI();   // Отработка действий по обмену по шине SPI
void workingI2C();   // Опрос датчиков на I2C по флагам на прерываниях
void workingFlag();  // Остановка дазеоров и моторов при обрыве связи

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size); // Коллбэк, вызываемый при событии UART Idle по окончания приема
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);                   // Коллбэк, вызываемый при событии UART по окончания приема ОПРЕДЕЛЕННОГО ЗАДАННОГО ЧИСЛА БАЙТ
void initLaser();                                                          // Инициализация лазеров зависимоти от типа датчкика. определяем переменные буфер приема для каждого UART
void initFirmware();                                                       // Заполнение данными Прошивки
uint64_t micros(void);                                                     // Возвращает микросекунды с момента начала работы

float normalizeAngle(float angle) ; // Приведение угла к диапазону -180...+180

struct dataUART dataUART[4];
uint8_t lenDataLaser; // Длинна полученных данных в буфере
HAL_StatusTypeDef status;
HAL_SPI_StateTypeDef statusGetState;

axises icm20948_gyro;  // Данные с гироскопа
axises icm20948_accel; // Данные с акселерометра

bool flagTimeOut = true;            // Флаг таймаута при обрыве связи по SPI
bool flagCollectDataForSPI = false; // Флаг можно собирать данные для отправки
bool flagCallBackUart = false;      // Флаг для указания нужно ли отрабатывать в колбеке  или обраьотка с самой функции

extern float getAngle(int32_t _pulse); // Пересчет импульсов в градусы
extern void setMotorAngle(int num, float _angle);
extern void setZeroMotor();
extern void BNO055_ReadData(); // Разовое считывание данных
extern volatile uint32_t millisCounter;
extern volatile uint32_t overflow_count; // Счётчик переполнений

// int laser_pred = 0;            // Переменная для запоминания предыдущей команды
u_int8_t modeControlMotor = 0; // Режим в котором находиттся мотор после последней команды управления
u_int8_t modeControlLaser = 0; // Режим в котором находиттся лазер после последней команды управления


// Время предсказания в секундах.  Начни с 0.01 (10 мс - один цикл SPI). Если мотор все равно чуть отстает, увеличь до 0.015 или 0.02.
#define PREDICTION_TIME 0.01f

// typedef struct SDataLaser
// {
//     uint32_t distance;
//     uint16_t signalQuality;
// } SDataLaser;

// SDataLaser dataLaser[4]; // Структура куда пишем даные из датчиков

//********************************* ФУНКЦИИ ***************************************************************************

// Функция для включения FPU (Floating Point Unit) на STM32F4xx
void EnableFPU(void)
{
    // Включение FPU (CP10 и CP11: полный доступ)
    SCB->CPACR |= ((3UL << 20) | (3UL << 22)); // CP10 = 0b11, CP11 = 0b11

    if ((SCB->CPACR & (0xF << 20)) != (0xF << 20))
        printf("FPU отключена!\n");
    else
        printf("FPU включена!\n");

    uint32_t cpacr = SCB->CPACR; // Чтение регистра CPACR
    if ((cpacr & ((3UL << 20) | (3UL << 22))) == ((3UL << 20) | (3UL << 22)))
        printf("FPU2 включён\n");
    else
        printf("FPU2 отключён\n");

    float result = 0.0f;
    uint32_t start = HAL_GetTick();
    for (int i = 0; i < 10000; i++)
    {
        result += sinf((float)i / 100.0f);
        result += tanf((float)i / 100.0f);
    }
    uint32_t end = HAL_GetTick();
    uint32_t rez = end - start;
    printf("Time: %lu ms, Result: %f\n", rez, result);
    if (rez > 100)
        printf(" SOFT FPU !!!\n");
    else
        printf(" +++ HARD FPU !!!\n");
}

// Вот функция на C, которая печатает целое число в бинарном формате с использованием printf:
void print_binary(int num)
{
    // Определяем количество бит в числе int
    int bits = sizeof(num) * CHAR_BIT;

    // Маска для проверки каждого бита (начинаем со старшего бита)
    unsigned mask = 1 << (bits - 1);

    // Пропускаем ведущие нули для более компактного вывода
    int leading_zero = 1;

    printf("Binary representation of %d: ", num);

    for (int i = 0; i < bits; i++)
    {
        if (num & mask)
        {
            putchar('1');
            leading_zero = 0;
        }
        else if (!leading_zero || i == bits - 1)
        {
            // Печатаем нули, если уже были единицы или это последний бит
            putchar('0');
        }
        mask >>= 1;
    }
    putchar('\n');
}

// Функция для возврата количества миллисекунд
uint32_t millis()
{
    return millisCounter;
}

void timer6() // Обработчик прерывания таймера TIM6	1 раз в 1 милисекунду
{
    static int count_timer_10millisec = 0; // Счетчик для запуска обработки движения моторов в лупе по флагу
    static int count_timer_50millisec = 0; // Счетчик для запуска каждые 50 милисекунд
    static int count_timer_1sec = 0;       // Счетчик для запуска

    count_timer_10millisec++;
    count_timer_50millisec++;
    count_timer_1sec++;

    millisCounter++; // Увеличиваем счетчик миллисекунд

    //  каждые 10 милисекунд
    if (count_timer_10millisec >= 10)
    {
        count_timer_10millisec = 0;
        // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);
        flag_timer_10millisec = true;
    }
    // каждые 50 милисекунд
    if (count_timer_50millisec >= 50)
    {
        count_timer_50millisec = 0;
        flag_timer_50millisec = true;
    }
    // 1 seconds
    if (count_timer_1sec >= 1000)
    {
        count_timer_1sec = 0;
        flag_timer_1sec = true;
    }
}

// Возвращает микросекунды с момента начала работы
uint64_t micros(void)
{
    uint32_t current_count = __HAL_TIM_GET_COUNTER(&htim2); // Текущее значение счётчика
    uint32_t overflows = overflow_count;                    // Текущее значение переполнений

    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2) && current_count > 0x80000000) // Проверяем, не произошло ли переполнение во время чтения
    {
        overflows--; // Если таймер переполнился после чтения current_count
    }
    uint64_t ret = ((uint64_t)overflows * 4294967296ULL) + current_count; // Вычисляем общее время в микросекундах

    // uint64_t ret2 = 1234567890123ULL;
    // printf("Elapsed Time: %llu microseconds\n", ret);
    // printf("Elapsed Time: %" PRIu64 " microseconds\n", ret); // Корректный вывод с использованием PRIu64

    // Разделяем на верхние и нижние 32 бита
    // uint32_t high = (uint32_t)(ret >> 32); // Верхние 32 бита
    // uint32_t low = (uint32_t)(ret & 0xFFFFFFFF); // Нижние 32 бита

    // DEBUG_PRINTF("Time from Start: %lu%09lu microseconds | current_count = %lu microseconds | overflows %lu \r\n", high, low, current_count, overflows);
    return ret;
}

void workingTimer() // Отработка действий по таймеру в 1, 50, 60 милисекунд
{
    // HAL_Delay(); // Пауза 500 миллисекунд.
    //----------------------------- 10 миллисекунд --------------------------------------
    if (flag_timer_10millisec)
    {
        flag_timer_10millisec = false;
        // HAL_GPIO_TogglePin(Led1_GPIO_Port, Led1_Pin);             // Инвертирование состояния выхода.
        // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10); // Инвертирование состояния выхода.
        // BNO055_ReadData(); // Разовое считывание данных
    }
    //----------------------------- 50 миллисекунд --------------------------------------
    if (flag_timer_50millisec)
    {
        flag_timer_50millisec = false;
        // disableMotor(); // Отключаем моторы если они не используются чтобы не грелись
        // flag_readBNO055 = true; // Флаг что можно читать данные с BNO055
        // flag_sendRequestBNO055 = true; // Взводим флаг что можно снова запрос к BNO055

        // DEBUG_PRINTF("50msec %li \r\n", millis());
        //  flag_data = true; // Есть новые данные по шине // РУчной вариант имитации пришедших данных с частотой 20Гц
        // static uint64_t current_time = 0;
        // static uint64_t now_time = 0;
        // // DEBUG_PRINTF("micros %u microseconds\n", micros());
        // // uint64_t elapsed_time = micros() - current_time; // Прошедшее время
        // now_time = micros();
        // uint32_t aaaa = now_time - current_time;
        // current_time = now_time; // Текущее время в микросекундах
        // printf("Elapsed Time: %lu microseconds\n", aaaa);
        // DEBUG_PRINTF("%lu \r\n", micros());
    }

    //----------------------------- 1 секунда --------------------------------------
    if (flag_timer_1sec) // Вызывается каждую секунду
    {
        flag_timer_1sec = false;
        printf("%li \r\n", millis());

        // statusGetState = HAL_SPI_GetState(&hspi1);
        // if (statusGetState == HAL_SPI_STATE_READY)
        // {
        //     DEBUG_PRINTF("Timer SPI. ПЕРЕЗАПУСК ПО ТАЙМЕРУ. ЭТО ОШИБКА\n");
        //     HAL_SPI_Abort(&hspi1);
        //     HAL_SPI_DMAStop(&hspi1);
        //     HAL_SPI_TransmitReceive_DMA(&hspi1, txBuffer, rxBuffer, BUFFER_SIZE); // // Перезапуск функции для следующего обмена// Запуск обмена данными по SPI с использованием DMA
        // }
        // else
        // {
        //     // DEBUG_PRINTF("Timer HAL_SPI_STATE_BUSY_TX_RX %u \n", statusGetState);
        // }
        // HAL_GPIO_TogglePin(Led1_GPIO_Port, Led1_Pin); // Инвертирование состояния выхода.
        //  uint8_t UART1_rxBuffer[4] = {0xAA,0xFF,0xAA,0xFF};
        //   uint8_t UART1_rxBuffer[1] = {0x56}; //Запрос версии "V"
        //   uint8_t UART1_rxBuffer[1] = {0x4F}; // Включить лазер "O"
        //   uint8_t UART1_rxBuffer[1] = {0x43}; // Выключить лазер "C"
    }
}

// Коллбэк, вызываемый при событии UART Idle по окончания приема
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2)
    {
        // DEBUG_PRINTF("USART2\n");
        dataUART[1].flag = 1;                                                                      // Обработка полученных данных
        dataUART[1].len = Size;                                                                    // Обработка полученных данных
        status = HAL_UARTEx_ReceiveToIdle_DMA(dataUART[1].huart, dataUART[1].adr, RX_BUFFER_SIZE); // После обработки вновь запустить прием
        dataUART[1].statusDMA = status;
    }
    else if (huart->Instance == UART4)
    {
        // DEBUG_PRINTF("USART4\n");
        dataUART[0].flag = 1;                                                                      // Обработка полученных данных
        dataUART[0].len = Size;                                                                    // Обработка полученных данных
        status = HAL_UARTEx_ReceiveToIdle_DMA(dataUART[0].huart, dataUART[0].adr, RX_BUFFER_SIZE); // После обработки вновь запустить прием
        dataUART[0].statusDMA = status;
    }
    else if (huart->Instance == UART5)
    {
        // DEBUG_PRINTF("USART5\n");
        dataUART[2].flag = 1;                                                                      // Обработка полученных данных
        dataUART[2].len = Size;                                                                    // Обработка полученных данных
        status = HAL_UARTEx_ReceiveToIdle_DMA(dataUART[2].huart, dataUART[2].adr, RX_BUFFER_SIZE); // После обработки вновь запустить прием
        dataUART[2].statusDMA = status;
    }
    else if (huart->Instance == USART6)
    {
        // DEBUG_PRINTF("USART6\n");
        dataUART[3].flag = 1;                                                                      // Обработка полученных данных
        dataUART[3].len = Size;                                                                    // Обработка полученных данных
        status = HAL_UARTEx_ReceiveToIdle_DMA(dataUART[3].huart, dataUART[3].adr, RX_BUFFER_SIZE); // После обработки вновь запустить прием
        dataUART[3].statusDMA = status;
    }
}

// Собираем нужные данные и пишем в структуру на отправку
void collect_Data_for_Send()
{
    Modul2Data_send.id++;
    Modul2Data_send.pinMotorEn = HAL_GPIO_ReadPin(En_Motor_GPIO_Port, En_Motor_Pin); // Считываем состояние пина драйверов

    for (int i = 0; i < 4; i++) // Информация по моторам всегда
    {
        Modul2Data_send.motor[i].status = motor[i].status;                               // Считываем состояние пина драйверов
        Modul2Data_send.motor[i].position = getAngle(motor[i].position);                 // Записываем текущую позицию преобразуя из импульсов в градусы, надо еще в глобальную систему преобразовывать
        Modul2Data_send.motor[i].destination = Data2Modul_receive.controlMotor.angle[i]; //
        // Modul2Data_send.motor[i].destination = getAngle(motor[i].destination);        // Считываем цель по позиции, надо еще в глобальную систему преобразовывать

        Modul2Data_send.micric[i] = HAL_GPIO_ReadPin(motor[i].micric_port, motor[i].micric_pin); //
    }

    for (int i = 0; i < 4; i++) // Информация по лазерам по ситуации
    {
        if (Data2Modul_receive.controlLaser.mode != 0) // Если команда работать с датчиком
        {
            Modul2Data_send.laser[i].status = dataUART[i].status;                              // Считываем статаус дальномера
            Modul2Data_send.laser[i].distance = (float)dataUART[i].distance * 0.001;           // Считываем измерение растояния и пересчитываем в метры !!!
            Modul2Data_send.laser[i].signalQuality = dataUART[i].quality;                      // Считываем качество сигнала измерение
            Modul2Data_send.laser[i].angle = (float)dataUART[i].angle;                         // Считываем угол в котором произвели измерение
            Modul2Data_send.laser[i].time = dataUART[i].time;                                  // Считываем время в котором произвели измерение
            Modul2Data_send.laser[i].numPillar = Data2Modul_receive.controlMotor.numPillar[i]; // Переписываем номер столба на который измеряли расстояние
            Modul2Data_send.laser[i].rate = dataUART[i].rate;
        }
        else
        {
            Modul2Data_send.laser[i].status = 888; // Статус не работаем с датчиком
            Modul2Data_send.laser[i].distance = 0;
            Modul2Data_send.laser[i].signalQuality = 0;
            Modul2Data_send.laser[i].angle = 0;
            Modul2Data_send.laser[i].time = 0;
            Modul2Data_send.laser[i].numPillar = -1; // Номер не существующего столба
            Modul2Data_send.laser[i].rate = 0;       //
        }
    }

    Modul2Data_send.bno055 = bno055;     // Записываем данные с BNO055
    Modul2Data_send.icm20948 = icm20948; // Записываем данные с ICM20948
    Modul2Data_send.spi = spi;           // Записываем данные по SPI

    uint32_t cheksum_send = 0;                                          // Считаем контрольную сумму отправляемой структуры
    unsigned char *adr_structura = (unsigned char *)(&Modul2Data_send); // Запоминаем адрес начала структуры. Используем для побайтной передачи
    for (int i = 0; i < sizeof(Modul2Data_send) - 4; i++)
    {
        cheksum_send += adr_structura[i]; // Побайтно складываем все байты структуры кроме последних 4 в которых переменная в которую запишем результат
    }

    Modul2Data_send.cheksum = cheksum_send; // Записываем контрольную сумму в структуру
    
    __disable_irq();             // 1. Выключаем прерывания (тишина!)
    DataForSPI = Modul2Data_send;           // Копируем в специальную переменную. Копируем данные (теперь никто не помешает)
    __enable_irq();              // 3. Включаем прерывания обратно
}

// Отработка пришедших команд. Изменение скорости, траектории и прочее
void executeDataReceive()
{
    uint64_t timeNow = micros();
    static uint64_t predTime = 0;
    float dt = (timeNow - predTime) / 1000000.0; // Находим разницу по времени от прыдыдущего расчета
    predTime = timeNow;
    if (dt <= 0.005)
        dt = 0.005; // Защита от деления на ноль
    // DEBUG_PRINTF("dt SPI = %f ", dt);

    // DEBUG_PRINTF("executeDataReceive... motor= %u laser= %u ", modeControlMotor, modeControlLaser);
    // DEBUG_PRINTF("in... motor= %lu laser= %lu \r\n", Data2Modul_receive.controlMotor.mode, Data2Modul_receive.controlLaser.mode);
    // Команда УПРАВЛЕНИЯ УГЛАМИ
    if (Data2Modul_receive.controlMotor.mode == 0) // Если пришла команда 0 Управления
    {
        modeControlMotor = 0; // Запоминаем в каком режиме Motor
        // Ничего не делаем
    }
    if (Data2Modul_receive.controlMotor.mode == 1) // Если пришла команда 1 Управления
    {
        modeControlMotor = 1; // Запоминаем в каком режиме Motor
        DEBUG_PRINTF("executeDataReceive mode= %lu status = %i %i %i %i \r\n", Data2Modul_receive.controlMotor.mode, motor[0].status, motor[1].status, motor[2].status, motor[3].status);
        for (int i = 0; i < 4; i++)
        {
            // 1. Получаем "сырой" угол от Мастера
            float targetAngle = Data2Modul_receive.controlMotor.angle[i]; 

            // 2. Сначала считаем скорость (Feed-Forward) и фильтруем её
            // Функция calcAngleSpeedMotor вернет 0, если это резкий скачок (смена столба), или реальную скорость, если это плавное слежение.
            motor[i].angleSpeed = calcAngleSpeedMotor(i, targetAngle, dt); // Расчет скорости для мотора в градусах в секунду

            // 3. ПРЕДСКАЗАНИЕ (ЭКСТРАПОЛЯЦИЯ)
            // Если скорость не 0 (значит мы в режиме слежения), добавляем прогноз
            float predictedAngle = targetAngle;
            
            if (motor[i].angleSpeed != 0.0f) 
            {
                // Угол = Текущий + (Скорость * Время_Предсказания)
                predictedAngle = targetAngle + (motor[i].angleSpeed * PREDICTION_TIME);
                
                // DEBUG: Можно посмотреть, насколько предсказание отличается от реальности
                DEBUG_PRINTF("Target: %.2f | Pred: %.2f \n", targetAngle, predictedAngle);
            }

            // 4. Ставим  мотору СКОРРЕКТИРОВАННЫЙ угол
            setMotorAngle(i, predictedAngle); // Устанавливаю motor[num].destination Функция setMotorAngle сама ограничит угол диапазоном 1..179, если прогноз вылетел за пределы
            
            // 5. Считаем итоговую скорость для мотора (P-регулятор + FeedForward)
            float speed = calcSpeedMotor(i, dt); // Расчет скорости для мотора  Внутри calcSpeedMotor используется motor[i].destination, который мы только что обновили (predictedAngle)
            
            // 6. Применяем скорость
            setMotorSpeed(i, speed);                                                   // Установка скорости
            // DEBUG_PRINTF("status = %i \r\n", motor[i].status);
        }
    }
    
    // Команда КОЛИБРОВКИ И УСТАНОВКИ В 0
    if (Data2Modul_receive.controlMotor.mode == 9 && modeControlMotor != 9) // Если пришла команда 9 Колибровки и предыдущая была другая
    {
        DEBUG_PRINTF("Start colibrovka \n");
        modeControlMotor = 9;  // Запоминаем в каком режиме Motor
        timerMode9 = millis(); // Запоминаем время начал
        flagMode9 = true;      // что мы начали режим колибровки
        rotationRight();       // Отводим мотор на 10 градусов
    }
    // Команда ВКЛЮЧЕНИЯ ЛАЗЕРНЫХ ДАТЧИКОВ
    if (Data2Modul_receive.controlLaser.mode == 1 && modeControlLaser != 1) // Если пришла команда и предыдущая была другая
    {
        modeControlLaser = 1; // Запоминаем в каком режиме Лазер
#ifdef LASER80
        // Непрерывное измерение
        laser80_continuousMeasurement(0); // Данные пойдут только через 500 милисекунд
        laser80_continuousMeasurement(1); // Данные пойдут только через 500 милисекунд
        laser80_continuousMeasurement(2); // Данные пойдут только через 500 милисекунд
        laser80_continuousMeasurement(3); // Данные пойдут только через 500 милисекунд
#endif
#ifdef LASER60
        sk60plus_startContinuousSlow(0);
        sk60plus_startContinuousSlow(1);
        sk60plus_startContinuousSlow(2);
        sk60plus_startContinuousSlow(3);
#endif
    }

    if (Data2Modul_receive.controlLaser.mode == 2 && modeControlLaser != 2) // Если пришла команда и находимся не в этом режиме
    {
        modeControlLaser = 2; // Запоминаем в каком режиме Лазер
#ifdef LASER80
        // Непрерывное измерение тут нет варианта выбора скорости, поэтому одинаково с командой 1
        laser80_continuousMeasurement(0); // Данные пойдут только через 500 милисекунд
        laser80_continuousMeasurement(1); // Данные пойдут только через 500 милисекунд
        laser80_continuousMeasurement(2); // Данные пойдут только через 500 милисекунд
        laser80_continuousMeasurement(3); // Данные пойдут только через 500 милисекунд
#endif
#ifdef LASER60
        sk60plus_startContinuousAuto(0);
        sk60plus_startContinuousAuto(1);
        sk60plus_startContinuousAuto(2);
        sk60plus_startContinuousAuto(3);
#endif
    }

    // Команда ВЫЛЮЧЕНИЯ ЛАЗЕРНЫХ ДАТЧИКОВ
    if (Data2Modul_receive.controlLaser.mode == 0 && modeControlLaser != 0) // Если пришла команда и предыдущая была другая
    {
        modeControlLaser = 0; // Запоминаем в каком режиме Лазер
#ifdef LASER80
        laser80_stopMeasurement(0);
        laser80_stopMeasurement(1);
        laser80_stopMeasurement(2);
        laser80_stopMeasurement(3);
#endif
#ifdef LASER60
        sk60plus_stopContinuous(0);
        sk60plus_stopContinuous(1);
        sk60plus_stopContinuous(2);
        sk60plus_stopContinuous(3);
#endif
    }
}

void initLaser() // Инициализация лазеров в зависимоти от типа датчика. определяем переменные и буфер приема для каждого UART
{
    printf("laserInit... \r\n");
    // Это общие данные для любых датчиков
    dataUART[0].num = 0;
    dataUART[0].adr = rx_bufferLaser0;
    dataUART[0].huart = &huart4;

    dataUART[1].num = 1;
    dataUART[1].adr = rx_bufferLaser1;
    dataUART[1].huart = &huart2;

    dataUART[2].num = 2;
    dataUART[2].adr = rx_bufferLaser2;
    dataUART[2].huart = &huart5;

    dataUART[3].num = 3;
    dataUART[3].adr = rx_bufferLaser3;
    dataUART[3].huart = &huart6;

    for (int i = 0; i < 4; i++)
    {
        HAL_UART_DMAStop(dataUART[i].huart);                                              // Остановка DMA
        HAL_UARTEx_ReceiveToIdle_DMA(dataUART[i].huart, dataUART[i].adr, RX_BUFFER_SIZE); // Двнные оказываются в буфере rx_bufferLaser
    }

#ifdef LASER80
    lenDataLaser = 11;

    printf("\r\n");
    HAL_Delay(100);
    laser80_stopMeasurement(0);
    laser80_stopMeasurement(1);
    laser80_stopMeasurement(2);
    laser80_stopMeasurement(3);

    for (int i = 0; i < 4; i++)
    {
        printf("\r\n");
        HAL_Delay(100);
        laser80_controlLaser(i, 1);
        HAL_Delay(100);
        laser80_setTimeInterval(i, 0);
        HAL_Delay(100);
        laser80_setResolution(i, 1);
        HAL_Delay(100);
        laser80_setRange(i, 30);
        HAL_Delay(100);
        laser80_setStartingPoint(i, 1);
        HAL_Delay(100);
        laser80_setFrequency(i, 10);
        HAL_Delay(100);
        laser80_controlLaser(i, 0);
        HAL_Delay(100);
        laser80_singleMeasurement(i);
    }

    // Непрерывное измерение
    // HAL_Delay(5000);
    // laser80_stopMeasurement(huart1,0x80);
    // laser80_continuousMeasurement(0); // Данные пойдут только через 500 милисекунд
    // laser80_continuousMeasurement(1); // Данные пойдут только через 500 милисекунд
    // laser80_continuousMeasurement(2); // Данные пойдут только через 500 милисекунд
    // laser80_continuousMeasurement(3); // Данные пойдут только через 500 милисекунд
#endif

#ifdef LASER60

    lenDataLaser = 13;

    sk60plus_autoBaund();

    sk60plus_setLaser(0, 1);
    sk60plus_readSerialNumber(0);
    sk60plus_readSoftwareVersion(0);
    sk60plus_readHardwareVersion(0);
    sk60plus_readInputVoltage(0);
    sk60plus_setLaser(0, 0);
    sk60plus_startSingleAuto(0);
    printf("---\r\n");

    sk60plus_setLaser(1, 1);
    sk60plus_readSerialNumber(1);
    sk60plus_readSoftwareVersion(1);
    sk60plus_readHardwareVersion(1);
    sk60plus_readInputVoltage(1);
    sk60plus_setLaser(1, 0);
    sk60plus_startSingleAuto(1);
    printf("---\r\n");

    sk60plus_setLaser(2, 1);
    sk60plus_readSerialNumber(2);
    sk60plus_readSoftwareVersion(2);
    sk60plus_readHardwareVersion(2);
    sk60plus_readInputVoltage(2);
    sk60plus_setLaser(2, 0);
    sk60plus_startSingleAuto(2);
    printf("---\r\n");

    sk60plus_setLaser(3, 1);
    sk60plus_readSerialNumber(3);
    sk60plus_readSoftwareVersion(2);
    sk60plus_readHardwareVersion(3);
    sk60plus_readInputVoltage(3);
    sk60plus_setLaser(3, 0);
    sk60plus_startSingleAuto(3);
    printf("---\r\n");

    // sk60plus_startContinuousAuto(0);
    // sk60plus_startContinuousAuto(1);
    // sk60plus_startContinuousAuto(2);
    // sk60plus_startContinuousAuto(3);

#endif
}
// Отработка действий по лазерным датчикам
void workingLaser()
{
    for (int i = 0; i < 4; i++)
    {
        if (dataUART[i].flag == 1)
        {
            dataUART[i].flag = 0;
#ifdef LASER80
            if (dataUART[i].adr[0] == 0x80 && dataUART[i].adr[1] == 0x06) // Если ответ без ошибки то
            {
                dataUART[i].status = 0; // Статус все хорошо
                dataUART[i].distance = laser80_calcDistance(dataUART[i].adr, lenDataLaser);
                // DEBUG_PRINTF("D %i = %lu \n", i, dataUART[i].distance);
                dataUART[i].quality = 0;
                dataUART[i].angle = getAngle(motor[i].position);
                dataUART[i].rate = (float)1000.0 / (millis() - dataUART[i].time);
                // DEBUG_PRINTF(" UART%i rate = %f time = %lu \r\n", dataUART[i].num, dataUART[i].rate, dataUART[i].time);
                dataUART[i].time = millis();
                // DEBUG_PRINTF(" UART%i dist = %lu qual = %u \r\n", dataUART[i].num, dataUART[i].distance, dataUART[i].quality);
            }
            else
            {
                dataUART[i].status = 999; // Статус ошибка
                dataUART[i].distance = 0;
                dataUART[i].quality = 0;
                dataUART[i].angle = 0;
                dataUART[i].time = 0;
                dataUART[i].rate = 0;
                // DEBUG_PRINTF("%li UART%i statusDMA= %i   /   ", millis(), dataUART[i].num, dataUART[i].statusDMA);
                // for (int j = 0; j < lenDataLaser; j++)
                // {
                //     DEBUG_PRINTF("%x ", dataUART[i].adr[j]);
                // }
                // DEBUG_PRINTF(" Error dataUART%i. \r\n", dataUART[i].num);
                // do
                // {
                //     HAL_UART_DMAStop(dataUART[i].huart);
                //     memset(dataUART[i].adr, 0, RX_BUFFER_SIZE);                                      // Очистка буфера
                //     status = HAL_UART_Receive_DMA(dataUART[i].huart, dataUART[i].adr, lenDataLaser); // Запускаем ожидание ответа, указываем куда и сколько байт мы ждем.
                //     // DEBUG_PRINTF("New status0 = %i ", status);
                //     HAL_Delay(1);

                // } while (status != 0);
                // DEBUG_PRINTF("New statusDMA = %i\r\n", status);
            }

#endif

#ifdef LASER60
            if (dataUART[i].adr[0] == 0xAA) // Если ответ без ошибки то
            {
                dataUART[i].status = 0; // Статус все хорошо
                dataUART[i].distance = laser60_calcDistance(dataUART[i].adr);
                dataUART[i].quality = laser60_calcSignalQuality(dataUART[i].adr);
                dataUART[i].angle = getAngle(motor[i].position);
                dataUART[i].rate = 1000 / (millis() - dataUART[i].time);
                dataUART[i].time = millis();
                // DEBUG_PRINTF(" UART%i dist = %lu qual = %u \r\n", dataUART[i].num, dataUART[i].distance, dataUART[i].quality);
            }
            else
            {
                dataUART[i].status = 999; // Статус ошибка
                dataUART[i].distance = 0;
                dataUART[i].quality = 0;
                dataUART[i].angle = 0;
                dataUART[i].time = 0;
                dataUART[i].rate = 0;
                // DEBUG_PRINTF("%li UART%i statusDMA= %i   /   ", millis(), dataUART[i].num, dataUART[i].statusDMA);
                // for (int j = 0; j < lenDataLaser; j++)
                // {
                //     DEBUG_PRINTF("%x ", dataUART[i].adr[j]);
                // }
                // DEBUG_PRINTF(" Error dataUART%i. \r\n", dataUART[i].num);
            }
#endif
        }
    }
}
// Отработка действий по обмену по шине SPI
void workingSPI()
{
    //----------------------------- По факту обмена данными с верхним уровнем --------------------------------------
#ifdef SPI_protocol
    if (flag_data) // Если обменялись данными
    {
        // DEBUG_PRINTF("    in %lu\n", millis());
        // HAL_GPIO_WritePin(Analiz2_GPIO_Port, Analiz2_Pin, GPIO_PIN_SET); // Инвертирование состояния выхода.
        flag_data = false;
        flagTimeOut = true;           // Флаг для выключения по таймауту
        flagCollectDataForSPI = true; // Флаг для сбора данных для следующего обмена
        flag_readBNO055 = true;

        timeSpi = millis(); // Запоминаем время обмена
        // DEBUG_PRINTF("In = %#x %#x %#x %#x | ", rxBuffer[0], rxBuffer[1], rxBuffer[2], rxBuffer[3]);
        // DEBUG_PRINTF("Out = %#x %#x %#x %#x \r\n", txBuffer[0], txBuffer[1], txBuffer[2], txBuffer[3]);
        // DEBUG_PRINTF("+\n");
        processingDataReceive(); // Обработка пришедших данных после состоявшегося обмена  !!! Подумать почему меняю данные даже если они с ошибкой, потом по факту когда будет все работать
        // DEBUG_PRINTF(" mode= %i \n",Data2Modul_receive.controlMotor.mode);
        executeDataReceive(); // Выполнение пришедших команд

        // DEBUG_PRINTF(" Receive id= %i cheksum= %i command= %i ", Data2Modul_receive.id, Data2Modul_receive.cheksum,Data2Modul_receive.command );
        // DEBUG_PRINTF("start = ");
        // for (int i = 0; i < sizeof(txBuffer); i++)
        // {
        //     DEBUG_PRINTF(" %x", txBuffer[i]);
        // }
        // DEBUG_PRINTF("\n");
        // collect_Data_for_Send(); // Собираем данные в структуре для отправки на момент прихода команлы, но БЕЗ учета команды.До исполнения команды.

        // DEBUG_PRINTF(" angle0= %.2f angle1= %.2f angle2= %.2f angle3= %.2f", Data2Modul_receive.angle[0], Data2Modul_receive.angle[1], Data2Modul_receive.angle[2], Data2Modul_receive.angle[3] );

        // spi_slave_queue_Send();  // Закладываем данные в буфер для передачи(обмена)

        // DEBUG_PRINTF("end   = ");
        // for (int i = 0; i < sizeof(txBuffer); i++)
        // {
        //     DEBUG_PRINTF(" %x", txBuffer[i]);
        // }
        // DEBUG_PRINTF("-----\n");
        // HAL_GPIO_WritePin(Analiz2_GPIO_Port, Analiz2_Pin, GPIO_PIN_RESET); // Инвертирование состояния выхода.
    }
#endif
}

// Опрос датчиков на I2C по флагам на прерываниях
void workingI2C()
{
    uint8_t static bufferBNO055[38] = {0}; // было 20 без гиро и акселя

    if (flag_readBNO055) // Если взведен флаг после обмена по SPI что можно теперь работать по I2C с BNO055
    {
        if (flag_sendRequestBNO055) // Если взведен флаг что нужно отправить запрос к BNO055
        {
            flag_sendRequestBNO055 = false; // Сбрасываем флаг отправки запроса к BNO055
            // DEBUG_PRINTF("    BNO055_Transmit_IT %lu \n", millis());
            // DEBUG_PRINTF("   + %lu\n", millis());
            BNO055_Transmit_IT(eBNO055_REGISTER_ACC_DATA_X_LSB); // Отправка запроса к датчику.Указываем с какого регистра будем читать
        }
        if (i2cTransferComplete) // Запрос на считывание заданного числа байт с датчика в буфер
        {
            i2cTransferComplete = 0;
            BNO055_Receive_IT(bufferBNO055, 38); // считываем 38 байт из BNO055
            // DEBUG_PRINTF("    BNO055_Receive_IT %lu \n", millis());
        }
        if (i2cReceiveComplete) // Обработка буфера после считывания данных по шине
        {
            i2cReceiveComplete = 0;
            calcBufferBNO(bufferBNO055);
            // DEBUG_PRINTF("    calcBuffer BNO %lu\n", millis());
            // BNO055_StatusCalibr();

            flag_readBNO055 = false;
            flag_sendRequestBNO055 = true; // Взводим флаг что можно снова запрос к BNO055
            flag_readICM20948 = true;      // Взводим флаг что можно работать дальше с ICM20948
        }
    }

#ifdef ICM20948
    // uint8_t static bufferICM20948[12] = {0}; // буфер для ICM20948
    if (flag_readICM20948)                   // Если взведен флаг что можно работать с ICM20948
    {
        // if (flag_sendRequestICM20948) // Если взведен флаг что нужно отправить запрос к ICM20948
        // {
        //     flag_sendRequestICM20948 = false;      // Сбрасываем флаг отправки запроса к ICM20948
        //     ICM20948_Transmit_IT(B0_ACCEL_XOUT_H); // Отправка запроса к датчику.Указываем с какого регистра будем читать
        //     // DEBUG_PRINTF("    ICM20948_Transmit_IT %lu \n", millis());
        // }
        // if (i2cTransferComplete) // Запрос на считывание заданного числа байт с датчика в буфер
        // {
        //     i2cTransferComplete = 0;
        //     ICM20948_Receive_IT(bufferICM20948, 12); // посылаем запрос на чтение 12 байт из ICM20948
        //     // DEBUG_PRINTF("    ICM20948_Receive_IT %lu \n", millis());
        // }
        // if (i2cReceiveComplete) // Обработка буфера после считывания данных по шине
        // {
        //     i2cReceiveComplete = 0;
        //     calcBufferICM(bufferICM20948, &icm20948_accel, &icm20948_gyro); // Обработка буфера после считывания данных по шине
    
        // !!!!!!!!!!!!!!!!!! ЗАЧЕМ Я ТУТ ДВАЖДЫ СЧИТЫВАЮ? СНАЧАЛА ЧЕРЕЗ ПРЕРЫВАНИЕ А ПОТОМ ПРОСТО ПО ЗАПРОСУ????
        // HAL_Delay(1);
        icm20948_gyro_read_dps(&icm20948_gyro); // Преобразуем, фильтруем данные гироскопа
        icm20948_accel_read_g(&icm20948_accel); // Преобразуем, фильтруем данные акселерометра

        float roll_A = 0.0f, pitch_A = 0.0f; // Углы считаем из Акселерометра только roll pitch. yaw не может быть посчитан

        roll_A = atan2f(icm20948_accel.y, sqrtf(icm20948_accel.x * icm20948_accel.x + icm20948_accel.z * icm20948_accel.z));   // Вычисление крена (Roll) //Такой расчет подходит только для статичного положения. В движении аксель будет врать
        pitch_A = atan2f(-icm20948_accel.x, sqrtf(icm20948_accel.y * icm20948_accel.y + icm20948_accel.z * icm20948_accel.z)); // Вычисление тангажа (Pitch) //Такой расчет подходит только для статичного положения. В движении аксель будет врать

        roll_A = roll_A * 180.0f / M_PI; // Перевод в градусы
        pitch_A = pitch_A * 180.0f / M_PI;

        // DEBUG_PRINTF("icm20948_accel %+8.3f %+8.3f %+8.3f | ", icm20948_accel.x, icm20948_accel.y, icm20948_accel.z);
        // DEBUG_PRINTF("bno055.accel %+8.3f %+8.3f %+8.3f \n| ", bno055.accel.x, bno055.accel.y, bno055.accel.z);

        MadgwickAHRSupdateIMU(icm20948_gyro.x, icm20948_gyro.y, icm20948_gyro.z, icm20948_accel.x, icm20948_accel.y, icm20948_accel.z); // Обновление фильтра Madgwick

        icm20948.status = 0;
        // Статус все хорошо
        icm20948.gyro.x = icm20948_gyro.x; // Записываем гироскоп в структуру для отправки
        icm20948.gyro.y = icm20948_gyro.y;
        icm20948.gyro.z = icm20948_gyro.z;

        icm20948.accel.x = icm20948_accel.x; // Записываем акселерометр в структуру для отправки
        icm20948.accel.y = icm20948_accel.y;
        icm20948.accel.z = icm20948_accel.z;

        icm20948.angleEuler.x = Madgw.roll; // Записываем углы Эйлера в структуру для отправки
        icm20948.angleEuler.y = Madgw.pitch;
        icm20948.angleEuler.z = Madgw.yaw;

        icm20948.linear.x = Madgw.linAcc.x; // Записываем линейное ускорение в структуру для отправки
        icm20948.linear.y = Madgw.linAcc.y;
        icm20948.linear.z = Madgw.linAcc.z;

        static uint32_t timeICM20948 = 0; // Время для подсчета частоты опроса
        if (timeICM20948 == 0)
            timeICM20948 = millis(); // Инициализация первого раза

        icm20948.rate = (float)1000.0 / (millis() - timeICM20948); // Считаем частоту
        timeICM20948 = millis();                                   // Запоминаем время

        // DEBUG_PRINTF("   - %lu\n", millis());
        // DEBUG_PRINTF("    calcBuffer ICM %lu\n", millis());

        flag_readICM20948 = false;
        flag_sendRequestICM20948 = true; // Взводим флаг что можно снова запрос к ICM20948

        DEBUG_PRINTF("BNO %+8.3f %+8.3f %+8.3f |", bno055.angleEuler.x, bno055.angleEuler.y, bno055.angleEuler.z);
        // DEBUG_PRINTF("ICM20948.Accel %+8.3f %+8.3f |", roll_A, pitch_A);
        DEBUG_PRINTF("Madgwick %+8.3f %+8.3f %+8.3f || \n ", Madgw.roll, Madgw.pitch, Madgw.yaw);
        // }
    }
#endif
}

// Остановка лазеров и моторов при обрыве связи
void workingFlag()
{
    if (flagCollectDataForSPI) // Сбор данных для обмена по флагу и таймеру
    {
        if (millis() - timeSpi > 6) // Если прошло больше 7 м секунд с момента обмена по  SPI
        {
            flagCollectDataForSPI = false;
            // DEBUG_PRINTF("    flagCollectData %lu\n", millis());
            collect_Data_for_Send(); // Собираем данные в структуре для отправки на момент прихода команлы, но БЕЗ учета команды.До исполнения команды.
        }
    }

    if (flagTimeOut) // Остановка при обрыве связи
    {
        if (millis() - timeSpi > 15000) // Если обмена нет больше 5 секунд то отключаем все
        {
            flagTimeOut = false;
            printf("    workingStopTimeOut ! \n");
            HAL_GPIO_WritePin(En_Motor_GPIO_Port, En_Motor_Pin, GPIO_PIN_SET); // Отключаем драйвера моторы// Установить пин HGH GPIO_PIN_SET — установить HIGH,  GPIO_PIN_RESET — установить LOW.
            modeControlMotor = 0;
            modeControlLaser = 0;
            HAL_I2C_DeInit(&hi2c1);        // Деинициализация I2C
            HAL_I2C_Init(&hi2c1);          // Повторная инициализация I2C
            flag_sendRequestBNO055 = true; // Взводим флаг что можно снова запрос к BNO055
            flag_sendRequestBNO055 = true; // Взводим флаг что можно снова запрос

#ifdef LASER80
            laser80_stopMeasurement(0);
            laser80_stopMeasurement(1);
            laser80_stopMeasurement(2);
            laser80_stopMeasurement(3);
#endif
#ifdef LASER60
            sk60plus_stopContinuous(0);
            sk60plus_stopContinuous(1);
            sk60plus_stopContinuous(2);
            sk60plus_stopContinuous(3);
#endif
        }
    }
}

// Заполнение данными Прошивки
void initFirmware()
{
    Modul2Data_send.firmware.gen = 1;
    Modul2Data_send.firmware.ver = 22;
    Modul2Data_send.firmware.debug = DEBUG_2;
#ifdef LASER60
    Modul2Data_send.firmware.laser = 60;
#endif
#ifdef LASER80
    Modul2Data_send.firmware.laser = 80;
#endif
    Modul2Data_send.firmware.motor = STEPMOTOR;
    printf("Firmware gen %hu ver %hu laser %hu motor %.1f debug %hu\n", Modul2Data_send.firmware.gen, Modul2Data_send.firmware.ver, Modul2Data_send.firmware.laser, Modul2Data_send.firmware.motor, Modul2Data_send.firmware.debug);
}

// Приведение угла к диапазону -180...+180
float normalizeAngle(float angle) 
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;
    return angle;
}





















#endif /*CODE_H*/
