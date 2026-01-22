/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BNO055_H
#define BNO055_H

#include "main.h"
#include "config.h"

#define BNO055_ADDRESS (0x29 << 1) // Адрес BNO055 (7-битный, сдвинут влево)

enum eBNO055PowerModes_t
{
    eNORMAL_POWER_MODE = 0b00000000,
    eLOW_POWER_MODE = 0b00000001,
    eSUSPEND_POWER_MODE = 0b00000010,
};

enum eBNO055Registers_t
{ /* DEFAULT    TYPE                  */
  /*page0*/
  eBNO055_REGISTER_CHIP_ID = 0x00,          /* 0x00       r                     */
  eBNO055_REGISTER_ACC_ID = 0x01,           /* 0xFB       r                     */
  eBNO055_REGISTER_MAG_ID = 0x02,           /* 0x32       r                     */
  eBNO055_REGISTER_GYR_ID = 0x03,           /* 0x0F       r                     */
  eBNO055_REGISTER_SW_REV_ID_LSB = 0x04,    /*            r                     */
  eBNO055_REGISTER_SW_REV_ID_MSB = 0x05,    /*            r                     */
  eBNO055_REGISTER_BL_REV_ID = 0x06,        /*            r                     */
  eBNO055_REGISTER_PAGE_ID = 0x07,          /* 0x00       rw                    */
  eBNO055_REGISTER_ACC_DATA_X_LSB = 0x08,   /* 0x00       r                     */
  eBNO055_REGISTER_ACC_DATA_X_MSB = 0x09,   /* 0x00       r                     */
  eBNO055_REGISTER_ACC_DATA_Y_LSB = 0x0A,   /* 0x00       r                     */
  eBNO055_REGISTER_ACC_DATA_Y_MSB = 0x0B,   /* 0x00       r                     */
  eBNO055_REGISTER_ACC_DATA_Z_LSB = 0x0C,   /* 0x00       r                     */
  eBNO055_REGISTER_ACC_DATA_Z_MSB = 0x0D,   /* 0x00       r                     */
  eBNO055_REGISTER_MAG_DATA_X_LSB = 0x0E,   /* 0x00       r                     */
  eBNO055_REGISTER_MAG_DATA_X_MSB = 0x0F,   /* 0x00       r                     */
  eBNO055_REGISTER_MAG_DATA_Y_LSB = 0x10,   /* 0x00       r                     */
  eBNO055_REGISTER_MAG_DATA_Y_MSB = 0x11,   /* 0x00       r                     */
  eBNO055_REGISTER_MAG_DATA_Z_LSB = 0x12,   /* 0x00       r                     */
  eBNO055_REGISTER_MAG_DATA_Z_MSB = 0x13,   /* 0x00       r                     */
  eBNO055_REGISTER_GYR_DATA_X_LSB = 0x14,   /* 0x00       r                     */
  eBNO055_REGISTER_GYR_DATA_X_MSB = 0x15,   /* 0x00       r                     */
  eBNO055_REGISTER_GYR_DATA_Y_LSB = 0x16,   /* 0x00       r                     */
  eBNO055_REGISTER_GYR_DATA_Y_MSB = 0x17,   /* 0x00       r                     */
  eBNO055_REGISTER_GYR_DATA_Z_LSB = 0x18,   /* 0x00       r                     */
  eBNO055_REGISTER_GYR_DATA_Z_MSB = 0x19,   /* 0x00       r                     */
  eBNO055_REGISTER_EUL_DATA_X_LSB = 0x1A,   /* 0x00       r                     */
  eBNO055_REGISTER_EUL_DATA_X_MSB = 0x1B,   /* 0x00       r                     */
  eBNO055_REGISTER_EUL_DATA_Y_LSB = 0x1C,   /* 0x00       r                     */
  eBNO055_REGISTER_EUL_DATA_Y_MSB = 0x1D,   /* 0x00       r                     */
  eBNO055_REGISTER_EUL_DATA_Z_LSB = 0x1E,   /* 0x00       r                     */
  eBNO055_REGISTER_EUL_DATA_Z_MSB = 0x1F,   /* 0x00       r                     */
  eBNO055_REGISTER_QUA_DATA_W_LSB = 0x20,   /* 0x00       r                     */
  eBNO055_REGISTER_QUA_DATA_W_MSB = 0x21,   /* 0x00       r                     */
  eBNO055_REGISTER_QUA_DATA_X_LSB = 0x22,   /* 0x00       r                     */
  eBNO055_REGISTER_QUA_DATA_X_MSB = 0x23,   /* 0x00       r                     */
  eBNO055_REGISTER_QUA_DATA_Y_LSB = 0x24,   /* 0x00       r                     */
  eBNO055_REGISTER_QUA_DATA_Y_MSB = 0x25,   /* 0x00       r                     */
  eBNO055_REGISTER_QUA_DATA_Z_LSB = 0x26,   /* 0x00       r                     */
  eBNO055_REGISTER_QUA_DATA_Z_MSB = 0x27,   /* 0x00       r                     */
  eBNO055_REGISTER_LIA_DATA_X_LSB = 0x28,   /* 0x00       r                     */
  eBNO055_REGISTER_LIA_DATA_X_MSB = 0x29,   /* 0x00       r                     */
  eBNO055_REGISTER_LIA_DATA_Y_LSB = 0x2A,   /* 0x00       r                     */
  eBNO055_REGISTER_LIA_DATA_Y_MSB = 0x2B,   /* 0x00       r                     */
  eBNO055_REGISTER_LIA_DATA_Z_LSB = 0x2C,   /* 0x00       r                     */
  eBNO055_REGISTER_LIA_DATA_Z_MSB = 0x2D,   /* 0x00       r                     */
  eBNO055_REGISTER_GRV_DATA_X_LSB = 0x2E,   /* 0x00       r                     */
  eBNO055_REGISTER_GRV_DATA_X_MSB = 0x2F,   /* 0x00       r                     */
  eBNO055_REGISTER_GRV_DATA_Y_LSB = 0x30,   /* 0x00       r                     */
  eBNO055_REGISTER_GRV_DATA_Y_MSB = 0x31,   /* 0x00       r                     */
  eBNO055_REGISTER_GRV_DATA_Z_LSB = 0x32,   /* 0x00       r                     */
  eBNO055_REGISTER_GRV_DATA_Z_MSB = 0x33,   /* 0x00       r                     */
  eBNO055_REGISTER_TEMP = 0x34,             /* 0x00       r                     */
  eBNO055_REGISTER_CALIB_STAT = 0x35,       /* 0x00       r                     */
  eBNO055_REGISTER_ST_RESULT = 0x36,        /* xxxx1111   r                     */
  eBNO055_REGISTER_INT_STA = 0x37,          /* 000x00xx   r  pg74               */
  eBNO055_REGISTER_SYS_CLK_STATUS = 0x38,   /* 00000000   r  pg74               */
  eBNO055_REGISTER_SYS_STATUS = 0x39,       /* 00000000   r  pg74               */
  eBNO055_REGISTER_SYS_ERR = 0x3A,          /* 00000000   r  pg75               */
  eBNO055_REGISTER_UNIT_SEL = 0x3B,         /* 0xx0x000   rw pg76               */
  eBNO055_REGISTER_OPR_MODE = 0x3D,         /* x???????   rw pg77               */
  eBNO055_REGISTER_PWR_MODE = 0x3E,         /* xxxxxx??   rw pg78               */
  eBNO055_REGISTER_SYS_TRIGGER = 0x3F,      /* 000xxxx0   w  pg78               */
  eBNO055_REGISTER_TEMP_SOURCE = 0x40,      /* xxxxxx??   rw pg78               */
  eBNO055_REGISTER_AXIS_MAP_CONFIG = 0x41,  /* xx??????   rw pg79               */
  eBNO055_REGISTER_AXIS_MAP_SIGN = 0x42,    /* xxxxx???   rw pg79               */
  eBNO055_REGISTER_SIC_MATRIX = 0x43,       /* xxxxxx??   ?? pg80               */
  eBNO055_REGISTER_ACC_OFFSET_X_LSB = 0x55, /* 0x00       rw                    */
  eBNO055_REGISTER_ACC_OFFSET_X_MSB = 0x56, /* 0x00       rw                    */
  eBNO055_REGISTER_ACC_OFFSET_Y_LSB = 0x57, /* 0x00       rw                    */
  eBNO055_REGISTER_ACC_OFFSET_Y_MSB = 0x58, /* 0x00       rw                    */
  eBNO055_REGISTER_ACC_OFFSET_Z_LSB = 0x59, /* 0x00       rw                    */
  eBNO055_REGISTER_ACC_OFFSET_Z_MSB = 0x5A, /* 0x00       rw                    */
  eBNO055_REGISTER_MAG_OFFSET_X_LSB = 0x5B, /* 0x00       rw                    */
  eBNO055_REGISTER_MAG_OFFSET_X_MSB = 0x5C, /* 0x00       rw                    */
  eBNO055_REGISTER_MAG_OFFSET_Y_LSB = 0x5D, /* 0x00       rw                    */
  eBNO055_REGISTER_MAG_OFFSET_Y_MSB = 0x5E, /* 0x00       rw                    */
  eBNO055_REGISTER_MAG_OFFSET_Z_LSB = 0x5F, /* 0x00       rw                    */
  eBNO055_REGISTER_MAG_OFFSET_Z_MSB = 0x60, /* 0x00       rw                    */
  eBNO055_REGISTER_GYR_OFFSET_X_LSB = 0x61, /* 0x00       rw                    */
  eBNO055_REGISTER_GYR_OFFSET_X_MSB = 0x62, /* 0x00       rw                    */
  eBNO055_REGISTER_GYR_OFFSET_Y_LSB = 0x63, /* 0x00       rw                    */
  eBNO055_REGISTER_GYR_OFFSET_Y_MSB = 0x64, /* 0x00       rw                    */
  eBNO055_REGISTER_GYR_OFFSET_Z_LSB = 0x65, /* 0x00       rw                    */
  eBNO055_REGISTER_GYR_OFFSET_Z_MSB = 0x66, /* 0x00       rw                    */
  eBNO055_REGISTER_ACC_RADIUS_LSB = 0x67,   /* 0x00       rw                    */
  eBNO055_REGISTER_ACC_RADIUS_MSB = 0x68,   /* 0x00       rw                    */
  eBNO055_REGISTER_MAG_RADIUS_LSB = 0x69,   /* 0x00       rw                    */
  eBNO055_REGISTER_MAG_RADIUS_MSB = 0x6A,   /* 0x00       rw                    */

  /*page 1*/

  /*      eBNO055_REGISTER_PAGE_ID             = 0x07,   ??         rw see page0          */
  eBNO055_REGISTER_ACC_CONFIG = 0x08,       /* 00001101   rw pg87               */
  eBNO055_REGISTER_MAG_CONFIG = 0x09,       /* 00001011   rw pg87               */
  eBNO055_REGISTER_GYR_CONFIG = 0x0A,       /* 00111000   rw pg88               */
  eBNO055_REGISTER_GYR_CONFIG_1 = 0x0B,     /* 00000000   rw pg88               */
  eBNO055_REGISTER_ACC_SLEEP_CONFIG = 0x0C, /* ????????   rw pg89               */
  eBNO055_REGISTER_GYR_SLEEP_CONFIG = 0x0D, /* ????????   rw pg90               */
  eBNO055_REGISTER_INT_MSK = 0x0F,          /* 000x00xx   rw pg91               */
  eBNO055_REGISTER_INT_EN = 0x10,           /* 000x00xx   rw pg92               */
  eBNO055_REGISTER_ACC_AM_THRES = 0x11,     /* 00010100   rw pg92               */
  eBNO055_REGISTER_ACC_INT_SETTINGS = 0x12, /* 00000011   rw pg93               */
  eBNO055_REGISTER_ACC_HG_DURATION = 0x13,  /* 00001111   rw pg93               */
  eBNO055_REGISTER_ACC_HG_THRES = 0x14,     /* 11000000   rw pg93               */
  eBNO055_REGISTER_ACC_NM_THRES = 0x15,     /* 00001010   rw pg93               */
  eBNO055_REGISTER_ACC_NM_SET = 0x16,       /* x0001011   rw pg94               */
  eBNO055_REGISTER_GYR_INT_SETTING = 0x17,  /* 00000000   rw pg95               */
  eBNO055_REGISTER_GYR_HR_X_SET = 0x18,     /* 00000001   rw pg95               */
  eBNO055_REGISTER_GYR_DUR_X = 0x19,        /* 00011001   rw pg96               */
  eBNO055_REGISTER_ACC_HR_Y_SET = 0x1A,     /* 00000001   rw pg96               */
  eBNO055_REGISTER_GYR_DUR_Y = 0x1B,        /* 00011001   rw pg96               */
  eBNO055_REGISTER_ACC_HR_Z_SET = 0x1C,     /* 00000001   rw pg97               */
  eBNO055_REGISTER_GYR_DUR_Z = 0x1D,        /* 00011001   rw pg97               */
  eBNO055_REGISTER_GYR_AM_THRES = 0x1E,     /* 00000100   rw pg97               */
  eBNO055_REGISTER_GYR_AM_SET = 0x1F,       /* 00001010   rw pg98               */
};

enum eBNO055Mode_t
{                             /*HW SENS POWER    SENS SIG         FUSION       */
                              /*  A   M   G       A   M   G       E   Q   L   G*/
  eCONFIGMODE = 0b00000000,   /*  y   y   y       n   n   n       n   n   n   n*/
  eACCONLY = 0b00000001,      /*  y   n   n       y   n   n       n   n   n   n*/
  eMAGONLY = 0b00000010,      /*  n   y   n       n   y   n       n   n   n   n*/
  eGYROONLY = 0b00000011,     /*  n   n   y       n   n   y       n   n   n   n*/
  eACCMAG = 0b00000100,       /*  y   y   n       y   y   n       n   n   n   n*/
  eACCGYRO = 0b00000101,      /*  y   n   y       y   n   y       n   n   n   n*/
  eMAGGYRO = 0b00000110,      /*  n   y   y       n   y   y       n   n   n   n*/
  eAMG = 0b00000111,          /*  y   y   y       y   y   y       n   n   n   n*/
  eIMU = 0b00001000,          /*  y   n   y       y   n   y       y   y   y   y*/
  eCOMPASS = 0b00001001,      /*  y   y   n       y   y   n       y   y   y   y*/
  eM4G = 0b00001010,          /*  y   y   n       y   y   y       y   y   y   y*/
  eNDOF_FMC_OFF = 0b00001011, /*  y   y   y       y   y   y       y   y   y   y*/
  eNDOF = 0b00001100,         /*  y   y   y       y   y   y       y   y   y   y*/
};

// #define BNO055_CHIP_ID 0x00        // Регистр идентификатора чипа
// #define BNO055_OPR_MODE 0x3D       // Регистр режима работы
// #define BNO055_EULER_H_LSB 0x1A    // Начало данных углов Эйлера (Heading, Roll, Pitch)

uint8_t bno_txBuffer[2];                  // Буфер передачи
uint8_t bno_rxBuffer[6];                  // Буфер приема
volatile uint8_t i2cTransferComplete = 0; // Флаг завершения операции
volatile uint8_t i2cReceiveComplete = 0;  // Флаг завершения операции
uint8_t chip_id = 0;

#define OFFSET_SIZE 22
uint8_t BNO055_Offset_Array_dafault1[OFFSET_SIZE] = {234, 255, 18, 0, 228, 255, 248, 255, 40, 254, 248, 255, 253, 255, 1, 0, 1, 0, 232, 3, 176, 4}; // Массив для хранения офсетов по умолчанию
uint8_t BNO055_Offset_Array_dafault2[OFFSET_SIZE] = {240, 255, 7, 0, 249, 255, 0, 0, 0, 0, 0, 0, 255, 255, 255, 255, 255, 255, 232, 3, 224, 1};     // Красный датчик
uint8_t BNO055_Offset_Array_dafault2025[OFFSET_SIZE] = {240, 255, 7, 0, 249, 255, 26, 0, 57, 0, 30, 0, 254, 255, 1, 0, 1, 0, 232, 3, 126, 4};       // На даче в 2025 году на тестовой плате
uint8_t BNO055_Offset_Array[OFFSET_SIZE] = {0};                                                                                                     //

struct BNO055_Info_s
{
    uint8_t SystemStatusCode;
    uint8_t SelfTestStatus;
    uint8_t SystemError;
    uint8_t accel_rev;
    uint8_t mag_rev;
    uint8_t gyro_rev;
    uint8_t bl_rev;
    uint32_t sw_rev;
    uint8_t Calibr_sys;
    uint8_t Calibr_accel;
    uint8_t Calibr_gyro;
    uint8_t Calibr_mag;
    uint8_t MAP_Config;
    uint8_t MAP_Sign;
    uint8_t Calibr_Status;
};

struct BNO055_Info_s BNO055;
//****************************************** ОПРЕДЕЛЕНИЕ ФУНКЦИЙ ***********************************

HAL_StatusTypeDef BNO055_Read(uint8_t reg, uint8_t *buffer, uint16_t size);      // Функция для чтения из регистра BNO055
HAL_StatusTypeDef BNO055_Write(uint8_t reg, uint8_t value);                      // Функция для записи в регистр BNO055
HAL_StatusTypeDef BNO055_Mem_Write(uint8_t reg, uint8_t *data_, uint16_t size_); // Функция для записи массива в регистры последовательно BNO055

void BNO055_Transmit_IT(uint8_t reg);                   // Функция для отправки команды используя перрывание
void BNO055_Receive_IT(uint8_t *buffer, uint16_t size); // Функция для чтения данных из BNO055 используя прерывание
void BNO055_Write_IT(uint8_t reg, uint8_t value);       // Функция для записи данных в BNO055 используя прерывание

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);

void BNO055_Init();
void BNO055_Reset();                                      // Перезапуск датчика
void BNO055_SetMode(uint8_t mode_);                       // Установка нужного режима работы
void BNO055_StatusInfo();                                 // Запрос информации о статусе датчика
void BNO055_RevInfo();                                    // Информация о прошивках датчика
void BNO055_GetOffset_from_BNO055(uint8_t *offsetArray_); // Считывание оффсет из датчика
void BNO055_SetOffset_to_BNO055(uint8_t *offsetArray_);   // Запись оффсет в датчик
void BNO055_ReadData();                                   // Разовое считывание данных
void BNO055_SetOrientation();                             // Установка ориентации датчика.
void BNO055_StatusCalibr();                               // Запрос статуса колибровки

//****************************************** РЕАЛИЗАЦИЯ ФУНКЦИЙ ***********************************

//******************************* ОПРОС ДАТЧИКА ЧЕРЕЗ ОПРОС_ОТВЕТ ******************************
// Функция для чтения из регистра BNO055
HAL_StatusTypeDef BNO055_Read(uint8_t reg, uint8_t *buffer, uint16_t size)
{
    // Отправляем адрес регистра
    if (HAL_I2C_Master_Transmit(&hi2c1, BNO055_ADDRESS, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        return HAL_ERROR;
    }
    // Читаем данные
    return HAL_I2C_Master_Receive(&hi2c1, BNO055_ADDRESS, buffer, size, HAL_MAX_DELAY);
}
// Функция для записи в регистр BNO055
HAL_StatusTypeDef BNO055_Write(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return HAL_I2C_Master_Transmit(&hi2c1, BNO055_ADDRESS, data, sizeof(data), HAL_MAX_DELAY);
}
// Функция для записи массива в регистры последовательно BNO055
HAL_StatusTypeDef BNO055_Mem_Write(uint8_t reg, uint8_t *data_, uint16_t size_)
{
    return HAL_I2C_Mem_Write(&hi2c1, BNO055_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data_, size_, HAL_MAX_DELAY);
}
//***************************** ЧЕРЕЗ ПРЕРЫВАНИЯ **************************************
// Функция для чтения данных из BNO055 используя прерывание
void BNO055_Transmit_IT(uint8_t _reg)
{
    i2cTransferComplete = 0;
    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) // Проверяем, готов ли I2C
    {
        HAL_Delay(1); // Ждем 1 милисекунду в надежде что шина освободиться
        printf("BNO055_Transmit_IT I2C not ready, HAL_Delay 1 msec \n");
    }

    // uint32_t start_time = HAL_GetTick(); // Вариант 1 : С блокирующим ожиданием(но не более 1 мс)
    // while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
    // {
    //     if (HAL_GetTick() - start_time >= 1)
    //     { // Проверяем, не прошло ли уже 1 мс
    //         printf("BNO055_Transmit_IT I2C timeout after 1 ms\n");
    //         break; // Выходим из цикла по таймауту
    //     }
    //     __NOP();
    //     __NOP();
    //     __NOP(); // 3 такта паузы
    // }

    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) // Проверяем второй раз, готов ли I2C
    {
        printf("BNO055_Transmit_IT I2C not ready, resetting...\n");
        HAL_I2C_DeInit(&hi2c1); // Деинициализация I2C
        HAL_I2C_Init(&hi2c1);   // Повторная инициализация I2C
    }

    static uint8_t reg = eBNO055_REGISTER_ACC_DATA_X_LSB;                                   // Статическая переменная для хранения регистра Передаём адрес переменной reg, который сохранится после выхода из функции
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit_IT(&hi2c1, BNO055_ADDRESS, &reg, 1); // Отправляем адрес регистра
    if (status != HAL_OK)
    {
        printf("BNO055_Transmit_IT ERROR -> ");
        switch (status) // Обработка ошибки
        {
        case HAL_OK:
            printf("I2C transmission started successfully\n");
            break;
        case HAL_ERROR:
            printf("I2C HAL_ERROR: %lu\n", HAL_I2C_GetError(&hi2c1));
            break;
        case HAL_BUSY:
            printf("I2C HAL_BUSY: Previous operation not completed\n");
            break;
        case HAL_TIMEOUT:
            printf("I2C HAL_TIMEOUT: Transmission timeout\n");
            break;
        default:
            printf("I2C Unknown error: %lu\n", HAL_I2C_GetError(&hi2c1));
            break;
        }
    }

    // uint64_t startTime = micros(); // Запоминаем время начала передачи
    // while (startTime + 500 > micros() ) // Ждем завершения передачи
    // {
    // }
    // DEBUG_PRINTF("BNO055_Transmit_IT \n");
}

// Функция для чтения данных из BNO055 используя прерывание
void BNO055_Receive_IT(uint8_t *buffer, uint16_t size)
{
    i2cReceiveComplete = 0;
    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) // Проверяем, готов ли I2C
        printf("BNO055_Receive_IT I2C not ready, resetting...\n");
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive_IT(&hi2c1, BNO055_ADDRESS, buffer, size); // Запускаем чтение данных из регистра
    if (status != HAL_OK)
    {
        printf("BNO055_Receive_IT ERROR\n");
        switch (status) // Обработка ошибки
        {
        case HAL_OK:
            printf("I2C transmission started successfully\n");
            break;
        case HAL_ERROR:
            printf("I2C HAL_ERROR: %lu\n", HAL_I2C_GetError(&hi2c1));
            break;
        case HAL_BUSY:
            printf("I2C HAL_BUSY: Previous operation not completed\n");
            break;
        case HAL_TIMEOUT:
            printf("I2C HAL_TIMEOUT: Transmission timeout\n");
            break;
        default:
            printf("I2C Unknown error: %lu\n", HAL_I2C_GetError(&hi2c1));
            break;
        }
    }
}

// Функция для записи данных в BNO055 используя прерывание
void BNO055_Write_IT(uint8_t reg, uint8_t value)
{
    txBuffer[0] = reg;   // Устанавливаем адрес регистра
    txBuffer[1] = value; // Данные для записи
    i2cTransferComplete = 0;
    HAL_I2C_Master_Transmit_IT(&hi2c1, BNO055_ADDRESS, txBuffer, 2);
}
//*********************************************** CALLBACK *********************
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        i2cTransferComplete = 1; // Флаг завершения операции
        // DEBUG_PRINTF("tx in %lu\n",millis());
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        i2cReceiveComplete = 1; // Флаг завершения операции
        // DEBUG_PRINTF("rx in %lu\n",millis());
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    { // Проверяем, что ошибка относится к I2C1
        // Лог ошибок
        printf("I2C Error: 0x%02lX\n", hi2c->ErrorCode);

        // Обработка ошибок
        if (hi2c->ErrorCode & HAL_I2C_ERROR_BERR)
        {
            printf("Bus Error\n");
        }
        if (hi2c->ErrorCode & HAL_I2C_ERROR_ARLO)
        {
            printf("Arbitration Lost\n");
        }
        if (hi2c->ErrorCode & HAL_I2C_ERROR_AF)
        {
            __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_AF); // Очистка ошибки
            printf("Acknowledge Failure\n");
        }
        if (hi2c->ErrorCode & HAL_I2C_ERROR_OVR)
        {
            printf("Overrun/Underrun\n");
        }
        if (hi2c->ErrorCode & HAL_I2C_ERROR_TIMEOUT)
        {
            printf("Timeout Error\n");
        }
        if (hi2c->ErrorCode & HAL_I2C_ERROR_DMA)
        {
            printf("DMA Transfer Error\n");
        }

        // Попытка повторной инициализации или сброс шины I2C
        printf("I2C HAL_I2C_DeInit... -> ");
        HAL_I2C_DeInit(hi2c); // Деинициализация I2C
        // HAL_Delay(3);       // Короткая задержка
        printf("I2C HAL_I2C_Init. -> ");
        HAL_I2C_Init(hi2c);            // Повторная инициализация
        printf("I2C reinitialized\n"); // Возможно, добавить пользовательскую логику, например, уведомление системы
    }
}
//*****************************************************************************************************
void BNO055_Init()
{
    printf("+++ BNO055_Init\n");
    // Проверяем идентификатор чипа
    if (BNO055_Read(eBNO055_REGISTER_CHIP_ID, &chip_id, 1) == HAL_OK && chip_id == 0xA0)
    {
        printf("    chip_id = %X ", chip_id);
        printf("    BNO055 detected!\n");
        BNO055_Reset();
        BNO055_SetMode(eCONFIGMODE);               /* Go to config mode if not there */
        BNO055_Write(eBNO055_REGISTER_PAGE_ID, 0); // Устанавливаем работы с регистрами нулевой страницы
        HAL_Delay(25);
        BNO055_Write(eBNO055_REGISTER_PWR_MODE, eNORMAL_POWER_MODE); // Нормальный режим работы по питанию
        HAL_Delay(25);
        BNO055_Write(eBNO055_REGISTER_SYS_TRIGGER, 0b10000000); // Установка использовать внешний кристал обычно SYS_TRIGGER, адрес 0x3F). Бит EXT_CLK_SEL (бит 7) устанавливается в 1, переключая датчик на внешний кристалл.
        HAL_Delay(25);
        BNO055_StatusInfo();
        BNO055_RevInfo();
        BNO055_GetOffset_from_BNO055(BNO055_Offset_Array);
        // writeUint8ToFile(BNO055_Offset_Array_dafault2025, 22, "bno055.cfg");  // Вызов функции для целых чисел
        // readUint8FromFile(BNO055_Offset_Array, 22, "bno055.cfg"); // Вызов функции для целых чисел
        BNO055_SetOffset_to_BNO055(BNO055_Offset_Array_dafault2025);
        BNO055_GetOffset_from_BNO055(BNO055_Offset_Array);
        BNO055_SetOrientation(); // Установка ориентации датчика.
        BNO055_StatusInfo();
        BNO055_StatusCalibr();
        // BNO055_SetMode(eGYROONLY); // Режим работы где он все сам считает	  eIMU
        BNO055_SetMode(eIMU); // Режим работы где он все сам считает	  eIMU
        // BNO055_SetMode(eNDOF_FMC_OFF); // Режим работы где он все сам считает	  eIMU
        // BNO055_SetMode(eNDOF); // Режим работы где он все сам считает	  eIMU
        HAL_Delay(500);
        for (size_t i = 0; i < 3; i++)
        {
            BNO055_StatusCalibr();
            HAL_Delay(25);
        }

        // BNO055_ReadData(); // Разовое считывание данных
        printf("--- BNO055_Init End. \n");
        HAL_Delay(2000);
    }
    else
    {
        printf("BNO055 not found!\n");
        // while (1)
        //     ;
    }
}
// Перезапуск датчика
void BNO055_Reset()
{
    printf("+++ BNO055_Reset\n");
    int timeOut = 0;
    chip_id = 0;
    BNO055_Write(eBNO055_REGISTER_SYS_TRIGGER, 0b00100000); /* reset the sensor */
    HAL_Delay(500);
    int err = 0;
    while (chip_id != 0xA0)
    {
        BNO055_Read(eBNO055_REGISTER_CHIP_ID, &chip_id, 1);
        HAL_Delay(100);
        printf("    WAITING BNO055.... %i ", timeOut);
        printf("    chip_id = %X \n", chip_id);
        if (++timeOut == 10)
        {
            err = 1;
            printf("RESET BNO055 NOT ANSWER OVER 1 secunds !!! \n");
            break;
        }
    }
    if (!err)
    {
        printf("    chip_id = %X ", chip_id);
        printf("    Successfully connected to  BNO055 after RESET.\n");
    }
    printf("--- BNO055_Reset\n");
}
// Установка нужного режима работы
void BNO055_SetMode(uint8_t mode_)
{
    BNO055_Write(eBNO055_REGISTER_OPR_MODE, mode_); // | eFASTEST_MODE);  /* Go to config mode if not there */
    printf("    BNO055_SetMode => %i \n", mode_);
    HAL_Delay(50);
}
// Установка ориентации как установлен датчик. Для применения требуется перезагрузка по питанию
void BNO055_SetOrientation()
{
    printf("+++ BNO055_SetOrientation... \n");
    /*     Placement AXIS_REMAP_CONFIG AXIS_REMAP_SIGN
    P0 0x21 0x04
    P1 (default) 0x24 0x00
    P2 0x24 0x06
    P3 0x21 0x02
    P4 0x24 0x03
    P5 0x21 0x01 - применяю этот вариант
    P6 0x21 0x07
    P7 0x24 0x05
    */
    uint8_t AXIS_MAP_CONFIG = 0x24; // P1 по даташиту
    uint8_t AXIS_MAP_SIGN = 0x00;
    BNO055_Write(eBNO055_REGISTER_AXIS_MAP_CONFIG, AXIS_MAP_CONFIG);
    printf("    Set BNO055_AXIS_MAP_CONFIG => %#X \n", AXIS_MAP_CONFIG);
    BNO055_Write(eBNO055_REGISTER_AXIS_MAP_SIGN, AXIS_MAP_SIGN);
    printf("    Set BNO055_AXIS_MAP_SIGN => %#X \n", AXIS_MAP_SIGN);

    /* Check the REGISTER_AXIS_MAP_CONFIG */
    BNO055_Read(eBNO055_REGISTER_AXIS_MAP_CONFIG, &BNO055.MAP_Config, 1);
    printf("    Read BNO055.MAP_Config: %#X\n", BNO055.MAP_Config);

    /* Check the REGISTER_AXIS_MAP_SIGN */
    BNO055_Read(eBNO055_REGISTER_AXIS_MAP_SIGN, &BNO055.MAP_Sign, 1);
    printf("    Read BNO055.MAP_Sign: %#X\n", BNO055.MAP_Sign);

    printf("--- BNO055_SetOrientation\n");
}

// Запрос статуса колибровки
void BNO055_StatusCalibr()
{
    // printf("+++ BNO055_StatusCalibr ");

    BNO055_Write(eBNO055_REGISTER_PAGE_ID, 0); // Устанавливаем работы с регистрами нулевой страницы
    BNO055_Read(eBNO055_REGISTER_CALIB_STAT, &BNO055.Calibr_Status, 1);
    printf("%i |", BNO055.Calibr_Status);

    uint8_t calData = BNO055.Calibr_Status;
    BNO055.Calibr_sys = (calData >> 6) & 0x03;
    printf(" Calibr_sys  : %u", BNO055.Calibr_sys);

    BNO055.Calibr_gyro = (calData >> 4) & 0x03;
    printf(" Calibr_gyro : %u", BNO055.Calibr_gyro);

    BNO055.Calibr_accel = (calData >> 2) & 0x03;
    printf(" Calibr_accel: %u", BNO055.Calibr_accel);

    BNO055.Calibr_mag = calData & 0x03;
    printf(" Calibr_mag  : %u", BNO055.Calibr_mag);

    if (BNO055.Calibr_sys < 3 || BNO055.Calibr_gyro < 3 || BNO055.Calibr_accel < 3 || BNO055.Calibr_mag < 3)
    {
        printf(" Calibrovka FALSE.\n");
    }
    else
    {
        printf(" Calibrovka TRUE.\n");
        BNO055_GetOffset_from_BNO055(BNO055_Offset_Array); // Считывание оффсет из датчика
        // writeUint8ToFile(BNO055_Offset_Array, 22, "bno055.cfg");  // Запись оффсетов в файл
        return;
    }
}

// Запрос информации о статусе датчика
void BNO055_StatusInfo()
{
    printf("+++ BNO055_getStatusInfo\n");

    BNO055_Write(eBNO055_REGISTER_PAGE_ID, 0); // Устанавливаем работы с регистрами нулевой страницы

    BNO055_Read(eBNO055_REGISTER_SYS_STATUS, &BNO055.SystemStatusCode, 1);
    printf("    BNO055.SystemStatusCode= ");
    if (BNO055.SystemStatusCode != 0)
    {
        printf("%i", BNO055.SystemStatusCode);
        /* System Status (see section 4.3.58)
       ---------------------------------
       0 = Idle
       1 = System Error
       2 = Initializing Peripherals
       3 = System Iniitalization
       4 = Executing Self-Test
       5 = Sensor fusio algorithm running
       6 = System running without fusion algorithms */
    }
    else
    {
        printf("    Ok.\n");
    }

    BNO055_Read(eBNO055_REGISTER_ST_RESULT, &BNO055.SelfTestStatus, 1);
    printf("    BNO055.SelfTestStatus= ");
    if (BNO055.SelfTestStatus != 0b1111)
    {
        printf("%i", BNO055.SelfTestStatus);
        /* Self Test Results (see section )
       --------------------------------
       1 = test passed, 0 = test failed

       Bit 0 = Accelerometer self test
       Bit 1 = Magnetometer self test
       Bit 2 = Gyroscope self test
       Bit 3 = MCU self test

       0b1111 = all good! */
    }
    else
    {
        printf("Ok.\n");
    }

    BNO055_Read(eBNO055_REGISTER_SYS_ERR, &BNO055.SystemError, 1);
    printf("    BNO055.SystemError= ");
    if (BNO055.SystemError != 0)
    {
        printf("%i", BNO055.SystemError);
        /* System Error (see section 4.3.59)
           ---------------------------------
           0 = No error
           1 = Peripheral initialization error
           2 = System initialization error
           3 = Self test result failed
           4 = Register map value out of range
           5 = Register map address out of range
           6 = Register map write error
           7 = BNO low power mode not available for selected operat ion mode
           8 = Accelerometer power mode not available
           9 = Fusion algorithm configuration error
           A = Sensor configuration error */
        HAL_Delay(5000);
    }
    else
    {
        printf("Ok.\n");
    }
}

// Информация о прошивках датчика
void BNO055_RevInfo()
{
    printf("+++ BNO055_getRevInfo\n");
    BNO055_Write(eBNO055_REGISTER_PAGE_ID, 0); // Устанавливаем работы с регистрами нулевой страницы

    /* Check the accelerometer revision */
    BNO055_Read(eBNO055_REGISTER_ACC_ID, &BNO055.accel_rev, 1);
    printf("    BNO055.accel_rev: %i\n", BNO055.accel_rev);

    /* Check the magnetometer revision */
    BNO055_Read(eBNO055_REGISTER_MAG_ID, &BNO055.mag_rev, 1);
    printf("    BNO055.mag_rev: %i\n", BNO055.mag_rev);

    /* Check the gyroscope revision */
    BNO055_Read(eBNO055_REGISTER_GYR_ID, &BNO055.gyro_rev, 1);
    printf("    BNO055.gyro_rev: %i\n", BNO055.gyro_rev);

    /* Check the SW revision */
    BNO055_Read(eBNO055_REGISTER_BL_REV_ID, &BNO055.bl_rev, 1);
    printf("    BNO055.bl_rev: %i\n", BNO055.bl_rev);

    uint8_t a, b;
    BNO055_Read(eBNO055_REGISTER_SW_REV_ID_LSB, &a, 1);
    BNO055_Read(eBNO055_REGISTER_SW_REV_ID_MSB, &b, 1);
    BNO055.sw_rev = (((uint16_t)b) << 8) | ((uint16_t)a);
    printf("    BNO055.sw_rev: %lu\n", BNO055.sw_rev);

    printf("--- END Init BNO055.\n");
}
// Считывание оффсет из датчика
void BNO055_GetOffset_from_BNO055(uint8_t *offsetArray_)
{
    printf("BNO055_GetOffset_from_BNO055\n");
    BNO055_SetMode(eCONFIGMODE); /* Go to config mode if not there */
    BNO055_Read(eBNO055_REGISTER_ACC_OFFSET_X_LSB, offsetArray_, OFFSET_SIZE);

    for (uint8_t i = 0; i < OFFSET_SIZE; i++)
    {
        printf(" = %i", offsetArray_[i]);
    }
    printf("\n");
}
// Запись оффсет в датчик
void BNO055_SetOffset_to_BNO055(uint8_t *offsetArray_)
{
    printf("BNO055_SetOffset_toBNO055\n");
    BNO055_SetMode(eCONFIGMODE); /* Go to config mode if not there */

    for (uint8_t i = 0; i < OFFSET_SIZE; i++)
    {
        printf(" - %u", offsetArray_[i]);
    }
    printf("\n");

    BNO055_Mem_Write(eBNO055_REGISTER_ACC_OFFSET_X_LSB, offsetArray_, OFFSET_SIZE);
}
// Разбор данных из буфера и запись знвеяний в переменные
void calcBufferBNO(uint8_t *buffer)
{
    // DEBUG_PRINTF("+++ calcBuffer ");
    // for (int i = 0; i < 20; i++)
    // {
    //     DEBUG_PRINTF("=%i ",buffer[i]);
    // }
    // DEBUG_PRINTF ("\n");

    struct SXyz eulerAngles;
    struct SXyz linAccData;
    struct SXyz accelData;
    struct SXyz gyrolData;
    struct SXyz magData;

    uint8_t aHigh = 0, aLow = 0, bLow = 0, bHigh = 0, cLow = 0, cHigh = 0;
    static uint32_t timeBNO = 0;
    if (timeBNO == 0)
        timeBNO = millis(); // Инициализация первого раза

    // ACCELERATION ---------------------------------------------
    aLow = buffer[0];
    aHigh = buffer[1];
    bLow = buffer[2];
    bHigh = buffer[3];
    cLow = buffer[4];
    cHigh = buffer[5];

    accelData.x = (int16_t)(aLow | (aHigh << 8)) / 100.; //  1 m/s2 = 100 LSB
    accelData.y = (int16_t)(bLow | (bHigh << 8)) / 100.;
    accelData.z = (int16_t)(cLow | (cHigh << 8)) / 100.;

    // DEBUG_PRINTF("BNO055 Accel x= %+8.2f y= %+8.2f z= %+8.2f | ", accelData.x, accelData.y, accelData.z);

    // MAGNETROMETR  ---------------------------------------------
    aLow = buffer[6];
    aHigh = buffer[7];
    bLow = buffer[8];
    bHigh = buffer[9];
    cLow = buffer[10];
    cHigh = buffer[11];

    magData.x = (int16_t)(aLow | (aHigh << 8)) / 16.; //  Representation 1 µT = 16 LSB
    magData.y = (int16_t)(bLow | (bHigh << 8)) / 16.;
    magData.z = (int16_t)(cLow | (cHigh << 8)) / 16.;

    // DEBUG_PRINTF("magData x= %+8.2f y= %+8.2f z= %+8.2f | ", magData.x, magData.y, magData.z);

    // GYROSCOPE  ---------------------------------------------
    aLow = buffer[12];
    aHigh = buffer[13];
    bLow = buffer[14];
    bHigh = buffer[15];
    cLow = buffer[16];
    cHigh = buffer[17];

    gyrolData.x = (int16_t)(aLow | (aHigh << 8)) / 16.; // Table 3-22: Gyroscope unit settings  1 Dps = 16 LSB (gradus)     1 Rps = 900 LSB (radian)
    gyrolData.y = (int16_t)(bLow | (bHigh << 8)) / 16.;
    gyrolData.z = (int16_t)(cLow | (cHigh << 8)) / 16.;

    static axises smoothed_data = {0, 0, 0}; // Начальные значения
    float const ALPHA = 0.5;
    // Экспоненциальное сглаживание везде по всем осям используем один коефициент
    smoothed_data.x = ALPHA * gyrolData.x + (1 - ALPHA) * smoothed_data.x;
    smoothed_data.y = ALPHA * gyrolData.y + (1 - ALPHA) * smoothed_data.y;
    smoothed_data.z = ALPHA * gyrolData.z + (1 - ALPHA) * smoothed_data.z;

    // DEBUG_PRINTF("BNO055 Gyro raw = %+8.3f %+8.3f %+8.3f smoothed= %+8.3f %+8.3f %+8.3f | ", gyrolData.x, gyrolData.y, gyrolData.z, smoothed_data.x, smoothed_data.y, smoothed_data.z);

    gyrolData.x = smoothed_data.x;
    gyrolData.y = smoothed_data.y;
    gyrolData.z = smoothed_data.z;

    // DEBUG_PRINTF("BNO055 gyro x= %+8.3f y= %+8.3f z= %+8.3f | ", gyrolData.x, gyrolData.y, gyrolData.z);

    // a,b,c это регистры которые мы считываем. В них могут быть значения для любых осей. Оси переопределяются в eBNO055_REGISTER_AXIS_MAP_CONFIG в зависимости от положения датчика.
    // Я просто подбираю нужную ось и знак. Если нужно переделываю на 360 градусов или +-180

    // УГЛЫ---------------------------------------------
    aLow = buffer[18];
    aHigh = buffer[19];
    bLow = buffer[20];
    bHigh = buffer[21];
    cLow = buffer[22];
    cHigh = buffer[23];

    /* Shift values to create properly formed integer (low byte first) */ /* 1 degree = 16 LSB  1 radian = 900 LSB   */
    eulerAngles.x = -(int16_t)(cLow | (cHigh << 8)) / 16.;
    eulerAngles.y = -(int16_t)(bLow | (bHigh << 8)) / 16.;
    eulerAngles.z = (int16_t)(aLow | (aHigh << 8)) / 16.;

    // DEBUG_PRINTF(" eulerAngles x= %+8.3f y= %+8.3f z= %+8.3f | ", eulerAngles.x, eulerAngles.y, eulerAngles.z);

    // УСКОРЕНИЕ---------------------------------------------
    aLow = buffer[32];
    aHigh = buffer[33];
    bLow = buffer[34];
    bHigh = buffer[35];
    cLow = buffer[36];
    cHigh = buffer[37];

    // Перевод в m/s2 1m/s2 = 100 LSB, mg = 1LSB
    linAccData.x = (int16_t)(aLow | (aHigh << 8)) / 100.;
    linAccData.y = (int16_t)(bLow | (bHigh << 8)) / 100.;
    linAccData.z = (int16_t)(cLow | (cHigh << 8)) / 100.; // Дальше не используем так как не летаем а ездим по плоскости. И заменяем на угловую скорость полученную из угла Эллера

    // DEBUG_PRINTF("lin x= %+8.4f y= %+8.4f z= %+8.4f  //  ", linAccData.x, linAccData.y, linAccData.z);
    // DEBUG_PRINTF("\n");

    bno055.status = 0;
    bno055.angleEuler = eulerAngles;
    bno055.linear = linAccData;
    bno055.accel = accelData;
    bno055.gyro = gyrolData;
    bno055.mag = magData;
    bno055.rate = (float)1000.0 / (millis() - timeBNO); // Считаем частоту
    timeBNO = millis();
    // DEBUG_PRINTF("--- calcBuffer \n");
}
// Разовое считывание данных
void BNO055_ReadData()
{
    // DEBUG_PRINTF("+++ BNO055_ReadData\n");
    uint8_t buffer[38];
    if (BNO055_Read(eBNO055_REGISTER_ACC_DATA_X_LSB, buffer, 38) == HAL_OK) // Считываем в буфер
    {
        calcBufferBNO(buffer);
        // DEBUG_PRINTF("    BNO055_ReadData BNO055 x= %.4f y= %.4f z= %.4f \n", bno055.angleEuler.x, bno055.angleEuler.y, bno055.angleEuler.z);
    }
    else
    {
        DEBUG_PRINTF("BNO055_ReadData ERROR\n");
    }
    // DEBUG_PRINTF("--- BNO055_ReadData\n");
}

#endif