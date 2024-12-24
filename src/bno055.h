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
uint8_t chip_id = 0;

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
};

struct BNO055_Info_s BNO055;
//****************************************** ОПРЕДЕЛЕНИЕ ФУНКЦИЙ ***********************************

HAL_StatusTypeDef BNO055_Read(uint8_t reg, uint8_t *buffer, uint16_t size); // Функция для чтения из регистра BNO055
HAL_StatusTypeDef BNO055_Write(uint8_t reg, uint8_t value);                 // Функция для записи в регистр BNO055

void BNO055_Read_IT(uint8_t reg, uint8_t *buffer, uint16_t size); // Функция для чтения данных из BNO055 используя прерывание
void BNO055_Write_IT(uint8_t reg, uint8_t value);                 // Функция для записи данных в BNO055 используя прерывание

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);

void BNO055_Init();
void BNO055_Reset();                // Перезапуск датчика
void BNO055_SetMode(uint8_t mode_); // Установка нужного режима работы
void BNO055_getStatusInfo();        // Запрос информации о статусе датчика
void BNO055_getRevInfo();           // Информация о прошивках датчика

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
//***************************** ЧЕРЕЗ ПРЕРЫВАНИЯ **************************************
// Функция для чтения данных из BNO055 используя прерывание
void BNO055_Read_IT(uint8_t reg, uint8_t *buffer, uint16_t size)
{
    i2cTransferComplete = 0;
    // Отправляем адрес регистра
    HAL_I2C_Master_Transmit_IT(&hi2c1, BNO055_ADDRESS, &reg, 1);
    // Ждем завершения
    while (!i2cTransferComplete)
        ;

    i2cTransferComplete = 0;
    // Читаем данные из регистра
    HAL_I2C_Master_Receive_IT(&hi2c1, BNO055_ADDRESS, buffer, size);
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
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        i2cTransferComplete = 1; // Флаг завершения операции
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    { // Проверяем, что ошибка относится к I2C1
        // Лог ошибок
        DEBUG_PRINTF("I2C Error: 0x%02lX\n", hi2c->ErrorCode);

        // Обработка ошибок
        if (hi2c->ErrorCode & HAL_I2C_ERROR_BERR)
        {
            DEBUG_PRINTF("Bus Error\n");
        }
        if (hi2c->ErrorCode & HAL_I2C_ERROR_ARLO)
        {
            DEBUG_PRINTF("Arbitration Lost\n");
        }
        if (hi2c->ErrorCode & HAL_I2C_ERROR_AF)
        {
            DEBUG_PRINTF("Acknowledge Failure\n");
        }
        if (hi2c->ErrorCode & HAL_I2C_ERROR_OVR)
        {
            DEBUG_PRINTF("Overrun/Underrun\n");
        }
        if (hi2c->ErrorCode & HAL_I2C_ERROR_TIMEOUT)
        {
            DEBUG_PRINTF("Timeout Error\n");
        }
        if (hi2c->ErrorCode & HAL_I2C_ERROR_DMA)
        {
            DEBUG_PRINTF("DMA Transfer Error\n");
        }

        // Попытка повторной инициализации или сброс шины I2C
        HAL_I2C_DeInit(hi2c); // Деинициализация I2C
        HAL_Delay(100);       // Короткая задержка
        HAL_I2C_Init(hi2c);   // Повторная инициализация

        // Возможно, добавить пользовательскую логику, например, уведомление системы
        DEBUG_PRINTF("I2C reinitialized\n");
    }
}
//*****************************************************************************************************
void BNO055_Init()
{
    DEBUG_PRINTF("BNO055_Init...\n");
    // Проверяем идентификатор чипа
    if (BNO055_Read(eBNO055_REGISTER_CHIP_ID, &chip_id, 1) == HAL_OK && chip_id == 0xA0)
    {
        DEBUG_PRINTF("chip_id = %X ", chip_id);
        DEBUG_PRINTF("BNO055 detected!\n");
        BNO055_Reset();
        BNO055_SetMode(eCONFIGMODE);               /* Go to config mode if not there */
        BNO055_Write(eBNO055_REGISTER_PAGE_ID, 0); // Устанавливаем работы с регистрами нулевой страницы
        HAL_Delay(25);
        BNO055_Write(eBNO055_REGISTER_PWR_MODE, eNORMAL_POWER_MODE); // Нормальный режим работы по питанию
        HAL_Delay(25);
        BNO055_getStatusInfo();
        BNO055_getRevInfo();  
    }
    else
    {
        printf("BNO055 not found!\n");
        while (1)
            ;
    }
}
// Перезапуск датчика
void BNO055_Reset()
{
    DEBUG_PRINTF("BNO055_Reset... \n");
    int timeOut = 0;
    chip_id = 0;
    BNO055_Write(eBNO055_REGISTER_SYS_TRIGGER, 0b00100000); /* reset the sensor */
    HAL_Delay(500);
    int err = 0;
    while (chip_id != 0xA0)
    {
        BNO055_Read(eBNO055_REGISTER_CHIP_ID, &chip_id, 1);
        HAL_Delay(100);
        DEBUG_PRINTF("WAITING BNO055.... %i ", timeOut);
        DEBUG_PRINTF("chip_id = %X \n", chip_id);
        if (++timeOut == 10)
        {
            err = 1;
            DEBUG_PRINTF("RESET BNO055 NOT ANSWER OVER 1 secunds !!! \n");
            break;
        }
    }
    if (!err)
    {
        DEBUG_PRINTF("chip_id = %X ", chip_id);
        DEBUG_PRINTF(" Successfully connected to  BNO055 after RESET.\n");
    }
}
// Установка нужного режима работы
void BNO055_SetMode(uint8_t mode_)
{
    BNO055_Write(eBNO055_REGISTER_OPR_MODE, mode_); // | eFASTEST_MODE);  /* Go to config mode if not there */
    DEBUG_PRINTF("BNO055_SetMode => %i \n", mode_);
    HAL_Delay(25);
}

// Запрос информации о статусе датчика
void BNO055_getStatusInfo()
{
    DEBUG_PRINTF(" === BNO055_getStatusInfo ===\n");

    BNO055_Write(eBNO055_REGISTER_PAGE_ID, 0); // Устанавливаем работы с регистрами нулевой страницы

    BNO055_Read(eBNO055_REGISTER_SYS_STATUS, &BNO055.SystemStatusCode, 1);
    DEBUG_PRINTF("BNO055.SystemStatusCode= ");
    if (BNO055.SystemStatusCode != 0)
    {
        DEBUG_PRINTF("%i", BNO055.SystemStatusCode);
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
        DEBUG_PRINTF(" Ok.\n");
    }

    BNO055_Read(eBNO055_REGISTER_ST_RESULT, &BNO055.SelfTestStatus, 1);
    DEBUG_PRINTF("BNO055.SelfTestStatus= ");
    if (BNO055.SelfTestStatus != 0b1111)
    {
        DEBUG_PRINTF("%i", BNO055.SelfTestStatus);
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
        DEBUG_PRINTF(" Ok.\n");
    }

    BNO055_Read(eBNO055_REGISTER_SYS_ERR, &BNO055.SystemError, 1);
    DEBUG_PRINTF("BNO055.SystemError= ");
    if (BNO055.SystemError != 0)
    {
        DEBUG_PRINTF("%i", BNO055.SystemError);
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
        DEBUG_PRINTF("Ok.\n");
    }
}

// Информация о прошивках датчика
void BNO055_getRevInfo()
{
    DEBUG_PRINTF(" === BNO055_getRevInfo ===\n");
    BNO055_Write(eBNO055_REGISTER_PAGE_ID, 0); // Устанавливаем работы с регистрами нулевой страницы

    /* Check the accelerometer revision */
    BNO055_Read(eBNO055_REGISTER_ACC_ID, &BNO055.accel_rev, 1);
    DEBUG_PRINTF("BNO055.accel_rev: %i\n", BNO055.accel_rev);

    /* Check the magnetometer revision */
    BNO055_Read(eBNO055_REGISTER_MAG_ID, &BNO055.mag_rev,1);
    DEBUG_PRINTF("BNO055.mag_rev: %i\n", BNO055.mag_rev);

    /* Check the gyroscope revision */
    BNO055_Read(eBNO055_REGISTER_GYR_ID, &BNO055.gyro_rev, 1);
    DEBUG_PRINTF("BNO055.gyro_rev: %i\n", BNO055.gyro_rev);

    /* Check the SW revision */
    BNO055_Read(eBNO055_REGISTER_BL_REV_ID, &BNO055.bl_rev, 1);
    DEBUG_PRINTF("BNO055.bl_rev: %i\n", BNO055.bl_rev);

    uint8_t a, b;
    BNO055_Read(eBNO055_REGISTER_SW_REV_ID_LSB, &a, 1);
    BNO055_Read(eBNO055_REGISTER_SW_REV_ID_MSB, &b, 1);
    BNO055.sw_rev = (((uint16_t)b) << 8) | ((uint16_t)a);
    DEBUG_PRINTF("BNO055.sw_rev: %lu\n", BNO055.sw_rev);
    DEBUG_PRINTF("--- END Init BNO055.\n");
}

#endif