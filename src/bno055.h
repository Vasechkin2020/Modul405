/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BNO055_H
#define BNO055_H

#include "main.h"
#include "config.h"

#define BNO055_ADDRESS (0x29 << 1) // Адрес BNO055 (7-битный, сдвинут влево)
#define BNO055_CHIP_ID 0x00        // Регистр идентификатора чипа
#define BNO055_OPR_MODE 0x3D       // Регистр режима работы
#define BNO055_EULER_H_LSB 0x1A    // Начало данных углов Эйлера (Heading, Roll, Pitch)

uint8_t bno_txBuffer[2];                  // Буфер передачи
uint8_t bno_rxBuffer[6];                  // Буфер приема
volatile uint8_t i2cTransferComplete = 0; // Флаг завершения операции

//****************************************** ОПРЕДЕЛЕНИЕ ФУНКЦИЙ ***********************************

HAL_StatusTypeDef BNO055_Read(uint8_t reg, uint8_t *buffer, uint16_t size); // Функция для чтения из регистра BNO055
HAL_StatusTypeDef BNO055_Write(uint8_t reg, uint8_t value);                 // Функция для записи в регистр BNO055

void BNO055_Read_IT(uint8_t reg, uint8_t *buffer, uint16_t size); // Функция для чтения данных из BNO055 используя прерывание
void BNO055_Write_IT(uint8_t reg, uint8_t value);                 // Функция для записи данных в BNO055 используя прерывание

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
void BNO055_Init();

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
    // // Запуск передачи данных
    // if (HAL_I2C_Master_Transmit_IT(&hi2c1, I2C_ADDRESS, txBuffer, sizeof(txBuffer)) == HAL_OK)
    // {
    //     // Успешно начата передача, ждем завершения через прерывание
    //     HAL_Delay(100);
    // }
    // else
    // {
    //     // Если шина занята или ошибка
    //     printf("I2C Transmission Failed!\n");
    // }
    uint8_t chip_id = 0;
    // Проверяем идентификатор чипа
    if (BNO055_Read(BNO055_CHIP_ID, &chip_id, 1) == HAL_OK && chip_id == 0xA0)
    {
        printf("BNO055 1 ver detected!\n");
    }
    else
    {
        printf("BNO055 not found!\n");
        while (1)
            ;
    }
    // Проверяем идентификатор чипа
    BNO055_Read_IT(BNO055_CHIP_ID, &chip_id, 1);
    while (!i2cTransferComplete)
        ;
    if (chip_id == 0xA0)
    {
        DEBUG_PRINTF("BNO055 detected!\n");
    }
    else
    {
        DEBUG_PRINTF("BNO055 not found!\n");
        while (1)
            ;
    }
}

#endif