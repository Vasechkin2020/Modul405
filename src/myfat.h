#ifndef MYFAT_H
#define MYFAT_H

#include "main.h"   // Подключение заголовочного файла для работы с основными функциями микроконтроллера
#include "sdio.h"   // Подключение заголовочного файла для работы с SDIO интерфейсом
#include <string.h> // Подключение заголовочного файла для работы со строками
#include "..\lib\FATFS\fatfs.h"

FRESULT res;
FATFS SDFatFs;
FIL MyFile;
UINT bytesWritten, bytesRead;
FSIZE_t fileSize;

char writeBuffer[128]; // Буфер для записи данных (достаточно для 128 байт)
char readBuffer[128];  // Буфер для чтения данных

//********************* ОБЬЯВЛЕНИЕ  ******************************
void mountFilesystem();   // Функция для монтирования файловой системы
void unmountFilesystem(); // Функция для демонтирования файловой системы

FRESULT writeUint8ToFile(uint8_t *values, uint8_t size, const char *filename);  // Функция записи массива uint8_t в файл
FRESULT writeFloatToFile(float *values, uint8_t size, const char *filename);    // Функция записи массива float в файл
FRESULT readUint8FromFile(uint8_t *values, uint8_t size, const char *filename); // Функция считывания uint8_t из файла в массив
FRESULT readFloatFromFile(float *values, uint8_t size, const char *filename);   // Функция считывания float из файла в массив

// void saveLaserCfg();                                                                // Запись поправочных знаяений для лазерных датчиков
// void saveByte();
//********************* РЕАЛИЗАЦИЯ ******************************
// Функция для монтирования файловой системы
void mountFilesystem()
{
    res = f_mount(&SDFatFs, "", 1); // Монтирование файловой системы с отладкой
    if (res != FR_OK)
        printf("f_mount Failed to mount filesystem (error: %d)\r\n", res);
    else
        printf("f_mount Filesystem mounted successfully.\r\n");

    // Проверка состояния диска и режима доступа
    DWORD freeClusters, totalClusters, freeSectors;
    FATFS *fs;
    BYTE work[_MAX_SS]; // Буфер для проверки
    res = f_getfree("", &freeClusters, &fs);
    if (res != FR_OK)
        printf("f_getfree Failed to get free space (error: %d)\r\n", res);
    else
    {
        totalClusters = (fs->n_fatent - 2);     // Общее число кластеров
        freeSectors = freeClusters * fs->csize; // Свободное место в секторах
        printf("f_getfree Free clusters: %lu, Total clusters: %lu, Free sectors: %lu\r\n", freeClusters, totalClusters, freeSectors);

        // Проверка, смонтировано ли как "только для чтения"
        if (fs->fs_type & 0x80)
            printf("f_getfree Filesystem mounted as read-only!\r\n");
        else
            printf("f_getfree Filesystem mounted with read/write access.\r\n");
    }

    // Проверка состояния диска через disk_ioctl
    DSTATUS stat = disk_ioctl(0, GET_SECTOR_COUNT, work);
    if (stat & STA_NOINIT)
        printf("disk_ioctl Disk not initialized!\r\n");
    else if (stat & STA_NODISK)
        printf("disk_ioctl No disk present!\r\n");
    else
        printf("disk_ioctl Disk status: OK\r\n");
    //**-----------------------
    get_fattime(); // Установка текущего времени в формате FatFs
    //===============================
}

// Функция для демонтирования файловой системы
void unmountFilesystem()
{
    f_mount(NULL, "", 0);                // Демонтирование файловой системы
    printf("Filesystem unmounted.\r\n"); // Вывод сообщения об успешном демонтировании
}

// Функция записи массива uint8_t в файл
FRESULT writeUint8ToFile(uint8_t *values, uint8_t size, const char *filename)
{
    // Форматирование значений uint8_t в строку с разделителями
    char tempBuffer[8];
    writeBuffer[0] = '\0';
    for (int i = 0; i < size; i++)
    {
        snprintf(tempBuffer, sizeof(tempBuffer), "%u", values[i]);
        strcat(writeBuffer, tempBuffer);
        if (i < size - 1)
        {
            strcat(writeBuffer, ";");
        }
    }
    printf("Writing to %s: %s\r\n", filename, writeBuffer);

    printf("Opening file %s for writing... | ", filename);
    res = f_open(&MyFile, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK)
    {
        printf("Failed to open file %s for writing (error: %d)\r\n", filename, res);
        return res;
    }
    printf("File %s opened successfully.\r\n", filename);

    res = f_write(&MyFile, writeBuffer, strlen(writeBuffer), &bytesWritten);
    if (res != FR_OK || bytesWritten != strlen(writeBuffer))
    {
        printf("Failed to write to %s (error: %d, bytes written: %d)\r\n", filename, res, bytesWritten);
    }
    else
    {
        printf("Successfully wrote %d bytes to %s\r\n", bytesWritten, filename);
        res = f_sync(&MyFile);
        if (res != FR_OK)
        {
            printf("f_sync failed with error: %d\r\n", res);
        }
        else
        {
            printf("File synchronized.\r\n");
        }
        FSIZE_t fileSize = f_size(&MyFile);
        printf("File size after write: %lu bytes\r\n", fileSize);
    }
    f_close(&MyFile);
    return res;
}

// Функция записи массива float в файл
FRESULT writeFloatToFile(float *values, uint8_t size, const char *filename)
{
    // Форматирование значений float в строку с разделителями
    char tempBuffer[16];
    writeBuffer[0] = '\0';
    for (int i = 0; i < size; i++)
    {
        snprintf(tempBuffer, sizeof(tempBuffer), "%.4f", values[i]);
        strcat(writeBuffer, tempBuffer);
        if (i < size - 1)
        {
            strcat(writeBuffer, ";");
        }
    }
    printf("Writing to %s: %s\r\n", filename, writeBuffer);

    printf("Opening file %s for writing... | ", filename);
    res = f_open(&MyFile, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK)
    {
        printf("Failed to open file %s for writing (error: %d)\r\n", filename, res);
        return res;
    }
    printf("File %s opened successfully.\r\n", filename);

    res = f_write(&MyFile, writeBuffer, strlen(writeBuffer), &bytesWritten);
    if (res != FR_OK || bytesWritten != strlen(writeBuffer))
    {
        printf("Failed to write to %s (error: %d, bytes written: %d)\r\n", filename, res, bytesWritten);
    }
    else
    {
        printf("Successfully wrote %d bytes to %s\r\n", bytesWritten, filename);
        res = f_sync(&MyFile);
        if (res != FR_OK)
        {
            printf("f_sync failed with error: %d\r\n", res);
        }
        else
        {
            printf("File synchronized.\r\n");
        }
        FSIZE_t fileSize = f_size(&MyFile);
        printf("File size after write: %lu bytes\r\n", fileSize);
    }
    f_close(&MyFile);
    return res;
}

// Функция считывания uint8_t из файла в массив
FRESULT readUint8FromFile(uint8_t *values, uint8_t size, const char *filename)
{
    printf("Opening file %s for reading... | ", filename);
    res = f_open(&MyFile, filename, FA_READ);
    if (res != FR_OK)
    {
        printf("Failed to open file for reading (error: %d)\r\n", res);
        return res;
    }
    printf("File %s opened for reading.\r\n", filename);

    res = f_read(&MyFile, readBuffer, sizeof(readBuffer) - 1, &bytesRead);
    if (res != FR_OK || bytesRead == 0)
    {
        printf("Failed to read from file (error: %d, bytes read: %d)\r\n", res, bytesRead);
    }
    else
    {
        readBuffer[bytesRead] = '\0';
        printf("Read %d bytes from %s: %s\r\n", bytesRead, filename, readBuffer);

        uint8_t readValues[128];
        char *token = strtok(readBuffer, ";");
        int i = 0;
        while (token != NULL && i < size)
        {
            readValues[i] = (uint8_t)atoi(token);
            token = strtok(NULL, ";");
            i++;
        }
        // Копируем считанные значения в выходной массив
        for (int j = 0; j < size && j < i; j++)
        {
            values[j] = readValues[j];
        }
        printf("Parsed values: ");
        for (int j = 0; j < size; j++)
        {
            printf("%u", values[j]);
            if (j < size - 1)
            {
                printf("; ");
            }
        }
        printf("\r\n");
    }
    f_close(&MyFile);
    return res;
}

// Функция считывания float из файла в массив
FRESULT readFloatFromFile(float *values, uint8_t size, const char *filename)
{
    printf("Opening file %s for reading... | ", filename);
    res = f_open(&MyFile, filename, FA_READ);
    if (res != FR_OK)
    {
        printf("Failed to open file for reading (error: %d)\r\n", res);
        return res;
    }
    printf("File %s opened for reading.\r\n", filename);

    res = f_read(&MyFile, readBuffer, sizeof(readBuffer) - 1, &bytesRead);
    if (res != FR_OK || bytesRead == 0)
    {
        printf("Failed to read from file (error: %d, bytes read: %d)\r\n", res, bytesRead);
    }
    else
    {
        readBuffer[bytesRead] = '\0';
        printf("Read %d bytes from %s: %s\r\n", bytesRead, filename, readBuffer);

        float readValues[128];
        char *token = strtok(readBuffer, ";");
        int i = 0;
        while (token != NULL && i < size)
        {
            readValues[i] = (float)atof(token);
            token = strtok(NULL, ";");
            i++;
        }
        // Копируем считанные значения в выходной массив
        for (int j = 0; j < size && j < i; j++)
        {
            values[j] = readValues[j];
        }
        printf("Parsed values: ");
        for (int j = 0; j < size; j++)
        {
            printf("%.4f", values[j]);
            if (j < size - 1)
            {
                printf("; ");
            }
        }
        printf("\r\n");
    }
    f_close(&MyFile);
    return res;
}

/*

void saveLaserCfg() // Запись поправочных значений для лазерных датчиков
{
    float values[4] = {1.23f, 4.56f, 7.89f, 10.11f}; // Массив с 4 значениями типа float для записи

    snprintf(writeBuffer, sizeof(writeBuffer), "%.2f;%.2f;%.2f;%.2f", values[0], values[1], values[2], values[3]); // Форматирование значений в строку с разделителями
    printf("Writing to laser.cfg: %s\r\n", writeBuffer);                                                           // Вывод строки, которая будет записана

    printf("Opening file laser.cfg for writing... | ");              // Вывод сообщения о начале открытия файла для записи
    res = f_open(&MyFile, "laser.cfg", FA_CREATE_ALWAYS | FA_WRITE); // Открытие файла (создание или перезапись)
    if (res != FR_OK)                                                // Проверка успешности открытия
    {
        printf("Failed to open file laser.cfg for writing (error: %d)\r\n", res); // Вывод ошибки открытия
        return;                                                                   // Выход из функции при ошибке
    }
    printf("File laser.cfg opened successfully.\r\n"); // Вывод сообщения об успешном открытии

    res = f_write(&MyFile, writeBuffer, strlen(writeBuffer), &bytesWritten); // Запись данных в файл
    if (res != FR_OK || bytesWritten != strlen(writeBuffer))                 // Проверка успешности записи
    {
        printf("Failed to write to laser.cfg (error: %d, bytes written: %d)\r\n", res, bytesWritten); // Вывод ошибки записи
    }
    else // Если запись успешна
    {
        printf("Successfully wrote %d bytes to laser.cfg\r\n", bytesWritten); // Вывод количества записанных байт
        res = f_sync(&MyFile);                                                // Синхронизация данных на карту
        if (res != FR_OK)                                                     // Проверка успешности синхронизации
        {
            printf("f_sync failed with error: %d\r\n", res); // Вывод ошибки синхронизации
        }
        else // Если синхронизация успешна
        {
            printf("File synchronized.\r\n"); // Вывод сообщения об успешной синхронизации
        }
        FSIZE_t fileSize = f_size(&MyFile);                       // Получение размера файла
        printf("File size after write: %lu bytes\r\n", fileSize); // Вывод размера файла
    }
    f_close(&MyFile); // Закрытие файла

    printf("Opening file laser.cfg for reading... | "); // Вывод сообщения о начале открытия файла для чтения
    res = f_open(&MyFile, "laser.cfg", FA_READ);        // Открытие файла для чтения
    if (res != FR_OK)                                   // Проверка успешности открытия
    {
        printf("Failed to open file for reading (error: %d)\r\n", res); // Вывод ошибки открытия
        return;                                                         // Выход из функции при ошибке
    }
    printf("File laser.cfg opened for reading.\r\n"); // Вывод сообщения об успешном открытии

    res = f_read(&MyFile, readBuffer, sizeof(readBuffer) - 1, &bytesRead); // Чтение данных из файла
    if (res != FR_OK || bytesRead == 0)                                    // Проверка успешности чтения
    {
        printf("Failed to read from file (error: %d, bytes read: %d)\r\n", res, bytesRead); // Вывод ошибки чтения
    }
    else // Если чтение успешно
    {
        readBuffer[bytesRead] = '\0'; // Завершение строки нулевым байтом

        char *pos;
        if ((pos = strchr(readBuffer, '\n')) != NULL) // Удаляем символы перевода строки и возврата каретки
            *pos = '\0';
        if ((pos = strchr(readBuffer, '\r')) != NULL)
            *pos = '\0';
        printf("Read %d bytes from laser.cfg: %s\r\n", bytesRead, readBuffer); // Вывод прочитанных данных

        float readValues[4]; // Массив для хранения считанных значений
                             // sscanf(readBuffer, "%f;%f;%f;%f", &readValues[0], &readValues[1], &readValues[2], &readValues[3]); // Парсинг строки
        char *token = strtok(readBuffer, ";");
        int i = 0;
        while (token != NULL && i < 4)
        {
            readValues[i] = atof(token); // atof — преобразует строку в float
            i++;
            token = strtok(NULL, ";");
        }

        if (i != 4)
        {
            printf("Ошибка парсинга строки!\n");
            printf("Read %d bytes from laser.cfg: ", bytesRead);
            for (int i = 0; i < bytesRead; i++)
            {
                printf("%c", readBuffer[i] == '\n' ? '\\' : readBuffer[i]);
            }
            printf("\n");

            printf("Buffer hex dump: ");
            for (int i = 0; i < bytesRead && i < 30; i++)
            {
                printf("%02X ", (unsigned char)readBuffer[i]);
            }
            printf("\n");
        }
        else
            printf("Parsed values: %.2f; %.2f; %.2f; %.2f\r\n", readValues[0], readValues[1], readValues[2], readValues[3]); // Вывод парсенных значений
    }
    f_close(&MyFile); // Закрытие файла
}


void saveByte() // Функция для записи 1 байта в файл на SD карту
{
    char writeBuffer[16] = "1;2;3;4;5;6;7;8"; // Данные для записи
    char readBuffer[16];                      // Буфер для чтения

    f_open(&MyFile, "1.txt", FA_READ);
    fileSize = f_size(&MyFile);
    printf("f_open +++ 1 File size 1 txt: %lu bytes\r\n", fileSize);
    f_close(&MyFile);

    // Открытие файла для добавления 1 байта
    res = f_open(&MyFile, "1.txt", FA_OPEN_APPEND | FA_WRITE);
    if (res != FR_OK)
        printf("f_open Failed to open file for appending (error: %d)\r\n", res);
    else
        printf("f_open opened successfully.\r\n");

    // Запись 1 байта
    res = f_write(&MyFile, writeBuffer, 1, &bytesWritten);
    if (res != FR_OK || bytesWritten != 1)
        printf("f_write Failed to write to BUFFER (error: %d, bytes written: %d)\r\n", res, bytesWritten);
    else
    {
        printf("f_write Successfully wrote %d byte to BUFFER\r\n", bytesWritten);
        res = f_sync(&MyFile); // Синхронизация данных

        uint32_t sdError44 = HAL_SD_GetError(&hsd); // Получаем код ошибки SDIO
        uint32_t sdError42 = hsd.ErrorCode;

        printf("Error f_sync (0x%08lX)  = %lu hsd.ErrorCode= %lu \r\n", sdError44, sdError44, sdError42);

        if (res == FR_OK)
            printf("f_sync: OK. Data safely written to SD card.\r\n");
        else
        {
            printf("f_sync failed with error: %d\r\n", res);
            switch (res) // Дополнительно: расшифровка ошибки
            {
            case FR_DISK_ERR:
                printf("Error: Low-level disk I/O error.\r\n");
                break;
            case FR_INT_ERR:
                printf("Error: Assertion failed or internal FATFS error.\r\n");
                break;
            case FR_NOT_READY:
                printf("Error: SD card not ready (e.g. removed).\r\n");
                break;
            case FR_WRITE_PROTECTED:
                printf("Error: Card is write-protected.\r\n");
                break;
            case FR_DENIED:
                printf("Error: Access denied (e.g. read-only file or no write permission).\r\n");
                break;
            case FR_INVALID_OBJECT:
                printf("Error: Invalid file object (e.g. file not opened).\r\n");
                break;
            case FR_TIMEOUT:
                printf("Error: Operation timed out.\r\n");
                break;
            default:
                printf("Error: Unknown error code.\r\n");
                break;
            }
        }
    }

    f_close(&MyFile); // Закрытие файла
}
*/
#endif