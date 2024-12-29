#ifndef LASER80_H
#define LASER80_H

enum codeOperation codeOperationUART1;
enum codeOperation codeOperationUART2;
enum codeOperation codeOperationUART3;
enum codeOperation codeOperationUART4;

#define RX_BUFFER_SIZE 64                     // Размер буфера приема БУФЕР БОЛЬШЕГО РАЗМЕРА ЧТОБЫ НЕ СРАБАТЫВАЛИ ПРЕРЫВАНИЯ НА СЕРЕДИНУ БУФЕРА !!!!!!!!!!!!!!!!!!!!!!!!!!!! МИнимум в 2 раза больше данных иначе обрабатывать середину буфера
uint8_t rx_bufferLaser0[RX_BUFFER_SIZE] = {0}; // Буфер для приема данных
uint8_t rx_bufferLaser1[RX_BUFFER_SIZE] = {0}; // Буфер для приема данных
uint8_t rx_bufferLaser2[RX_BUFFER_SIZE] = {0}; // Буфер для приема данных
uint8_t rx_bufferLaser3[RX_BUFFER_SIZE] = {0}; // Буфер для приема данных

extern bool flagCallBackUart; // Флаг для указания нужно ли отрабатывать в колбеке  или обраьотка с самой функции
extern HAL_StatusTypeDef status;
extern uint8_t lenDataLaser; // Длинна полученных данных в буфере
extern uint32_t millis();    // Длинна полученных данных в буфере
extern struct dataUART dataUART[4];

uint8_t bufRead11[11]; // Буфер для чтения // Буфер для приема одиночного сообщения без цикла, а после небольшой задержки
//     //*********************** ОБЬЯВЛЕНИЕ ФУНКЦИЙ *****************************************

void laser80_Init();
uint8_t WaitForFlagOrTimeout(uint16_t port_, uint32_t timeout_ms); // Функция задержки до взведения флага или таймаута
uint8_t lazer80_calcCs(uint8_t *data_, uint8_t len_);              // Расчет контрольной суммы. Берется массив всех оправляемых данных без последнего байта и суммируется побайтно, потом в бинарном виде инвертируются 1 в нолик и нолик в единицу и потом прибавляется 1
//     // Функции применимык к конкретному датчику, так как задается адрес датчика

uint32_t laser80_calcDistance(uint8_t *rx_bufferUART, u_int8_t len_); // Вычисление дистанции по полученному буферу или ошибка
void laser80_controlLaser(uint16_t port_, uint8_t status_);           // Управление лазером 1- Включен 0-Выключен

// void laser80_singleMeasurement(UART_HandleTypeDef *huart, uint8_t addr_);                              // Одиночное измерение примерно 800 милисекунд
void laser80_singleMeasurement(uint8_t port_);      // Одиночное измерение примерно 800 милисекунд
void laser80_continuousMeasurement(uint16_t port_); // Непрерывное измерение
void laser80_stopMeasurement(uint8_t port_);        // Прекратить измерение
// void laser80_stopMeasurement(UART_HandleTypeDef *huart, uint8_t *rx_bufferUART_);       // Прекратить измерение

void laser80_setAddress(UART_HandleTypeDef *huart, uint8_t addr_); // Установка нового адрес на датчике
void laser80_broadcastMeasurement(UART_HandleTypeDef *huart);      // Единое измерние. Команда всем подключенным датчикам произвести измерение.Потом его надо считать с каждого датчика
bool laser80_getCache(UART_HandleTypeDef *huart, uint8_t addr_);   // Считывание данных из буфера датчика результат измерения

void laser80_setFrequency(uint16_t port_, uint8_t freq_);     // Установка частоты измерений, задается в герцах 3,5,10,20 только такие частоты
void laser80_setStartingPoint(uint16_t port_, uint8_t data_); // Установка точки откоторой считем расстояние. 1- от носа 0 - от зада
void laser80_setTimeInterval(uint16_t port_, uint8_t data_);  // Установка инрервала вывода значения при настройке. Не понятно что это.
void laser80_setRange(uint16_t port_, uint8_t range_);        // Установление максимального диапзона измерений. Возможно 5,10,30,50,80 метров
void laser80_setResolution(u_int16_t port_, uint8_t reso_);   // Установка разрешения измерения есди 1- то 1 мм, если 2 то 0,1 мм. Непонятно работает ли нет фактически и на чем сказывается (время измерения?)

//     // Функции применяются ко всем датчикам на линии и поэтому калибровку и прочее делать с одиночным датчиком
//     bool setDistanceModification(int8_t data_); // Модификация дистанции. Думаю что колибровка измерений. Можно в плюс или в минус
//     bool setTestPowerOn(uint8_t data_);            // Установка нужно ли проводить тест датчика после включения питания. Значение 0 или 1
//

//     //*********************** РЕАЛИЗАЦИЯ ФУНКЦИЙ *****************************************

// Прекратить измерение
// void laser80_stopMeasurement(UART_HandleTypeDef *huart, uint8_t *rx_bufferUART_)
void laser80_stopMeasurement(uint8_t port_)
{
    DEBUG_PRINTF("stopMeasurement ");
    static uint8_t buf[4] = {0x80, 0x04, 0x02, 0x7A};
    buf[3] = lazer80_calcCs(buf, 4);
    HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100);
    HAL_Delay(10); // Задержка

    DEBUG_PRINTF(" DATA => %X %X %X %X ", dataUART[port_].adr[0], dataUART[port_].adr[1], dataUART[port_].adr[2], dataUART[port_].adr[3]);
    if (dataUART[port_].adr[0] == 0x80 && dataUART[port_].adr[1] == 0x04 && dataUART[port_].adr[2] == 0x82 && dataUART[port_].adr[3] == 0xFA)
    {
        DEBUG_PRINTF(" ok \r\n");
    }
    else
    {
        DEBUG_PRINTF(" ERROR \r\n");
    }
    memset(dataUART[port_].adr, 0, RX_BUFFER_SIZE); // Очистка буфера
    // DEBUG_PRINTF("\r\n");
}

// Непрерывное измерение
void laser80_continuousMeasurement(uint16_t port_)
{
    DEBUG_PRINTF("continuousMeasurement ");
    static uint8_t buf[4] = {0x80, 0x06, 0x03, 0x00};
    buf[3] = lazer80_calcCs(buf, 4);
    HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100);
}

// //************************************************************************************
// Установка нового адрес на датчике Широковещательный команда, смотреть что датчик один в этот момент на шине.
void laser80_setAddress(UART_HandleTypeDef *huart, uint8_t addr_)
{
    flagCallBackUart = false; // Эту функцию Не нужно отрабатывать в колбеке
    uint8_t buf[5] = {0xFA, 0x04, 0x01, addr_, 0x00};
    buf[4] = lazer80_calcCs(buf, 5);
    HAL_UART_Transmit(huart, buf, sizeof(buf), 100);
    HAL_Delay(50); // Задержка на время ожидания ответа и сразу разбираем ответ который будет в буфере который указали для ожмдания прерывания по DMA
    // DEBUG_PRINTF("setAddress DATA => %X %X %X %X %X\n", rx_bufferLaser0[0], rx_bufferLaser0[1], rx_bufferLaser0[2], rx_bufferLaser0[3], rx_bufferLaser0[4]);
    if (rx_bufferLaser0[0] == 0xFA && rx_bufferLaser0[1] == 0x04 && rx_bufferLaser0[2] == 0x81 && rx_bufferLaser0[3] == 0x81)
    {
        // DEBUG_PRINTF("setAddress ok \n");
        // return true;
    }
    else
    {
        // DEBUG_PRINTF("setAddress ERROR\n");
        // return false;
    }
}
// Одиночное измерение примерно 800 милисекунд
// void laser80_singleMeasurement(UART_HandleTypeDef *huart, uint8_t addr_)
void laser80_singleMeasurement(uint8_t port_)
{
    DEBUG_PRINTF("singleMeasurement ");
    static uint8_t buf[4] = {0x80, 0x06, 0x02, 0x78};
    buf[3] = lazer80_calcCs(buf, 4);
    HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100);
    HAL_Delay(1000); // Задержка
    DEBUG_PRINTF(" DATA => %X %X %X %X ", dataUART[port_].adr[0], dataUART[port_].adr[1], dataUART[port_].adr[2], dataUART[port_].adr[3]);
    for (int i = 0; i < dataUART[port_].len; i++)
    {
        DEBUG_PRINTF("%X-", dataUART[port_].adr[i]);
    }
    DEBUG_PRINTF("\r\n");
    //************************
    // // digitalWrite(PIN_LED, 1);
    // // clearBuf();
    // uint8_t buf[4] = {addr_, 0x06, 0x02, 0x00}; // Команда без последнего байта, там будет контрольная сумма, а пока 0х00
    // buf[3] = lazer80_calcCs(buf, 4);
    // // Serial2.write(buf, 4);
    // HAL_UART_Transmit(huart, buf, sizeof(buf), 100);
    // HAL_Delay(5); // Задержка что точно исполниться и не испортися чем-то другим
    // codeOperationUART1 = Single;
    //************************

    // HAL_delay(850);
    uint8_t len = 11;
    uint8_t bufRead[len];
    for (int i = 0; i < len; i++)
    {
        bufRead[i] = dataUART[port_].adr[i];
        DEBUG_PRINTF("%X-", bufRead[i]);
    }
    DEBUG_PRINTF("\n");
    uint8_t sot = (bufRead[3] - 0x30) * 100;       // По таблице ASCII отнимаем 48 и получаем сколько сотен метров
    uint8_t des = (bufRead[4] - 0x30) * 10;        // По таблице ASCII отнимаем 48 и получаем сколько десятков метров
    uint8_t met = (bufRead[5] - 0x30) * 1;         // По таблице ASCII отнимаем 48 и получаем сколько единиц метров
    float desMet = (bufRead[7] - 0x30) * 0.1;   // По таблице ASCII отнимаем 48 и получаем сколько десятых долей метра
    float sotMet = (bufRead[8] - 0x30) * 0.01;  // По таблице ASCII отнимаем 48 и получаем сколько сотых долей метра
    float tysMet = (bufRead[9] - 0x30) * 0.001; // По таблице ASCII отнимаем 48 и получаем сколько тысячных долей метра
    float distance = sot + des + met + desMet + sotMet + tysMet;

    DEBUG_PRINTF("Meas= %i - %i - %i . %.1f %.2f %.3f | ", sot, des, met, desMet, sotMet, tysMet);
    DEBUG_PRINTF("Distance= %f \n", distance);
    // if (bufRead[0] == addr_ && buf[1] == 0x06 && buf[2] == 0x82 && buf[3] == 0x45 && buf[4] == 0x52 && buf[4] == 0x52) // Проверка на ошибку. Возвращает ADDR 06 82"'E' 'R' 'R' '-' '-' '3X' '3X' ”CS
    // {
    //     DEBUG_PRINTF("readMeasurement ERROR \n");
    //     return false;
    // }
}

// Вычисление дистанции по полученному буферу или ошибка
uint32_t laser80_calcDistance(uint8_t *rx_bufferUART, u_int8_t len_)
{
    float distance = 0;
    uint32_t dist;
    if (len_ == 11 && rx_bufferUART[3] != 0x45 && rx_bufferUART[4] != 0x52 && rx_bufferUART[5] != 0x52) // по кодам ASCII ERR
    {
        uint8_t sot = (rx_bufferUART[3] - 0x30) * 100;    // По таблице ASCII отнимаем 48 и получаем сколько сотен метров
        uint8_t des = (rx_bufferUART[4] - 0x30) * 10;     // По таблице ASCII отнимаем 48 и получаем сколько десятков метров
        uint8_t met = (rx_bufferUART[5] - 0x30) * 1;      // По таблице ASCII отнимаем 48 и получаем сколько единиц метров
        float desMet = (rx_bufferUART[7] - 0x30) * 0.1;   // По таблице ASCII отнимаем 48 и получаем сколько десятых долей метра
        float sotMet = (rx_bufferUART[8] - 0x30) * 0.01;  // По таблице ASCII отнимаем 48 и получаем сколько сотых долей метра
        float tysMet = (rx_bufferUART[9] - 0x30) * 0.001; // По таблице ASCII отнимаем 48 и получаем сколько тысячных долей метра
        distance = sot + des + met + desMet + sotMet + tysMet;
        // DEBUG_PRINTF("Meas= %i - %i - %i . %.1f %.2f %.3f | ", sot, des, met, desMet, sotMet, tysMet);
        //  DEBUG_PRINTF("Distance= %.3f \n", distance);
    }
    dist = distance * 1000;
    // DEBUG_PRINTF("Dist= %lu \n", dist);
    return dist;
}

// Единое измерние. Команда всем подключенным датчикам произвести измерение.Потом его надо считать с каждого датчика
void laser80_broadcastMeasurement(UART_HandleTypeDef *huart)
{
    uint8_t buf[4] = {0xFA, 0x06, 0x06, 0xFA};
    // Serial2.write(buf, 4);
    HAL_UART_Transmit(huart, buf, sizeof(buf), 100);
    HAL_Delay(10); // Задержка что точно исполниться и не испортися чем-то другим
    codeOperationUART1 = Broadcast;
}
// Считывание данных из буфера датчика результат измерения
bool laser80_getCache(UART_HandleTypeDef *huart, uint8_t addr_)
{
    // clearBuf();
    uint8_t buf[4] = {addr_, 0x06, 0x07, 0x00};
    buf[3] = lazer80_calcCs(buf, 4); // Считаем контрольную сумму и записываем последним байтом
                                     // Serial2.write(buf, 4);
    HAL_UART_Transmit(huart, buf, sizeof(buf), 100);
    HAL_Delay(1); // Задержка что точно исполниться и не испортися чем-то другим
    codeOperationUART1 = Cache;

    // delay(20);
    // uint8_t len = 11;
    // uint8_t bufRead[len];
    // for (int i = 0; i < len; i++)
    // {
    //     bufRead[i] = Serial2.read();
    //     DEBUG_PRINTF("%X-", bufRead[i]);
    // }

    // if (bufRead[0] == addr_ && buf[1] == 0x06 && buf[2] == 0x82 && buf[3] == 0x45 && buf[4] == 0x52 && buf[4] == 0x52) // Проверка на ошибку. Возвращает ADDR 06 82"'E' 'R' 'R' '-' '-' '3X' '3X' ”CS
    // {
    //     DEBUG_PRINTF("readMeasurement ERROR \n");
    //     return false;
    // }
    // String rez = bufRead[3] + bufRead[4] + bufRead[5] + "." + bufRead[6] + bufRead[6] + bufRead[8];

    // DEBUG_PRINTF(" rez = %s \n", rez.c_str());
    // Serial.println("rezultat=" + rez);

    return true;
}
// Функция задержки до взведения флага или таймаута
uint8_t WaitForFlagOrTimeout(uint16_t port_, uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick(); // Текущее значение системного счетчика
    while (1)
    {
        if (dataUART[port_].flag)
            return 1; // Флаг взведен, возвращаем успех
        if ((HAL_GetTick() - start_tick) >= timeout_ms)
            return 0; // Тайм-аут, возвращаем ошибку
    }
}

// Управление лазером 1- Включен 0-Выключен
void laser80_controlLaser(uint16_t port_, uint8_t status_)
{
    DEBUG_PRINTF("controlLaser -> ");
    if (status_ == 0) //  Выключить
    {
        uint8_t buf[5] = {0x80, 0x06, 0x05, 0x00, 0x00};
        buf[4] = lazer80_calcCs(buf, 5);
        HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100);
    }
    else //  Включить
    {
        uint8_t buf[5] = {0x80, 0x06, 0x05, 0x01, 0x00};
        buf[4] = lazer80_calcCs(buf, 5);
        HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100);
    }
    HAL_Delay(50); // Задержка
    // WaitForFlagOrTimeout(port_, 200);
    dataUART[port_].flag = 0; // Убираем флаг

    DEBUG_PRINTF(" DATA => %X %X %X %X ", dataUART[port_].adr[0], dataUART[port_].adr[1], dataUART[port_].adr[2], dataUART[port_].adr[3]);
    if (dataUART[port_].adr[0] == 0x80 && dataUART[port_].adr[1] == 0x06 && dataUART[port_].adr[2] == 0x85 && dataUART[port_].adr[3] == 0x01)
        DEBUG_PRINTF(" ok \r\n");
    else
        DEBUG_PRINTF(" ERROR \r\n");
    memset(dataUART[port_].adr, 0, RX_BUFFER_SIZE); // Очистка буфера
    // DEBUG_PRINTF("\r\n");
}
// Установка интервала вывода значения при настройке. Не понятно что это.
void laser80_setTimeInterval(uint16_t port_, uint8_t data_)
{
    DEBUG_PRINTF("setTimeInterval -> ");

    if (data_ == 0) // через 0 секунду
    {
        uint8_t buf[5] = {0xFA, 0x04, 0x05, 0x00, 0xFD};
        HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100);
    }
    if (data_ == 1) // через 1 секунду
    {
        uint8_t buf[5] = {0xFA, 0x04, 0x05, 0x01, 0xFC};
        HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100);
    }
    HAL_Delay(50); // Задержка что точно исполниться и не испортися чем-то другим

    DEBUG_PRINTF(" DATA => %X %X %X %X ", dataUART[port_].adr[0], dataUART[port_].adr[1], dataUART[port_].adr[2], dataUART[port_].adr[3]);
    if (dataUART[port_].adr[0] == 0xFA && dataUART[port_].adr[1] == 0x04 && dataUART[port_].adr[2] == 0x85 && dataUART[port_].adr[3] == 0x7D)
        DEBUG_PRINTF(" ok \r\n");
    else
        DEBUG_PRINTF(" ERROR \r\n");
    memset(dataUART[port_].adr, 0, RX_BUFFER_SIZE); // Очистка буфера
    // DEBUG_PRINTF("\r\n");
}

// // Модификация дистанции. Думаю что колибровка измерений. Можно в плюс или в минус
// bool CLaser80M::setDistanceModification(int8_t data_)
// {
//     clearBuf();
//     uint8_t sign;
//     if (data_ > 0) // Если корректировка плюсовая
//     {
//         sign = 0x2b;
//     }
//     else // если минусовая
//     {
//         sign = 0x2d;
//     }
//     uint8_t data = abs(data_);
//     uint8_t buf[6] = {0xFA, 0x04, 0x06, sign, data, 0x00};
//     // uint8_t *bufAddr = (uint8_t *)buf; // Создаем переменную в которую пишем адрес буфера в нужном формате
//     buf[5] = calcCs(buf, 6);
//     Serial2.write(buf, 6);
//     delay(1000);
//     uint8_t bufRead[5];
//     bufRead[0] = Serial2.read();
//     bufRead[1] = Serial2.read();
//     bufRead[2] = Serial2.read();
//     bufRead[3] = Serial2.read();
//     bufRead[4] = Serial2.read();
//     DEBUG_PRINTF("setAddress DATA => %X %X %X %X %X\n", bufRead[0], bufRead[1], bufRead[2], bufRead[3], bufRead[4]);
//     if (bufRead[0] == 0xFA && bufRead[1] == 0x04 && bufRead[2] == 0x8B && bufRead[3] == 0x77)
//     {
//         DEBUG_PRINTF("setDistanceModification ok \n");
//         return true;
//     }
//     else
//     {
//         DEBUG_PRINTF("setDistanceModification ERROR \n");
//         return false;
//     }
// }

// Установка точки откоторой считем расстояние. 1- от носа 0 - от зада
void laser80_setStartingPoint(uint16_t port_, uint8_t data_)
{
    DEBUG_PRINTF("setStartingPoint -> ");
    if (data_ == 0) //
    {
        uint8_t buf[5] = {0xFA, 0x04, 0x08, 0x00, 0xFA};
        HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100);
    }
    if (data_ == 1) //
    {
        uint8_t buf[5] = {0xFA, 0x04, 0x08, 0x01, 0xF9};
        HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100);
    }
    HAL_Delay(25); // Задержка что точно исполниться и не испортися чем-то другим

    DEBUG_PRINTF(" DATA => %X %X %X %X ", dataUART[port_].adr[0], dataUART[port_].adr[1], dataUART[port_].adr[2], dataUART[port_].adr[3]);
    if (dataUART[port_].adr[0] == 0xFA && dataUART[port_].adr[1] == 0x04 && dataUART[port_].adr[2] == 0x88 && dataUART[port_].adr[3] == 0x7A)
        DEBUG_PRINTF(" %i ok \r\n", data_);
    else
        DEBUG_PRINTF(" ERROR \r\n");
    memset(dataUART[port_].adr, 0, RX_BUFFER_SIZE); // Очистка буфера
    // DEBUG_PRINTF("\r\n");
}

// // Установка нужно ли проводить тест датчика после включения питания. Значение 0 или 1
// bool CLaser80M::setTestPowerOn(uint8_t data_)
// {
//     clearBuf();
//     if (data_ == 0) //
//     {
//         uint8_t buf[5] = {0xFA, 0x04, 0x0D, 0x00, 0xF5}; // Нет теста
//         Serial2.write(buf, 5);
//     }
//     if (data_ == 1) //
//     {
//         uint8_t buf[5] = {0xFA, 0x04, 0x0D, 0x01, 0xF4}; // Тестировать
//         Serial2.write(buf, 5);
//     }
//     return true;
// }

// Установление максимального диапзона измерений. Возможно 5,10,30,50,80 метров
void laser80_setRange(u_int16_t port_, uint8_t range_)
{
    DEBUG_PRINTF("setRange -> ");
    if (range_ == 5) //
    {
        uint8_t buf[5] = {0xFA, 0x04, 0x09, 0x05, 0xF4};
        HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100);
    }
    if (range_ == 10) //
    {
        uint8_t buf[5] = {0xFA, 0x04, 0x09, 0x0A, 0xEE};
        HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100);
    }
    if (range_ == 30) //
    {
        uint8_t buf[5] = {0xFA, 0x04, 0x09, 0x1E, 0xDB};
        HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100);
    }
    if (range_ == 50) //
    {
        uint8_t buf[5] = {0xFA, 0x04, 0x09, 0x32, 0xC7};
        HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100);
    }
    if (range_ == 80) //
    {
        uint8_t buf[5] = {0xFA, 0x04, 0x09, 0x50, 0xA9};
        HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100);
    }
    HAL_Delay(50); // Задержка что точно исполниться и не испортися чем-то другим

    DEBUG_PRINTF(" DATA => %X %X %X %X ", dataUART[port_].adr[0], dataUART[port_].adr[1], dataUART[port_].adr[2], dataUART[port_].adr[3]);
    if (dataUART[port_].adr[0] == 0xFA && dataUART[port_].adr[1] == 0x04 && dataUART[port_].adr[2] == 0x89 && dataUART[port_].adr[3] == 0x79)
        DEBUG_PRINTF(" %i ok \r\n", range_);
    else
        DEBUG_PRINTF(" ERROR \r\n");
    memset(dataUART[port_].adr, 0, RX_BUFFER_SIZE); // Очистка буфера
    // DEBUG_PRINTF("\r\n");
}

// Установка разрешения измерения есди 1- то 1 мм, если 2 то 0,1 мм. Непонятно работает ли нет фактически и на чем сказывается (время измерения?)
void laser80_setResolution(u_int16_t port_, uint8_t reso_)
{
    DEBUG_PRINTF("setResolution -> ");
    if (reso_ == 1) //
    {
        uint8_t buf[5] = {0xFA, 0x04, 0x0C, 0x01, 0xF5};
        HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100);
    }
    if (reso_ == 2) //
    {
        uint8_t buf[5] = {0xFA, 0x04, 0x0C, 0x02, 0xF4};
        HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100);
    }
    HAL_Delay(50); // Задержка что точно исполниться и не испортися чем-то другим

    DEBUG_PRINTF(" DATA => %X %X %X %X ", dataUART[port_].adr[0], dataUART[port_].adr[1], dataUART[port_].adr[2], dataUART[port_].adr[3]);
    if (dataUART[port_].adr[0] == 0xFA && dataUART[port_].adr[1] == 0x04 && dataUART[port_].adr[2] == 0x8C && dataUART[port_].adr[3] == 0x76)
        DEBUG_PRINTF(" %i ok \r\n", reso_);
    else
        DEBUG_PRINTF(" ERROR \r\n");
    memset(dataUART[port_].adr, 0, RX_BUFFER_SIZE); // Очистка буфера
    // DEBUG_PRINTF("\r\n");
}

// Установка частоты измерений, задается в герцах 3,5,10,20 только такие частоты
void laser80_setFrequency(uint16_t port_, uint8_t freq_)
{
    DEBUG_PRINTF("setFrequency -> ");
    if (freq_ == 3) //  примерно 3 Hz
    {
        uint8_t buf[5] = {0xFA, 0x04, 0x0A, 0x00, 0xF8};
        HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100);
    }
    if (freq_ == 5) //  фактически 7 Hz
    {
        uint8_t buf[5] = {0xFA, 0x04, 0x0A, 0x05, 0xF3};
        HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100);
    }
    if (freq_ == 10) //  фактически 8 Hz
    {
        uint8_t buf[5] = {0xFA, 0x04, 0x0A, 0x0A, 0xEE};
        HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100);
    }
    if (freq_ == 20) //  фактически 18 Hz
    {
        uint8_t buf[5] = {0xFA, 0x04, 0x0A, 0x14, 0xE4};
        HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100);
    }
    HAL_Delay(50);

    DEBUG_PRINTF(" DATA => %X %X %X %X ", dataUART[port_].adr[0], dataUART[port_].adr[1], dataUART[port_].adr[2], dataUART[port_].adr[3]);
    if (dataUART[port_].adr[0] == 0xFA && dataUART[port_].adr[1] == 0x04 && dataUART[port_].adr[2] == 0x8A && dataUART[port_].adr[3] == 0x78)
        DEBUG_PRINTF(" %i ok \r\n", freq_);
    else
        DEBUG_PRINTF(" ERROR \r\n");
    memset(dataUART[port_].adr, 0, RX_BUFFER_SIZE); // Очистка буфера
    // DEBUG_PRINTF("\r\n");
}

uint8_t lazer80_calcCs(uint8_t *data_, uint8_t len_) // Расчет контрольной суммы. Берется массив всех оправляемых данных без последнего байта и суммируется побайтно, потом в бинарном виде инвертируются 1 в нолик и нолик в единицу и потом прибавляется 1
{
    uint8_t ret;
    uint32_t sum = 0;
    // Serial.println(len_, DEC);
    for (int i = 0; i < (len_ - 1); i++) // Считаем без последнего байта
    {
        sum += data_[i];
        // Serial.print(data_[i], HEX);
        //  Serial.print("-");
    }
    // Serial.println(" |");
    // Serial.print(sum, DEC);
    // Serial.print(" ");
    // Serial.print(sum, HEX);
    // Serial.print(" ");
    // Serial.println(sum, BIN);
    ret = (sum ^ 0b1111111111111111) + 1; // инвертируем биты
    // Serial.print(ret, DEC);
    // Serial.print(" ");
    // Serial.print(ret, HEX);
    // Serial.print(" ");
    // Serial.println(ret, BIN);
    return ret;
}
#endif