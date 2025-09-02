#ifndef SK60PLUS_H
#define SK60PLUS_H

/*
-------------------------------------------------- SK60-plus laser ranging Module User Manual ------------------------------------------------------
http://en.shsenky.com/index.php?c=show&id=35
http://en.shsenky.com/index.php?c=show&id=63
http://en.shsenky.com/index.php?c=category&id=67
Both high frequency 20Hz & high precision 2mm · Suitable for tracking and measuring medium and low speed moving targets

The SK60-plus laser ranging module uses the phase method to detect the distance between the target object by non-contact transmission and reception of the laser. The SK60-plus laser ranging module has been developed for fast and accurate distance measurement. Even in harsh environments, it can still show excellent performance.
The upgrade advantage of SK60-PLUS compared with SK60 mainly lies in the improvement of measurement frequency and measurement time, which makes SK60-Plus have a wider extension in application. And maintain millimeter-level accuracy while maintaining high frequencies.
① The measurement frequency of the SK60-Plus is 20Hz, while the SK60 is 2~3Hz. The high frequency of measurement means more frequent data acquisition and updates, which can provide more accurate real-time measurement results. For example, in the tracking and measurement of medium and low speed moving targets, SK60-plus can provide more accurate position data to track the position changes of targets in real time.
②The measurement time of SK60-Plus is greater than 40ms, and the measurement time of SK60 is greater than 0.3s. The reduction of measurement time can improve measurement efficiency and response speed. By reducing measurement time, the SK100-plus can obtain measurement data faster, which improves the real-time and flexibility of the application. For applications that require quick access to target locations, the SK60-plus can greatly improve task execution efficiency.
③SK60-plus maintains millimeter-level accuracy (2mm) while still maintaining a stable accuracy level under high frequency (20Hz) measurements.
Whether it is in industrial applications where the size of the target needs to be measured with high accuracy, or in logistics tracking applications where the location of the target needs to be measured quickly, the SK60-plus provides accurate measurement results to meet the needs.

Pin Name Function Default Description
1 PWREN Digital Input Low Module power up enable pin, active HIGH
2 TXD Digital Output High Module USART Transmit pin, Open-Drain
3 RXD Digital Input High Module USART Receive pin, Open-Drain
8 VCC/VBAT Power Power Power supply， DC 2.5V~3.3V 300mA+
5 GND Power Ground Module power ground
Application may need more than one laser range finder module to read out each distance. In multi-slave applications, beware to set address for each laser module before it be connected to the network to
avoid the addressing conflict. After factory stage all modules address was set to 0x00 as default.
--
USART Interface
 Baudrate：Auto Detect (9600bps ~115200bps recommend) OR Default 19200bps
 Start bits：1 bit
 Data bits：8 bits
 Stop bits：1 bit
 Parity：none
 Flow control：none
!!!IMPORTANT: Fixed baudrate 19200bps will be used under one condition: 1. Auto baudrate stage time up (~2.5 seconds), no auto baudrate uint8_t 0x55 received or wrong uint8_t received.
In initial state, Slave module (laser rangefinder) is in power down mode before Master pull up the PWREN pin. After PWREN goes high, and if nRST pin. Used please also remember to de-assert the
nRST ping by pull it up, Slave will take about 100 milliseconds to do self-boot, and then entering auto baud rate detect stage.
Master transfer 1uint8_t fixed data 0x55 to slave for auto baud rate, if success, slave will reply 1-uint8_t data to master, which present the slave itself address. In one master and multi-slave communication situation,
the self-address reply from the slaves may cause USART bus conflict, keep in mind this uint8_t should be ignored. Communication between master and slave has been established after the successful auto baud rate. Now
master can send command frame to slave.
*/

/*
// ！！！Note: Do not set slave address to broadcast address 0x7F, this address is reserved for one master to multi-slave network which needs all slave to measure distance
at the same time, and no slave reply measure result until master ask one of them to.
*/

extern HAL_StatusTypeDef status;
extern bool flagCallBackUart; // Флаг для указания нужно ли отрабатывать в колбеке  или обраьотка с самой функции
// extern struct SDataLaser dataLaser[4]; // Структура куда пишем даные из датчиков
uint8_t _addr = 0x00;        // Адрес датчика. Заполняем при инициализации
uint8_t _bufRead[13];        // Буфер для чтения // Буфер для приема одиночного сообщения без цикла, а после небольшой задержки
uint8_t const pinPwrEn = 22; // Пин на котором управляем включением/выключением датчиков

enum statusCode
{
    No_error = 0x0000,
    Power_input_too_low = 0x0001,     //, power voltage should >= 2.2V,
    Internal_error = 0x0002,          // don't care
    Module_temperature_low = 0x0003,  //(< -20℃)
    Module_temperature_high = 0x0004, //(> +40℃)
    Target_out_of_range = 0x0005,     //
    Invalid_measure_result = 0x0006,
    Background_light_too_strong = 0x0007,
    Laser_signal_too_weak = 0x0008,
    Laser_signal_too_strong = 0x0009,
    Hardware_fault_1 = 0x000A,
    Hardware_fault_2 = 0x000B,
    Hardware_fault_3 = 0x000C,
    Hardware_fault_4 = 0x000D,
    Hardware_fault_5 = 0x000E,
    Laser_signal_not_stable = 0x000F,
    Hardware_fault_6 = 0x0010,
    Hardware_fault_7 = 0x0011,
    Invalid_Frame = 0x0081,
};
// Массив строк для представления имен enum
const char *ErrorNames[] = {
    "No_error",
    "Power_input_too_low",
    "Internal_error",
    "Module_temperature_low",
    "Module_temperature_high",
    "Target_out_of_range",
    "Invalid_measure_result",
    "Background_light_too_strong",
    "Laser_signal_too_weak",
    "Laser_signal_too_strong",
    "Hardware_fault_1",
    "Hardware_fault_2",
    "Hardware_fault_3",
    "Hardware_fault_4",
    "Hardware_fault_5",
    "Laser_signal_not_stable",
    "Hardware_fault_6",
    "Hardware_fault_7"};

void clearBuf();      // Очистка буфера приема
bool readBuf();       // Буфер для чтения байт
bool checkingError(); // Проверка и разбор ошибки ответа
                      //-------------------------------------------------- PUBLIC -----------------------------------------------------------

uint32_t _status = 0;        // Статус датчика. Изначально 0 (неопределенный)
uint16_t _hVersion = 0;      // Версия железа. Изначально 0 (неопределенный)
uint16_t _sVersion = 0;      // Версия софта. Изначально 0 (неопределенный)
uint32_t _sNumber = 0;       // Серийный номер. Изначально 0 (неопределенный)
uint32_t _distance = 0;      //
float _angle = 0;            // Угол какой был у маторов в момент старта измерения
uint16_t _signalQuality = 0; //
uint16_t _inputVoltage = 0;  //
uint16_t _offSet = 0;        // Поправочное смещение. Изначально 0 (неопределенный)

void init(uint8_t addr_);
uint8_t calcCs(uint8_t *data_, uint8_t len_);

void sk60plus_autoBaund(); // Установка скорости обмена

void sk60plus_readHardwareVersion(u_int16_t port_);                                   // Function: master read out the module’s HW version number;
void sk60plus_readSoftwareVersion(u_int16_t port_);                                   // Function: master read out the module’s SW version number;
void sk60plus_readSerialNumber(u_int16_t port_);                                      // Function: master read out the module’s serial number;
void sk60plus_readInputVoltage(u_int16_t port_);                                      // Function: master read out the module’s input voltage in mV with BCD encode;
void sk60plus_setModulAddress(UART_HandleTypeDef *huart, uint8_t addr_, uint8_t YY_); // Function: master set slave’s address, this address will not lost after module power off;
void sk60plus_setModulMeasureOffset(UART_HandleTypeDef *huart, int16_t ZZYY_);        // Function: master set slave’s measure offset. For example, if the offset 0xZZYY = 0x7B(+123) , it means the final output of measure result will PLUS 123 millimeters , if the offset 0xZZYY = 0xFF85(-123), it means the final output of measure result will MINUS 123 millimeters
void sk60plus_setLaser(uint16_t port_, uint8_t ZZ_);                                  // Function: turn on or turn off laser beam, if 0xZZ = 0x01 laser on, 0xZZ = 0x00 laser off.

void sk60plus_startSingleAuto(u_int16_t port_); // Function: Reply measure result to master, measure result = 0xAABBCCDD millimeters (frame uint8_t6 = 0xAA, uint8_t7 = 0xBB, uint8_t8 = 0xCC, uint8_t9 = 0xDD) and signal quality = 0x101， lesssignal quality number stands for stronger laser signal and more reliable distance result
void sk60plus_startSingleSlow(u_int16_t port_); // Function: Initiate slave to do 1-shot measure in slow mode.
void sk60plus_startSingleFast(u_int16_t port_); // Function: Initiate slave to do 1-shot measure in fast mode.

void sk60plus_startContinuousAuto(u_int16_t port_);      // Function: Initiate slave to do continuous measure in auto mode
void sk60plus_startContinuousSlow(u_int16_t port_);      // Function: Initiate slave to do continuous measure in slow mode.
void sk60plus_startContinuousFast(u_int16_t port_);      // Function: Initiate slave to do continuous measure in fast mode
void sk60plus_startContinuousSuperFast(u_int16_t port_); // Function: Initiate slave to do continuous measure in Superfast mode 25Hz

void sk60plus_stopContinuous(u_int16_t port_); // Function: Master transfer one uint8_t 0x58 (upper case character ‘X’) to stop continuous measure mode immediately

void sk60plus_getBroadcastSingleMeasure(); // Function: Initiate all slave to do 1-shot measure in auto mode/
void sk60plus_getMeasureResult();          // Function: master read out the distance measure result;
void sk60plus_getStatus();                 // Function: master read out the module’s status after previous command executed;

bool sk60plus_readStatus();        // Function: master read out the module’s status after previous command executed;
bool sk60plus_readMeasureResult(); // Function: master read out the distance measugetesult;

uint32_t laser60_calcDistance(uint8_t *rx_bufferUART);      // Расчет растояния на основе полученных данных
uint16_t laser60_calcSignalQuality(uint8_t *rx_bufferUART); // Расчет качества сигнала на основе полученных данных

/* Master send out 1-shot measure commands to slave address 0x7F, that will make all online slaves to measure distance at the same time， but none of them will return its measure result to master until
master ask each one to return the measure result. Before master send out the reading measure result command, master should read the slave’s status code to make sure there was no error occurred during    this slave measuring.
After sending this command out, master polling each slave address for their status, if slave replies its status code with 0x0000, means no error, then send readMeasureResult() command to read back the
distance. Measure result for each slave will NOT overwrite until next successful measure command with a new distance result.
*/

/* Table 6-2 Registers
    No. Register Name Function
    1 0x0000 REG_ERR_CODE System status code
    2 0x0006 REG_BAT_VLTG Input voltage
    3 0x0010 REG_ADDRESS Module address
    4 0x0012 REG_OFFSET Module measure result offset
    5 0x0020 REG_MEA_START Initiate measure
    6 0x0022 REG_MEA_RESULT Measure result
    7 0x01BE REG_CTRL_LD Laser diode control
*/

void sk60plus_init(uint8_t addr_)
{
    _addr = addr_; // Запоминаем адрес датчика при инициализации
    // setLaser(1);
    // readSerialNumber();
    // readHardwareVersion();
    // readSoftwareVersion();
    // readInputVoltage();
    // setLaser(0);
    // DEBUG_PRINTF("--- \n");
}
// void sk60plus_clearBuf()
// {
//     while (Serial2.available()) // Очищаем буфер
//         Serial2.read();
// }

uint8_t calcCs(uint8_t *data_, uint8_t len_) // Расчет контрольной суммы. Берется массив всех оправляемых данных без последнего байта и суммируется побайтно, потом в бинарном виде инвертируются 1 в нолик и нолик в единицу и потом прибавляется 1
{
    uint32_t sum = 0;
    // Serial.println(len_, DEC);
    for (int i = 1; i < (len_ - 1); i++) // Считаем без последнего байта и без первого
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
    return sum;
}

// Установка скорости обмена
void sk60plus_autoBaund()
{
    HAL_GPIO_WritePin(laserEn_GPIO_Port, laserEn_Pin, 0); // Пин датчика PWREN
    HAL_Delay(100);
    HAL_GPIO_WritePin(laserEn_GPIO_Port, laserEn_Pin, 1);
    HAL_Delay(100);
    uint8_t buf[1] = {0x55};
    for (int i = 0; i < 5; i++) // в течении 2 секунд посылкаем байт 0х55 для установки нужной скорости обмена
    {
        HAL_UART_Transmit(&huart2, buf, sizeof(buf), 100); // Отправляем команду
        HAL_UART_Transmit(&huart4, buf, sizeof(buf), 100); // Отправляем команду
        HAL_UART_Transmit(&huart5, buf, sizeof(buf), 100); // Отправляем команду
        HAL_UART_Transmit(&huart6, buf, sizeof(buf), 100); // Отправляем команду
        HAL_Delay(25);
    }
}

// Function: master set slave’s address, this address will not lost after module power off; Slave address set to 0xYY (!!!Beware: address only take bit[6:0], other bits will be ignored).
void sk60plus_setModulAddress(UART_HandleTypeDef *huart, uint8_t addr_, uint8_t YY_)
{
    flagCallBackUart = false; // Эту функцию Не нужно отпабатывать в колбеке
    printf("setModulAddress ->");
    memset(_bufRead, 0, sizeof(_bufRead));             // Очистка буфера
    HAL_UART_DMAStop(huart);                           // Остановка DMA
    status = HAL_UART_Receive_DMA(huart, _bufRead, 9); // Запускаем ожидание ответа, указываем куда и сколько байт мы ждем.
    printf("status= %i \r\n", status);

    uint8_t buf[9] = {0xAA, addr_, 0x00, 0x10, 0x00, 0x01, 0x00, YY_, 0x00};
    buf[8] = calcCs(buf, 9);
    HAL_UART_Transmit(huart, buf, sizeof(buf), 100); // Отправляем команду

    HAL_Delay(100);
    if (_bufRead[0] == 0xAA && _bufRead[7] == YY_) // Если ответ правильный смотри что в ответе
    {
        printf("setModulAddress = %0#6X  \n", _bufRead[7]);
    }
    else
    {
        printf("setModulAddress = ERROR !!! \n");
    }
}
// Function: master read out the module’s HW version number;
void sk60plus_readHardwareVersion(uint16_t port_)
{
    printf("readHardwareVersion -> ");
    uint8_t addr = _addr | 0b10000000; // R/W indicate bit, 0: Master write to Slave, 1: Master read from Slave  Slave address is 0x51, address has only 7-bits, so the address is from 0x00 to 0x7F, 0x00 is the default address before master issue module address change command, 0x7F is the broadcast address reserved for one-master to multi-slave network;
    uint8_t buf[5] = {0xAA, addr, 0x00, 0x0A, 0x00};
    buf[4] = calcCs(buf, 5);
    HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100); // Отправляем команду
    HAL_Delay(25);
    for (int i = 0; i < dataUART[port_].len; i++)
    {
        printf("%x ", dataUART[port_].adr[i]);
    }

    if (dataUART[port_].adr[0] == 0xAA) // Если в буфере правильный ответ
    {
        _hVersion = (uint16_t)(dataUART[port_].adr[7] | (dataUART[port_].adr[6] << 8));
        printf("hVersion = %0#6X = %i\n", _hVersion, _hVersion);
    }
    else
        printf("ERROR !!! \n");
}
// Function: master read out the module’s SW version number;
void sk60plus_readSoftwareVersion(u_int16_t port_)
{
    printf("readSoftwareVersion -> ");
    uint8_t addr = _addr | 0b10000000; // R/W indicate bit, 0: Master write to Slave, 1: Master read from Slave  Slave address is 0x51, address has only 7-bits, so the address is from 0x00 to 0x7F, 0x00 is the default address before master issue module address change command, 0x7F is the broadcast address reserved for one-master to multi-slave network;
    uint8_t buf[5] = {0xAA, addr, 0x00, 0x0C, 0x00};
    buf[4] = calcCs(buf, 5);
    HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100); // Отправляем команду
    HAL_Delay(25);

    for (int i = 0; i < dataUART[port_].len; i++)
    {
        printf("%x ", dataUART[port_].adr[i]);
    }

    if (dataUART[port_].adr[0] == 0xAA) // Если в буфере правильный ответ
    {
        _sVersion = (uint16_t)(dataUART[port_].adr[7] | (dataUART[port_].adr[6] << 8));
        printf("sVersion = %0#6X = %i \n", _sVersion, _sVersion);
    }
    else
        printf("ERROR !!! \n");
}
// Function: master read out the module’s Serial number;
void sk60plus_readSerialNumber(u_int16_t port_)
{
    printf("readSerialNumber    -> ");
    uint8_t addr = _addr | 0b10000000; // R/W indicate bit, 0: Master write to Slave, 1: Master read from Slave  Slave address is 0x51, address has only 7-bits, so the address is from 0x00 to 0x7F, 0x00 is the default address before master issue module address change command, 0x7F is the broadcast address reserved for one-master to multi-slave network;
    uint8_t buf[5] = {0xAA, addr, 0x00, 0x0E, 0x00};
    buf[4] = calcCs(buf, 5);
    HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100); // Отправляем команду
    HAL_Delay(25);

    for (int i = 0; i < dataUART[port_].len; i++)
    {
        printf("%x ", dataUART[port_].adr[i]);
    }

    if (dataUART[port_].adr[0] == 0xAA) // Если в буфере правильный ответ
    {
        //_sNumber = (uint16_t)(_bufRead[7] | (_bufRead[6] << 8));
        //_sNumber = (uint32_t)(_bufRead[9] | (_bufRead[8] << 8) | _bufRead[7] << 16 | (_bufRead[6] << 24));
        _sNumber = (uint32_t)(dataUART[port_].adr[9] | (dataUART[port_].adr[6] << 8) | dataUART[port_].adr[7] << 16 | (dataUART[port_].adr[8]) << 24);

        printf(" sNumber = %0#10lX = %li \n", _sNumber, _sNumber);
    }
    else
        printf("readSerialNumber = ERROR !!! \n");
}

// Function: master read out the module’s input voltage in mV with BCD encode;
void sk60plus_readInputVoltage(uint16_t port_)
{
    printf("readInputVoltage -> ");
    uint8_t addr = _addr | 0b10000000; // R/W indicate bit, 0: Master write to Slave, 1: Master read from Slave  Slave address is 0x51, address has only 7-bits, so the address is from 0x00 to 0x7F, 0x00 is the default address before master issue module address change command, 0x7F is the broadcast address reserved for one-master to multi-slave network;
    uint8_t buf[5] = {0xAA, addr, 0x00, 0x06, 0x00};
    buf[4] = calcCs(buf, 5);
    HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100); // Отправляем команду
    HAL_Delay(25);

    for (int i = 0; i < dataUART[port_].len; i++)
    {
        printf("%x ", dataUART[port_].adr[i]);
    }

    if (dataUART[port_].adr[0] == 0xAA)
    {
        uint16_t a1 = ((dataUART[port_].adr[6] >> 4) * 1000);        // Сдвигаем и убтраем младшие биты и прибавляем 0x30 тем самы превращаем в число
        uint16_t a2 = ((dataUART[port_].adr[6] & 0b00001111)) * 100; // Убираем старшие биты и прибавляет 0x30 тем самы превращаем в число
        uint16_t a3 = ((dataUART[port_].adr[7] >> 4)) * 10;          // Сдвигаем и убтраем младшие биты и прибавляем 0x30 тем самы превращаем в число
        uint16_t a4 = ((dataUART[port_].adr[7] & 0b00001111)) * 1;   // Убираем старшие биты и прибавляет 0x30 тем самы превращаем в число
        _inputVoltage = a1 + a2 + a3 + a4;
        printf("readInputVoltage = %i mV \n", _inputVoltage);
    }
    else
        printf("ERROR !!! \n");
}
// Function: turn on or turn off laser beam, if 0xZZ = 0x01 laser on, 0xZZ = 0x00 laser off.
void sk60plus_setLaser(uint16_t port_, uint8_t ZZ_)
{
    printf("setLaser          -> ");
    printf("status= %i ", status);

    uint8_t buf[9] = {0xAA, _addr, 0x01, 0xBE, 0x00, 0x01, 0x00, ZZ_, 0x00};
    buf[8] = calcCs(buf, 9);
    HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100); // Отправляем команду
    HAL_Delay(50);

    printf(" DATA => %X %X %X %X ", dataUART[port_].adr[0], dataUART[port_].adr[1], dataUART[port_].adr[2], dataUART[port_].adr[3]);
    if (dataUART[port_].adr[0] == 0xAA)
    {
        printf(" ok \r\n");
    }
    else
    {
        printf(" ERROR \r\n");
    }
    memset(dataUART[port_].adr, 0, RX_BUFFER_SIZE); // Очистка буфера
}

// Function: Master transfer one uint8_t 0x58 (upper case character ‘X’) to stop continuous measure mode immediately
void sk60plus_stopContinuous(u_int16_t port_)
{
    printf("stopContinuous = %0#6X \n", 0x58);
    uint8_t buf[1] = {0x58};
    HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100); // Отправляем команду
    // HAL_Delay(10);
}

// Function: Initiate slave to do 1-shot measure in Auto mode.
void sk60plus_startSingleAuto(u_int16_t port_)
{
    _distance = 0;
    _signalQuality = 0;
    printf("startSingleAuto    -> ");
    uint8_t buf[9] = {0xAA, _addr, 0x00, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00};
    buf[8] = calcCs(buf, 9);
    HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100); // Отправляем команду
    HAL_Delay(900);

    if (dataUART[port_].adr[0] == 0xAA)
    {
        _distance = (uint32_t)(dataUART[port_].adr[9] | dataUART[port_].adr[6] << 24 | dataUART[port_].adr[7] << 16 | dataUART[port_].adr[8] << 8);
        _signalQuality = (uint16_t)(dataUART[port_].adr[11] | dataUART[port_].adr[10] << 8);
        printf(" distance = %li mm signalQuality= %i \n", _distance, _signalQuality);
    }
    else
    {
        for (int i = 0; i < dataUART[port_].len; i++)
        {
            printf("%x ", dataUART[port_].adr[i]);
        }
        printf(" ERROR \r\n");
    }
}
// Function: Initiate slave to do 1-shot measure in Auto mode.
void sk60plus_startContinuousAuto(u_int16_t port_)
{
    printf("startContinuousAuto \n");
    uint8_t buf[9] = {0xAA, _addr, 0x00, 0x20, 0x00, 0x01, 0x00, 0x06, 0x00};
    buf[8] = calcCs(buf, 9);
    HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100); // Отправляем команду
}

void sk60plus_startContinuousSlow(u_int16_t port_) // Function: Initiate slave to do continuous measure in slow mode.
{
    printf("startContinuousSlow \n");
    uint8_t buf[9] = {0xAA, _addr, 0x00, 0x20, 0x00, 0x01, 0x00, 0x05, 0x00};
    buf[8] = calcCs(buf, 9);
    HAL_UART_Transmit(dataUART[port_].huart, buf, sizeof(buf), 100); // Отправляем команду
}
// void sk60plus_setModulMeasureOffset(int16_t ZZYY_) // Function: master set slave’s measure offset. For example, if the offset 0xZZYY = 0x7B(+123) , it means the final output of measure result will PLUS 123 millimeters , if the offset 0xZZYY = 0xFF85(-123), it means the final output of measure result will MINUS 123 millimeters
// {
//     printf("setModulMeasureOffset \n");
//     clearBuf();
//     uint8_t ZZ = ZZYY_ >> 8;
//     uint8_t YY = ZZYY_ & 0b0000000011111111;
//     uint8_t buf[9] = {0xAA, _addr, 0x00, 0x12, 0x00, 0x01, ZZ, YY, 0x00};
//     buf[8] = calcCs(buf, 9);
//     Serial2.write(buf, 9);
//     delay(10);
//     if (readBuf()) // Если ответ правильный смотри что в ответе
//     {
//         _offSet = (uint16_t)(_bufRead[7] | (_bufRead[6] << 8));

//         printf("setModulMeasureOffset = %0#6X \n", _offSet);
//     }
// }

// void sk60plus_startSingleFast() // Function: Initiate slave to do 1-shot measure in fast mode.
// {
//     printf("startSingleFast \n");
//     clearBuf();
//     uint8_t buf[9] = {0xAA, _addr, 0x00, 0x20, 0x00, 0x01, 0x00, 0x02, 0x00};
//     buf[8] = calcCs(buf, 9);
//     Serial2.write(buf, 9);
//     delay(200);
//     if (readBuf())
//     {
//         _distance = (uint32_t)(_bufRead[9] | _bufRead[6] << 24 | _bufRead[7] << 16 | _bufRead[8] << 8);
//         _signalQuality = (uint16_t)(_bufRead[11] | _bufRead[10] << 8);
//     }
//     printf("startSingleFast distance = %i mm signalQuality= %i \n", _distance, _signalQuality);
// }

// void sk60plus_startContinuousFast() // Function: Initiate slave to do 1-shot measure in fast mode.
// {
//     printf("startContinuousFast \n");
//     clearBuf();
//     uint8_t buf[9] = {0xAA, _addr, 0x00, 0x20, 0x00, 0x01, 0x00, 0x06, 0x00};
//     buf[8] = calcCs(buf, 9);
//     Serial2.write(buf, 9);
//     delay(200);
//     if (readBuf())
//     {
//         _distance = (uint32_t)(_bufRead[9] | _bufRead[6] << 24 | _bufRead[7] << 16 | _bufRead[8] << 8);
//         _signalQuality = (uint16_t)(_bufRead[11] | _bufRead[10] << 8);
//     }
//     printf("startContinuousFast distance = %i mm signalQuality= %i \n", _distance, _signalQuality);
// }

// void sk60plus_startContinuousSuperFast() // Function: Initiate slave to do 1-shot measure in Superfast mode. ~35Hz
// {
//     printf("startContinuousSuperFast \n");
//     clearBuf();
//     uint8_t buf[9] = {0xAA, _addr, 0x00, 0x20, 0x00, 0x01, 0x00, 0x07, 0x00};
//     buf[8] = calcCs(buf, 9);
//     Serial2.write(buf, 9);
//     delay(200);
//     if (readBuf())
//     {
//         _distance = (uint32_t)(_bufRead[9] | _bufRead[6] << 24 | _bufRead[7] << 16 | _bufRead[8] << 8);
//         _signalQuality = (uint16_t)(_bufRead[11] | _bufRead[10] << 8);
//     }
//     printf("startContinuousSuperFast distance = %i mm signalQuality= %i \n", _distance, _signalQuality);
// }
// void sk60plus_getBroadcastSingleMeasure() // Function: Initiate all slave to do 1-shot measure in auto mode/
// {
//     // printf("startBroadcastSingleMeasure \n");
//     clearBuf();
//     uint8_t buf[9] = {0xAA, 0x7F, 0x00, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00};
//     buf[8] = calcCs(buf, 9);
//     Serial2.write(buf, 9);
// }
// void sk60plus_getStatus()
// {
//     // printf("getStatus \n");
//     clearBuf();
//     uint8_t addr = _addr | 0b10000000; // R/W indicate bit, 0: Master write to Slave, 1: Master read from Slave  Slave address is 0x51, address has only 7-bits, so the address is from 0x00 to 0x7F, 0x00 is the default address before master issue module address change command, 0x7F is the broadcast address reserved for one-master to multi-slave network;
//     uint8_t buf[5] = {0xAA, addr, 0x00, 0x00, 0x00};
//     buf[4] = calcCs(buf, 5);
//     Serial2.write(buf, 5);
// }
// bool sk60plus_readStatus()
// {
//     (readBuf()) ? _status = _bufRead[7] : _status = 0xFF;
//     // printf("readStatus=  %0#4X \n", _status);
//     if (_status == 0x00)
//         return true;
//     else
//         return false;
// }
// void sk60plus_getMeasureResult() // Function: master read out the distance measure result;
// {
//     // printf("getMeasureResult \n");
//     clearBuf();
//     uint8_t addr = _addr | 0b10000000; // R/W indicate bit, 0: Master write to Slave, 1: Master read from Slave  Slave address is 0x51, address has only 7-bits, so the address is from 0x00 to 0x7F, 0x00 is the default address before master issue module address change command, 0x7F is the broadcast address reserved for one-master to multi-slave network;
//     uint8_t buf[5] = {0xAA, addr, 0x00, 0x22, 0x00};
//     buf[4] = calcCs(buf, 5);
//     Serial2.write(buf, 5);
// }
// bool sk60plus_readMeasureResult() // Function: master read out the distance measure result;
// {
//     if (readBuf())
//     {
//         _distance = (uint32_t)(_bufRead[9] | _bufRead[6] << 24 | _bufRead[7] << 16 | _bufRead[8] << 8);
//         _signalQuality = (uint16_t)(_bufRead[11] | _bufRead[10] << 8);
//         DEBUG_PRINTF(" l= %i d= %i SQ= %i \n", _addr, _distance, _signalQuality);
//         return true;
//     }
//     else
//     {
//         DEBUG_PRINTF("readMeasureResult ERROR \n");
//         return false;
//     }
// }

// // Метод считывания данных ответа в буфер общий для всех. Потом берем нужные байты. Если ответ короче то в конце FF будет
// bool sk60plus_readBuf()
// {
//     for (int i = 0; i < 13; i++)
//     {
//         _bufRead[i] = Serial2.read();
//         // if (_bufRead[i] != 0xFF)
//         // DEBUG_PRINTF("%0#4X-", _bufRead[i]);
//     }
//     // DEBUG_PRINTF("\n");
//     //  uint8_t addr = _addr | 0b10000000;                 // R/W indicate bit, 0: Master write to Slave, 1: Master read from Slave  Slave address is 0x51, address has only 7-bits, so the address is from 0x00 to 0x7F, 0x00 is the default address before master issue module address change command, 0x7F is the broadcast address reserved for one-master to multi-slave network;
//     if (_bufRead[0] == 0xAA) // Если первый байт 0xAA  и ответ от нужного датчика
//     {
//         return true;
//     }
//     else
//     {
//         return false;
//     }
// }

// Расчет растояния на основе полученных данных
uint32_t laser60_calcDistance(uint8_t *rx_bufferUART)
{
    return (uint32_t)(rx_bufferUART[9] | rx_bufferUART[6] << 24 | rx_bufferUART[7] << 16 | rx_bufferUART[8] << 8);
}
// Расчет качества сигнала на основе полученных данных
uint16_t laser60_calcSignalQuality(uint8_t *rx_bufferUART)
{
    return (uint16_t)(rx_bufferUART[11] | rx_bufferUART[10] << 8);
}
#endif