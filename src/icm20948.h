/*
 * icm20948.h
 *
 *  Created on: Dec 26, 2020
 *      Author: mokhwasomssi
 */

#ifndef __ICM20948_H__
#define __ICM20948_H__

#include "spi.h" // header from stm32cubemx code generate
#include <stdbool.h>
#include "main.h" // header from stm32cubemx code generate

/* User Configuration */
// #define ICM20948_SPI					(&hspi1)

// #define ICM20948_SPI_CS_PIN_PORT		GPIOA
// #define ICM20948_SPI_CS_PIN_NUMBER		GPIO_PIN_4

// I2C настройки

extern I2C_HandleTypeDef hi2c1;

#define ICM20948_I2C (&hi2c1)			 // Укажите ваш I2C handle (например, hi2c1)
#define ICM20948_I2C_ADDRESS (0x69 << 1) // Адрес ICM-20948 (0x68 или 0x69, сдвинутый влево)

/* Defines */
#define READ 0x80
#define WRITE 0x00

/* Typedefs */
// Формирование значения для регистра REG_BANK_SEL
// Значение банка (ub) сдвигается на 4 бита влево, так как биты [5:4] в REG_BANK_SEL задают номер банка
// Например, ub_0 = 0x00, ub_1 = 0x10, ub_2 = 0x20, ub_3 = 0x30
typedef enum
{
	ub_0 = 0 << 4,
	ub_1 = 1 << 4,
	ub_2 = 2 << 4,
	ub_3 = 3 << 4
} userbank;

typedef enum
{
	_250dps,
	_500dps,
	_1000dps,
	_2000dps
} gyro_full_scale;

typedef enum
{
	_2g,
	_4g,
	_8g,
	_16g
} accel_full_scale;

typedef struct
{
	float x;
	float y;
	float z;
} axises;

typedef enum
{
	power_down_mode = 0,
	single_measurement_mode = 1,
	continuous_measurement_10hz = 2,
	continuous_measurement_20hz = 4,
	continuous_measurement_50hz = 6,
	continuous_measurement_100hz = 8
} operation_mode;

/* Main Functions */

void enable_i2c_mode(void);					   // Отключение SPI и включение I2C
void I2C_ScanDevices(I2C_HandleTypeDef *hi2c); // Функция для сканирования устройств на шине I2C
// sensor init function.
// if sensor id is wrong, it is stuck in while.
// Инициализация ICM-20948
// Настраивает основные параметры датчика (сброс, включение I2C, выбор источника тактирования, настройка фильтров и т.д.)
// Вызывается перед началом работы с акселерометром и гироскопом
void icm20948_init();

// Инициализация магнитометра AK09916
// Настраивает магнитометр, подключённый через внутренний I2C-мастер ICM-20948
// Включает I2C-мастер ICM-20948 и устанавливает параметры работы AK09916
void ak09916_init();

// Чтение необработанных данных гироскопа (16-битные значения с АЦП)
// Параметры:
// - data: Указатель на структуру axises (x, y, z), куда записываются необработанные данные для каждой оси (в единицах АЦП)
// Используется для получения сырых данных с гироскопа без преобразования в физические единицы
void icm20948_gyro_read(axises *data);

// Чтение необработанных данных акселерометра (16-битные значения с АЦП)
// Параметры:
// - data: Указатель на структуру axises (x, y, z), куда записываются необработанные данные для каждой оси (в единицах АЦП)
// Используется для получения сырых данных с акселерометра без преобразования
void icm20948_accel_read(axises *data);

// Чтение необработанных данных магнитометра AK09916 (16-битные значения с АЦП)
// Параметры:
// - data: Указатель на структуру axises (x, y, z), куда записываются необработанные данные магнитометра
// Возвращает: true, если данные успешно прочитаны, false в случае ошибки (например, магнитометр не готов)
// Используется для получения сырых данных магнитного поля
bool ak09916_mag_read(axises *data);

// Преобразование необработанных данных гироскопа в угловую скорость (градусы в секунду, dps)
// Параметры:
// - data: Указатель на структуру axises (x, y, z), куда записываются значения в dps
// Преобразует 16-битные значения АЦП в физические единицы на основе текущей шкалы (full scale) гироскопа
void icm20948_gyro_read_dps(axises *data);

// Преобразование необработанных данных акселерометра в ускорение (g)
// Параметры:
// - data: Указатель на структуру axises (x, y, z), куда записываются значения в g (ускорение силы тяжести)
// Преобразует 16-битные значения АЦП в физические единицы на основе текущей шкалы акселерометра
void icm20948_accel_read_g(axises *data);

// Преобразование необработанных данных магнитометра в магнитное поле (микротесла, uT)
// Параметры:
// - data: Указатель на структуру axises (x, y, z), куда записываются значения в uT
// Возвращает: true, если данные успешно прочитаны и преобразованы, false в случае ошибки
// Преобразует сырые данные магнитометра в физические единицы
bool ak09916_mag_read_uT(axises *data);

// Проверка идентификатора ICM-20948
// Читает регистр WHO_AM_I (адрес 0x00, банк 0), который должен вернуть 0xEA
// Возвращает: true, если прочитано 0xEA (подтверждает, что устройство ICM-20948), false в случае ошибки
// Используется для проверки связи с датчиком
bool icm20948_who_am_i();

// Проверка идентификатора магнитометра AK09916
// Читает регистр WIA2 (адрес 0x01) через внутренний I2C-мастер ICM-20948, который должен вернуть 0x09
// Возвращает: true, если прочитано 0x09 (подтверждает, что магнитометр AK09916), false в случае ошибки
// Используется для проверки связи с магнитометром
bool ak09916_who_am_i();

// Сброс ICM-20948
// Устанавливает бит DEV_RESET в регистре PWR_MGMT_1 (адрес 0x06, банк 0) для выполнения аппаратного сброса
// После сброса требуется задержка (около 50 мс) для стабилизации
// Используется для возврата датчика в начальное состояние
void icm20948_device_reset();

// Программный сброс магнитометра AK09916
// Отправляет команду сброса через внутренний I2C-мастер ICM-20948 в регистр CNTL3 (адрес 0x32)
// Используется для возврата магнитометра в начальное состояние
void ak09916_soft_reset();

// Вывод ICM-20948 из спящего режима
// Сбрасывает бит SLEEP в регистре PWR_MGMT_1 (адрес 0x06, банк 0) для активации датчика
// Используется для включения акселерометра и гироскопа после спящего режима
void icm20948_wakeup();

// Перевод ICM-20948 в спящий режим
// Устанавливает бит SLEEP в регистре PWR_MGMT_1 (адрес 0x06, банк 0) для снижения энергопотребления
// Используется для отключения акселерометра и гироскопа, когда они не нужны
void icm20948_sleep();

// Включение SPI slave-режима для ICM-20948
// Настраивает ICM-20948 как SPI-ведомое устройство (не используется в вашем случае, так как вы работаете по I2C)
// Используется для специфичных приложений, где ICM-20948 выступает как подчинённое устройство по SPI
void icm20948_spi_slave_enable();

// Сброс внутреннего I2C-мастера ICM-20948
// Устанавливает бит I2C_MST_RST в регистре USER_CTRL (адрес 0x03, банк 0) для сброса I2C-мастера
// Используется для перезапуска внутреннего I2C-мастера, который управляет связью с магнитометром AK09916
void icm20948_i2c_master_reset();

// Включение внутреннего I2C-мастера ICM-20948
// Устанавливает бит I2C_MST_EN в регистре USER_CTRL (адрес 0x03, банк 0) для активации I2C-мастера
// Необходимо для связи с магнитометром AK09916 через внутренний I2C-интерфейс ICM-20948
void icm20948_i2c_master_enable();
void icm20948_i2c_master_disable();

// Настройка частоты тактирования внутреннего I2C-мастера
// Параметры:
// - config: Значение от 0 до 15, определяющее частоту I2C-мастера (см. datasheet, раздел 6.5)
// Используется для настройки скорости связи между ICM-20948 и AK09916 (обычно ~400 кГц)
void icm20948_i2c_master_clk_frq(uint8_t config);

// Выбор источника тактирования для ICM-20948
// Параметры:
// - source: Значение для регистра PWR_MGMT_1 (CLKSEL, биты [2:0]), задающее источник тактирования
// Используется для выбора лучшего источника тактирования (обычно авто-выбор для минимального дрейфа)
void icm20948_clock_source(uint8_t source);

// Включение выравнивания частоты дискретизации (ODR align)
// Устанавливает бит ODR_ALIGN_EN в регистре ODR_ALIGN_EN (адрес 0x09, банк 0)
// Используется для синхронизации частоты вывода данных (ODR) между акселерометром и гироскопом
void icm20948_odr_align_enable();

// Настройка низкочастотного фильтра для гироскопа
// Параметры:
// - config: Значение от 0 до 7, задающее частоту среза фильтра (см. datasheet, раздел 6.8)
// Используется для уменьшения шума в данных гироскопа
void icm20948_gyro_low_pass_filter(uint8_t config);

// Настройка низкочастотного фильтра для акселерометра
// Параметры:
// - config: Значение от 0 до 7, задающее частоту среза фильтра (см. datasheet, раздел 6.8)
// Используется для уменьшения шума в данных акселерометра
void icm20948_accel_low_pass_filter(uint8_t config);

// Установка делителя частоты дискретизации для гироскопа
// Параметры:
// - divider: Значение делителя (Output Data Rate = 1.125 кГц / (1 + divider))
// Используется для настройки частоты вывода данных гироскопа (например, 100 Гц при divider = 10)
void icm20948_gyro_sample_rate_divider(uint8_t divider);

// Установка делителя частоты дискретизации для акселерометра
// Параметры:
// - divider: Значение делителя (Output Data Rate = 1.125 кГц / (1 + divider))
// Используется для настройки частоты вывода данных акселерометра
void icm20948_accel_sample_rate_divider(uint16_t divider);

// Настройка режима работы магнитометра AK09916
// Параметры:
// - mode: Режим работы (например, одиночный замер, непрерывный режим, см. datasheet AK09916)
// Используется для установки режима работы магнитометра (например, периодические измерения)
void ak09916_operation_mode_setting(operation_mode mode);

// Калибровка гироскопа
// Выполняет программную калибровку для устранения смещений (offset) в данных гироскопа
// Должна выполняться перед выбором полной шкалы (full scale) для точных измерений
void icm20948_gyro_calibration();

// Калибровка акселерометра
// Выполняет программную калибровку для устранения смещений в данных акселерометра
// Должна выполняться перед выбором полной шкалы для точных измерений
void icm20948_accel_calibration();

// Выбор полной шкалы для гироскопа
// Параметры:
// - full_scale: Значение шкалы (например, ±250 dps, ±500 dps, см. datasheet, раздел 6.8)
// Используется для установки диапазона измерений гироскопа (влияет на чувствительность)
void icm20948_gyro_full_scale_select(gyro_full_scale full_scale);

// Выбор полной шкалы для акселерометра
// Параметры:
// - full_scale: Значение шкалы (например, ±2g, ±4g, см. datasheet, раздел 6.8)
// Используется для установки диапазона измерений акселерометра (влияет на чувствительность)
void icm20948_accel_full_scale_select(accel_full_scale full_scale);

// Включаем bypass режим для первоначальной настройки магнитометра
void icm20948_bypass_en();
// Выключаем bypass режим для первоначальной настройки магнитометра
void icm20948_bypass_disable();
//***********************************************************************************************************
// void icm20948_init();
// void ak09916_init();

// // 16 bits ADC value. raw data.
// void icm20948_gyro_read(axises *data);
// void icm20948_accel_read(axises *data);
// bool ak09916_mag_read(axises *data);

// // Convert 16 bits ADC value to their unit.
// void icm20948_gyro_read_dps(axises *data);
// void icm20948_accel_read_g(axises *data);
// bool ak09916_mag_read_uT(axises *data);

// /* Sub Functions */
// bool icm20948_who_am_i();
// bool ak09916_who_am_i();

// void icm20948_device_reset();
// void ak09916_soft_reset();

// void icm20948_wakeup();
// void icm20948_sleep();

// void icm20948_spi_slave_enable();

// void icm20948_i2c_master_reset();
// void icm20948_i2c_master_enable();
// void icm20948_i2c_master_clk_frq(uint8_t config); // 0 - 15

// void icm20948_clock_source(uint8_t source);
// void icm20948_odr_align_enable();

// void icm20948_gyro_low_pass_filter(uint8_t config);	 // 0 - 7
// void icm20948_accel_low_pass_filter(uint8_t config); // 0 - 7

// // Output Data Rate = 1.125kHz / (1 + divider)
// void icm20948_gyro_sample_rate_divider(uint8_t divider);
// void icm20948_accel_sample_rate_divider(uint16_t divider);
// void ak09916_operation_mode_setting(operation_mode mode);

// // Calibration before select full scale.
// void icm20948_gyro_calibration();
// void icm20948_accel_calibration();

// void icm20948_gyro_full_scale_select(gyro_full_scale full_scale);
// void icm20948_accel_full_scale_select(accel_full_scale full_scale);

/* ICM-20948 Registers */
#define ICM20948_ID 0xEA
#define REG_BANK_SEL 0x7F

// USER BANK 0
#define B0_WHO_AM_I 0x00
#define B0_USER_CTRL 0x03
#define B0_LP_CONFIG 0x05
#define B0_PWR_MGMT_1 0x06
#define B0_PWR_MGMT_2 0x07
#define B0_INT_PIN_CFG 0x0F
#define B0_INT_ENABLE 0x10
#define B0_INT_ENABLE_1 0x11
#define B0_INT_ENABLE_2 0x12
#define B0_INT_ENABLE_3 0x13
#define B0_I2C_MST_STATUS 0x17
#define B0_INT_STATUS 0x19
#define B0_INT_STATUS_1 0x1A
#define B0_INT_STATUS_2 0x1B
#define B0_INT_STATUS_3 0x1C
#define B0_DELAY_TIMEH 0x28
#define B0_DELAY_TIMEL 0x29
#define B0_ACCEL_XOUT_H 0x2D
#define B0_ACCEL_XOUT_L 0x2E
#define B0_ACCEL_YOUT_H 0x2F
#define B0_ACCEL_YOUT_L 0x30
#define B0_ACCEL_ZOUT_H 0x31
#define B0_ACCEL_ZOUT_L 0x32
#define B0_GYRO_XOUT_H 0x33
#define B0_GYRO_XOUT_L 0x34
#define B0_GYRO_YOUT_H 0x35
#define B0_GYRO_YOUT_L 0x36
#define B0_GYRO_ZOUT_H 0x37
#define B0_GYRO_ZOUT_L 0x38
#define B0_TEMP_OUT_H 0x39
#define B0_TEMP_OUT_L 0x3A
#define B0_EXT_SLV_SENS_DATA_00 0x3B
#define B0_EXT_SLV_SENS_DATA_01 0x3C
#define B0_EXT_SLV_SENS_DATA_02 0x3D
#define B0_EXT_SLV_SENS_DATA_03 0x3E
#define B0_EXT_SLV_SENS_DATA_04 0x3F
#define B0_EXT_SLV_SENS_DATA_05 0x40
#define B0_EXT_SLV_SENS_DATA_06 0x41
#define B0_EXT_SLV_SENS_DATA_07 0x42
#define B0_EXT_SLV_SENS_DATA_08 0x43
#define B0_EXT_SLV_SENS_DATA_09 0x44
#define B0_EXT_SLV_SENS_DATA_10 0x45
#define B0_EXT_SLV_SENS_DATA_11 0x46
#define B0_EXT_SLV_SENS_DATA_12 0x47
#define B0_EXT_SLV_SENS_DATA_13 0x48
#define B0_EXT_SLV_SENS_DATA_14 0x49
#define B0_EXT_SLV_SENS_DATA_15 0x4A
#define B0_EXT_SLV_SENS_DATA_16 0x4B
#define B0_EXT_SLV_SENS_DATA_17 0x4C
#define B0_EXT_SLV_SENS_DATA_18 0x4D
#define B0_EXT_SLV_SENS_DATA_19 0x4E
#define B0_EXT_SLV_SENS_DATA_20 0x4F
#define B0_EXT_SLV_SENS_DATA_21 0x50
#define B0_EXT_SLV_SENS_DATA_22 0x51
#define B0_EXT_SLV_SENS_DATA_23 0x52
#define B0_FIFO_EN_1 0x66
#define B0_FIFO_EN_2 0x67
#define B0_FIFO_RST 0x68
#define B0_FIFO_MODE 0x69
#define B0_FIFO_COUNTH 0X70
#define B0_FIFO_COUNTL 0X71
#define B0_FIFO_R_W 0x72
#define B0_DATA_RDY_STATUS 0x74
#define B0_FIFO_CFG 0x76

// USER BANK 1
#define B1_SELF_TEST_X_GYRO 0x02
#define B1_SELF_TEST_Y_GYRO 0x03
#define B1_SELF_TEST_Z_GYRO 0x04
#define B1_SELF_TEST_X_ACCEL 0x0E
#define B1_SELF_TEST_Y_ACCEL 0x0F
#define B1_SELF_TEST_Z_ACCEL 0x10
#define B1_XA_OFFS_H 0x14
#define B1_XA_OFFS_L 0x15
#define B1_YA_OFFS_H 0x17
#define B1_YA_OFFS_L 0x18
#define B1_ZA_OFFS_H 0x1A
#define B1_ZA_OFFS_L 0x1B
#define B1_TIMEBASE_CORRECTION_PLL 0x28

// USER BANK 2
#define B2_GYRO_SMPLRT_DIV 0x00
#define B2_GYRO_CONFIG_1 0x01
#define B2_GYRO_CONFIG_2 0x02
#define B2_XG_OFFS_USRH 0x03
#define B2_XG_OFFS_USRL 0x04
#define B2_YG_OFFS_USRH 0x05
#define B2_YG_OFFS_USRL 0x06
#define B2_ZG_OFFS_USRH 0x07
#define B2_ZG_OFFS_USRL 0x08
#define B2_ODR_ALIGN_EN 0x09
#define B2_ACCEL_SMPLRT_DIV_1 0x10
#define B2_ACCEL_SMPLRT_DIV_2 0x11
#define B2_ACCEL_INTEL_CTRL 0x12
#define B2_ACCEL_WOM_THR 0x13
#define B2_ACCEL_CONFIG 0x14
#define B2_ACCEL_CONFIG_2 0x15
#define B2_FSYNC_CONFIG 0x52
#define B2_TEMP_CONFIG 0x53
#define B2_MOD_CTRL_USR 0X54

// USER BANK 3
#define B3_I2C_MST_ODR_CONFIG 0x00
#define B3_I2C_MST_CTRL 0x01
#define B3_I2C_MST_DELAY_CTRL 0x02
#define B3_I2C_SLV0_ADDR 0x03
#define B3_I2C_SLV0_REG 0x04
#define B3_I2C_SLV0_CTRL 0x05
#define B3_I2C_SLV0_DO 0x06
#define B3_I2C_SLV1_ADDR 0x07
#define B3_I2C_SLV1_REG 0x08
#define B3_I2C_SLV1_CTRL 0x09
#define B3_I2C_SLV1_DO 0x0A
#define B3_I2C_SLV2_ADDR 0x0B
#define B3_I2C_SLV2_REG 0x0C
#define B3_I2C_SLV2_CTRL 0x0D
#define B3_I2C_SLV2_DO 0x0E
#define B3_I2C_SLV3_ADDR 0x0F
#define B3_I2C_SLV3_REG 0x10
#define B3_I2C_SLV3_CTRL 0x11
#define B3_I2C_SLV3_DO 0x12
#define B3_I2C_SLV4_ADDR 0x13
#define B3_I2C_SLV4_REG 0x14
#define B3_I2C_SLV4_CTRL 0x15
#define B3_I2C_SLV4_DO 0x16
#define B3_I2C_SLV4_DI 0x17

/* AK09916 Registers */
#define AK09916_ID 0x09
#define MAG_SLAVE_ADDR 0x0C

#define MAG_WIA2 0x01
#define MAG_ST1 0x10
#define MAG_HXL 0x11
#define MAG_HXH 0x12
#define MAG_HYL 0x13
#define MAG_HYH 0x14
#define MAG_HZL 0x15
#define MAG_HZH 0x16
#define MAG_ST2 0x18
#define MAG_CNTL2 0x31
#define MAG_CNTL3 0x32
#define MAG_TS1 0x33
#define MAG_TS2 0x34

#endif /* __ICM20948_H__ */