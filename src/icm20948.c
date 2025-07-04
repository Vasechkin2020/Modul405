/*
 * icm20948.c
 *
 *  Created on: Dec 26, 2020
 *      Author: mokhwasomssi
 */


#include "icm20948.h"
#include <string.h>



static float gyro_scale_factor;
static float accel_scale_factor;

/* Static Functions */
// static void cs_high();
// static void cs_low();

static void select_user_bank(userbank ub);

static uint8_t read_single_icm20948_reg(userbank ub, uint8_t reg);
static void write_single_icm20948_reg(userbank ub, uint8_t reg, uint8_t val);
static uint8_t *read_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t len);
static void write_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t *val, uint8_t len);

static uint8_t read_single_ak09916_reg(uint8_t reg);
static void write_single_ak09916_reg(uint8_t reg, uint8_t val);
static uint8_t *read_multiple_ak09916_reg(uint8_t reg, uint8_t len);

/* Main Functions */
// Отключение SPI и включение I2C
void enable_i2c_mode(void)
{
    // Запись 0x00 в регистр USER_CTRL (бит I2C_IF_DIS = 0 для включения I2C)
    write_single_icm20948_reg(ub_0, 0x03, 0x00);
}

// Функция для сканирования устройств на шине I2C
// Параметры:
// - hi2c: Указатель на I2C handle, настроенный в STM32CubeMX (например, &hi2c1)
// Возвращает: Ничего (выводит результаты в UART или другой интерфейс для отладки)
void I2C_ScanDevices(I2C_HandleTypeDef *hi2c)
{
    // Переменная для отслеживания статуса операции I2C
    HAL_StatusTypeDef status;
    
    // Буфер для тестового запроса (пустой, так как нам нужен только ответ на адрес)
    uint8_t dummy = 0;
    
    // Вывод начального сообщения для отладки
    // Используется UART или другой интерфейс для вывода результатов
    printf("Scanning I2C bus...\n");
    
    // Цикл по всем возможным 7-битным адресам I2C (0x00–0x7F)
    // Адреса 0x00–0x07 и 0x78–0x7F зарезервированы в I2C, но мы проверяем их для полноты
    for (uint8_t addr = 0; addr <= 0x7F; addr++)
    {
        // Формирование 8-битного адреса устройства (7-битный адрес, сдвинутый влево, с битом R/W=0)
        // HAL ожидает 8-битный адрес (например, 0x68 << 1 = 0xD0 для записи)
        uint16_t device_addr = addr << 1;
        
        // Попытка отправить тестовый запрос на адрес устройства
        // HAL_I2C_Master_Transmit используется для проверки, отвечает ли устройство
        // Параметры: I2C handle, адрес устройства, буфер (пустой), длина (0 или 1), тайм-аут (10 мс)
        status = HAL_I2C_Master_Transmit(hi2c, device_addr, &dummy, 0, 50);
        
        // Проверка статуса: если HAL_OK, устройство ответило (ACK)
        if (status == HAL_OK)
        {
            // Вывод найденного адреса в формате 7-битного адреса
            printf("I2C device found at address: 0x%02X (7-bit)\n", addr);
        }
        // Задержка для предотвращения перегрузки шины I2C
        // Небольшая задержка (1 мс) даёт шине время на восстановление
        HAL_Delay(1);
    }
    
    // Вывод завершающего сообщения
    printf("I2C scan completed.\n");
	HAL_Delay(5000);
}



void icm20948_init()
{
	DEBUG_PRINTF("+++ icm20948_init \n");
	icm20948_device_reset();
	icm20948_who_am_i();
	// enable_i2c_mode();					   // Отключение SPI и включение I2C

	icm20948_clock_source(1); // Используется для выбора лучшего источника тактирования (обычно авто-выбор для минимального дрейфа)
	// icm20948_odr_align_enable(); // Используется для синхронизации частоты вывода данных (ODR) между акселерометром и гироскопом

	// icm20948_spi_slave_enable(); Мне не нужно

	icm20948_gyro_low_pass_filter(0); // Настройка низкочастотного фильтра для гироскопа //  - config: Значение от 0 до 7, задающее частоту среза фильтра (см. datasheet, раздел 6.8) // Используется для уменьшения шума в данных гироскопа
	icm20948_accel_low_pass_filter(0); // Настройка низкочастотного фильтра для акселрометра //  - config: Значение от 0 до 7, задающее частоту среза фильтра (см. datasheet, раздел 6.8) // Используется для уменьшения шума в данных гироскопа

	icm20948_gyro_sample_rate_divider(0); // Установка делителя частоты дискретизации для гироскопа // - divider: Значение делителя (Output Data Rate = 1.125 кГц / (1 + divider))// Используется для настройки частоты вывода данных гироскопа (например, 100 Гц при divider = 10)
	icm20948_accel_sample_rate_divider(0); // Установка делителя частоты дискретизации для акселрометра  //  - divider: Значение делителя (Output Data Rate = 1.125 кГц / (1 + divider)) // Используется для настройки частоты вывода данных гироскопа (например, 100 Гц при divider = 10)

	// icm20948_gyro_calibration(); 
	// icm20948_accel_calibration();

	icm20948_gyro_full_scale_select(_250dps); // Выбор полной шкалы для гироскопа //  - full_scale: Значение шкалы (например, ±250 dps, ±500 dps, см. datasheet, раздел 6.8) // Используется для установки диапазона измерений гироскопа (влияет на чувствительность)
	icm20948_accel_full_scale_select(_2g); // Выбор полной шкалы для акселерометра  // - full_scale: Значение шкалы (например, ±2g, ±4g, см. datasheet, раздел 6.8) // Используется для установки диапазона измерений акселерометра (влияет на чувствительность)

	icm20948_wakeup();
	DEBUG_PRINTF("    End icm20948_init \n");
	HAL_Delay(1000);
}

void ak09916_init()
{
	DEBUG_PRINTF("+++ ak09916_init \n");

	icm20948_bypass_en();
	// icm20948_i2c_master_reset();
    // HAL_Delay(100); // Задержка 50 мс после сброса
	// icm20948_i2c_master_enable();
    // HAL_Delay(100); // Задержка 50 мс после сброса
	// icm20948_i2c_master_clk_frq(9);
    // HAL_Delay(100); // Задержка 50 мс после сброса

    // ak09916_soft_reset(); // Сброс магнитометра
  // Включение I2C-мастера и инициализация AK09916
  // icm20948_i2c_master_reset(); // Сброс I2C-мастера
  // icm20948_i2c_master_enable(); // Включение I2C-мастера
  // icm20948_i2c_master_clk_frq(7); // Частота ~400 кГц


    // HAL_Delay(100); // Задержка 50 мс после сброса
	// uint8_t i2c_mst_status = read_single_icm20948_reg(ub_0, 0x17); // I2C_MST_STATUS
    // printf("2 I2C_MST_STATUS: 0x%02X\n", i2c_mst_status); // Отладочный вывод

    // select_user_bank(ub_3); // Переключение на банкCorrelation: 0x00
    // write_single_icm20948_reg(ub_3, 0x31, 0x0C | 0x01); // SLV0_ADDR: адрес AK09916 (write)
    // write_single_icm20948_reg(ub_3, 0x32, 0x31); // SLV0_REG: регистр CNTL2
    // write_single_icm20948_reg(ub_3, 0x33, 0x80 | 0x01); // SLV0_CTRL: включение, 1 байт
    // write_single_icm20948_reg(ub_3, 0x34, 0x08); // SLV0_DO: непрерывный режим, 100 Гц
    // HAL_Delay(100); // Задержка для завершения

	// i2c_mst_status = read_single_icm20948_reg(ub_0, 0x17); // I2C_MST_STATUS
    // printf("2 I2C_MST_STATUS: 0x%02X\n", i2c_mst_status); // Отладочный вывод

	// select_user_bank(ub_3); // Переключение на банк 3
    // write_single_icm20948_reg(ub_3, 0x31, 0x0C); // SLV0_ADDR: адрес AK09916 (write)
    // write_single_icm20948_reg(ub_3, 0x32, 0x01); // SLV0_REG: регистр WIA2 (0x01)
    // write_single_icm20948_reg(ub_3, 0x33, 0x80 | 0x01); // SLV0_CTRL: включение, 1 байт
    // HAL_Delay(10); // Задержка для завершения операции
    // uint8_t wia2 = read_single_icm20948_reg(ub_3, 0x01); // Чтение EXT_SLV_SENS_DATA_00
    // printf("AK09916 WIA2: 0x%02X\n", wia2); // Отладочный вывод значения WIA2

	// select_user_bank(ub_3); // Переключение на банк 3
    // write_single_icm20948_reg(ub_3, 0x31, 0x0C); // SLV0_ADDR: адрес AK09916 (read)
    // write_single_icm20948_reg(ub_3, 0x32, 0x01); // SLV0_REG: регистр WIA2
    // write_single_icm20948_reg(ub_3, 0x33, 0x80 | 0x01); // SLV0_CTRL: включение, 1 байт
    // HAL_Delay(20); // Задержка 20 мс
    
	// wia2 = read_single_icm20948_reg(ub_3, 0x01); // Чтение EXT_SLV_SENS_DATA_00
    // printf("AK09916 WIA2: 0x%02X\n", wia2); // Вывод значения WIA2
    // uint8_t status = read_single_icm20948_reg(ub_0, 0x17); // Чтение I2C_MST_STATUS
    // printf("I2C_MST_STATUS: 0x%02X\n", status); // Вывод статуса


	// i2c_mst_status = read_single_icm20948_reg(ub_0, 0x17); // I2C_MST_STATUS
    // printf("3 I2C_MST_STATUS: 0x%02X\n", i2c_mst_status); // Отладочный вывод

	// while (!ak09916_who_am_i())
	// 	;
	// ak09916_who_am_i();
	// ak09916_soft_reset();
	// ak09916_operation_mode_setting(continuous_measurement_100hz);

	  // // Проверка связи с AK09916
  // if (ak09916_who_am_i()) {
  //     printf("AK09916 detected: WHO_AM_I = 0x09\n");
  // } else {
  //     printf("AK09916 not detected\n");
  // }
  
	DEBUG_PRINTF("    End ak09916_init \n");
}

void icm20948_gyro_read(axises *data)
{
	uint8_t *temp = read_multiple_icm20948_reg(ub_0, B0_GYRO_XOUT_H, 6);

	data->x = (int16_t)(temp[0] << 8 | temp[1]);
	data->y = (int16_t)(temp[2] << 8 | temp[3]);
	data->z = (int16_t)(temp[4] << 8 | temp[5]);
}

void icm20948_accel_read(axises *data)
{
	uint8_t *temp = read_multiple_icm20948_reg(ub_0, B0_ACCEL_XOUT_H, 6);

	data->x = (int16_t)(temp[0] << 8 | temp[1]);
	data->y = (int16_t)(temp[2] << 8 | temp[3]);
	data->z = (int16_t)(temp[4] << 8 | temp[5]) + accel_scale_factor;
	// Add scale factor because calibraiton function offset gravity acceleration.
}

bool ak09916_mag_read(axises *data)
{
	uint8_t *temp;
	uint8_t drdy, hofl; // data ready, overflow

	drdy = read_single_ak09916_reg(MAG_ST1) & 0x01;
	if (!drdy)
		return false;

	temp = read_multiple_ak09916_reg(MAG_HXL, 6);

	hofl = read_single_ak09916_reg(MAG_ST2) & 0x08;
	if (hofl)
		return false;

	data->x = (int16_t)(temp[1] << 8 | temp[0]);
	data->y = (int16_t)(temp[3] << 8 | temp[2]);
	data->z = (int16_t)(temp[5] << 8 | temp[4]);

	return true;
}

void icm20948_gyro_read_dps(axises *data)
{
	icm20948_gyro_read(data);

	data->x /= gyro_scale_factor;
	data->y /= gyro_scale_factor;
	data->z /= gyro_scale_factor;
}

void icm20948_accel_read_g(axises *data)
{
	icm20948_accel_read(data);

	data->x /= accel_scale_factor;
	data->y /= accel_scale_factor;
	data->z /= accel_scale_factor;
}

bool ak09916_mag_read_uT(axises *data)
{
	axises temp;
	bool new_data = ak09916_mag_read(&temp);
	if (!new_data)
		return false;

	data->x = (float)(temp.x * 0.15);
	data->y = (float)(temp.y * 0.15);
	data->z = (float)(temp.z * 0.15);

	return true;
}

/* Sub Functions */
bool icm20948_who_am_i()
{
	uint8_t icm20948_id = read_single_icm20948_reg(ub_0, B0_WHO_AM_I);
	if (icm20948_id == ICM20948_ID)
	{
		DEBUG_PRINTF("    ICM-20948 connected successfully WHO_AM_I = 0x%02X\n", ICM20948_ID);
		return true;
	}
	else
	{
		DEBUG_PRINTF("    ICM-20948 connection failed: 0x%02X real= 0x%02X\n ", ICM20948_ID , icm20948_id);
		return false;
	}
}

bool ak09916_who_am_i()
{
	uint8_t ak09916_id = read_single_ak09916_reg(MAG_WIA2);

	if (ak09916_id == AK09916_ID)
	{
		DEBUG_PRINTF("    ak09916_who_am_i connected successfully 0x%02X \n", AK09916_ID);
		return true;
	}
	else
	{
		DEBUG_PRINTF("    ak09916_who_am_i connection failed: 0x%02X real= 0x%02X \n ", AK09916_ID , ak09916_id);
		return false;
	}
}

void icm20948_device_reset()
{
	// write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, 0x80 | 0x41);
	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, 0x80);
	DEBUG_PRINTF("+++ icm20948_device_reset \n");
	HAL_Delay(100);
}

// Включаем bypass режим для первоначальной настройки магнитометра
void icm20948_bypass_en()
{
	write_single_icm20948_reg(ub_0, B0_INT_PIN_CFG, 0x02);
	DEBUG_PRINTF("+++ icm20948_bypass_en \n");
	HAL_Delay(100);
}
// Выключаем bypass режим для первоначальной настройки магнитометра
void icm20948_bypass_disable()
{
	write_single_icm20948_reg(ub_0, B0_INT_PIN_CFG, 0x00);
	DEBUG_PRINTF("+++ icm20948_bypass_disable \n");
	HAL_Delay(100);
}

void ak09916_soft_reset()
{
	write_single_ak09916_reg(MAG_CNTL3, 0x01);
	HAL_Delay(100);

	select_user_bank(ub_0); // Переключение на банк 0
    uint8_t status = read_single_icm20948_reg(ub_0, 0x17); // Чтение I2C_MST_STATUS
    DEBUG_PRINTF("I2C_MST_STATUS after reset: 0x%02X\n", status); // Вывод статуса
}

void icm20948_wakeup()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
	new_val &= 0xBF;

	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, new_val);
	DEBUG_PRINTF("+++ icm20948_wakeup \n");
	HAL_Delay(100);
}

void icm20948_sleep()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
	new_val |= 0x40;

	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, new_val);
	HAL_Delay(100);
}

void icm20948_spi_slave_enable()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
	new_val |= 0x10;

	write_single_icm20948_reg(ub_0, B0_USER_CTRL, new_val);
}

// Сброс внутреннего I2C-мастера ICM-20948
void icm20948_i2c_master_reset() // Используется для перезапуска внутреннего I2C-мастера, который управляет связью с магнитометром AK09916
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
	new_val |= 0x02;

	write_single_icm20948_reg(ub_0, B0_USER_CTRL, new_val);
}

void icm20948_i2c_master_enable()
{
	// uint8_t new_val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
	// new_val |= 0x20;

	// write_single_icm20948_reg(ub_0, B0_USER_CTRL, new_val);
	// HAL_Delay(100);

    select_user_bank(ub_0); // Переключение на банк 0
    uint8_t user_ctrl = read_single_icm20948_reg(ub_0, 0x03); // Чтение текущего USER_CTRL
	printf("0 USER_CTRL: 0x%02X\n", user_ctrl); // Вывод значения USER_CTRL
    user_ctrl |= 0x20; // Установка бита I2C_MST_EN (бит 5)
    write_single_icm20948_reg(ub_0, 0x03, user_ctrl); // Запись в USER_CTRL
	user_ctrl = read_single_icm20948_reg(ub_0, 0x03); // Чтение USER_CTRL
	printf("1 USER_CTRL: 0x%02X\n", user_ctrl); // Вывод значения USER_CTRL

}

void icm20948_i2c_master_clk_frq(uint8_t config)
{
	// uint8_t new_val = read_single_icm20948_reg(ub_3, B3_I2C_MST_CTRL);
	// new_val |= config;

	// write_single_icm20948_reg(ub_3, B3_I2C_MST_CTRL, new_val);

	select_user_bank(ub_3); // Переключение на банк 3
    uint8_t mst_ctrl = read_single_icm20948_reg(ub_3, 0x07); // Чтение I2C_MST_CTRL
    mst_ctrl &= 0xF0; // Очистка битов I2C_MST_CLK
    mst_ctrl |= (config & 0x0F); // Установка частоты (0–15)
    write_single_icm20948_reg(ub_3, 0x07, mst_ctrl); // Запись в I2C_MST_CTRL
	
	mst_ctrl = read_single_icm20948_reg(ub_3, 0x07); // Чтение I2C_MST_CTRL
    printf("I2C_MST_CTRL: 0x%02X\n", mst_ctrl); // Вывод I2C_MST_CTRL
}

void icm20948_clock_source(uint8_t source)
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
	new_val |= source;

	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, new_val);
	DEBUG_PRINTF("+++ icm20948_clock_source \n");
}

void icm20948_odr_align_enable()
{
	write_single_icm20948_reg(ub_2, B2_ODR_ALIGN_EN, 0x01);
}

void icm20948_gyro_low_pass_filter(uint8_t config)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1);
	new_val |= config << 3;

	write_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1, new_val);
}

void icm20948_accel_low_pass_filter(uint8_t config)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG);
	new_val |= config << 3;

	write_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1, new_val);
}

void icm20948_gyro_sample_rate_divider(uint8_t divider)
{
	write_single_icm20948_reg(ub_2, B2_GYRO_SMPLRT_DIV, divider);
}

void icm20948_accel_sample_rate_divider(uint16_t divider)
{
	uint8_t divider_1 = (uint8_t)(divider >> 8);
	uint8_t divider_2 = (uint8_t)(0x0F & divider);

	write_single_icm20948_reg(ub_2, B2_ACCEL_SMPLRT_DIV_1, divider_1);
	write_single_icm20948_reg(ub_2, B2_ACCEL_SMPLRT_DIV_2, divider_2);
}

void ak09916_operation_mode_setting(operation_mode mode)
{
	write_single_ak09916_reg(MAG_CNTL2, mode);
	HAL_Delay(100);
}

void icm20948_gyro_calibration()
{
	axises temp;
	int32_t gyro_bias[3] = {0};
	uint8_t gyro_offset[6] = {0};

	for (int i = 0; i < 100; i++)
	{
		icm20948_gyro_read(&temp);
		gyro_bias[0] += temp.x;
		gyro_bias[1] += temp.y;
		gyro_bias[2] += temp.z;
	}

	gyro_bias[0] /= 100;
	gyro_bias[1] /= 100;
	gyro_bias[2] /= 100;

	// Construct the gyro biases for push to the hardware gyro bias registers,
	// which are reset to zero upon device startup.
	// Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format.
	// Biases are additive, so change sign on calculated average gyro biases
	gyro_offset[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF;
	gyro_offset[1] = (-gyro_bias[0] / 4) & 0xFF;
	gyro_offset[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
	gyro_offset[3] = (-gyro_bias[1] / 4) & 0xFF;
	gyro_offset[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
	gyro_offset[5] = (-gyro_bias[2] / 4) & 0xFF;

	write_multiple_icm20948_reg(ub_2, B2_XG_OFFS_USRH, gyro_offset, 6);
}

void icm20948_accel_calibration()
{
	axises temp;
	uint8_t *temp2;
	uint8_t *temp3;
	uint8_t *temp4;

	int32_t accel_bias[3] = {0};
	int32_t accel_bias_reg[3] = {0};
	uint8_t accel_offset[6] = {0};

	for (int i = 0; i < 100; i++)
	{
		icm20948_accel_read(&temp);
		accel_bias[0] += temp.x;
		accel_bias[1] += temp.y;
		accel_bias[2] += temp.z;
	}

	accel_bias[0] /= 100;
	accel_bias[1] /= 100;
	accel_bias[2] /= 100;

	uint8_t mask_bit[3] = {0, 0, 0};

	temp2 = read_multiple_icm20948_reg(ub_1, B1_XA_OFFS_H, 2);
	accel_bias_reg[0] = (int32_t)(temp2[0] << 8 | temp2[1]);
	mask_bit[0] = temp2[1] & 0x01;

	temp3 = read_multiple_icm20948_reg(ub_1, B1_YA_OFFS_H, 2);
	accel_bias_reg[1] = (int32_t)(temp3[0] << 8 | temp3[1]);
	mask_bit[1] = temp3[1] & 0x01;

	temp4 = read_multiple_icm20948_reg(ub_1, B1_ZA_OFFS_H, 2);
	accel_bias_reg[2] = (int32_t)(temp4[0] << 8 | temp4[1]);
	mask_bit[2] = temp4[1] & 0x01;

	accel_bias_reg[0] -= (accel_bias[0] / 8);
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	accel_offset[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	accel_offset[1] = (accel_bias_reg[0]) & 0xFE;
	accel_offset[1] = accel_offset[1] | mask_bit[0];

	accel_offset[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	accel_offset[3] = (accel_bias_reg[1]) & 0xFE;
	accel_offset[3] = accel_offset[3] | mask_bit[1];

	accel_offset[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	accel_offset[5] = (accel_bias_reg[2]) & 0xFE;
	accel_offset[5] = accel_offset[5] | mask_bit[2];

	write_multiple_icm20948_reg(ub_1, B1_XA_OFFS_H, &accel_offset[0], 2);
	write_multiple_icm20948_reg(ub_1, B1_YA_OFFS_H, &accel_offset[2], 2);
	write_multiple_icm20948_reg(ub_1, B1_ZA_OFFS_H, &accel_offset[4], 2);
}

void icm20948_gyro_full_scale_select(gyro_full_scale full_scale)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1);

	switch (full_scale)
	{
	case _250dps:
		new_val |= 0x00;
		gyro_scale_factor = 131.0;
		break;
	case _500dps:
		new_val |= 0x02;
		gyro_scale_factor = 65.5;
		break;
	case _1000dps:
		new_val |= 0x04;
		gyro_scale_factor = 32.8;
		break;
	case _2000dps:
		new_val |= 0x06;
		gyro_scale_factor = 16.4;
		break;
	}

	write_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1, new_val);
}

void icm20948_accel_full_scale_select(accel_full_scale full_scale)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG);

	switch (full_scale)
	{
	case _2g:
		new_val |= 0x00;
		accel_scale_factor = 16384;
		break;
	case _4g:
		new_val |= 0x02;
		accel_scale_factor = 8192;
		break;
	case _8g:
		new_val |= 0x04;
		accel_scale_factor = 4096;
		break;
	case _16g:
		new_val |= 0x06;
		accel_scale_factor = 2048;
		break;
	}

	write_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG, new_val);
}

/* Static Functions */
// Вариант для I2C
static void select_user_bank(userbank ub)
{
    // Формирование значения для регистра REG_BANK_SEL
    // Значение банка (ub) сдвигается на 4 бита влево, так как биты [5:4] в REG_BANK_SEL задают номер банка
    // Например, ub_0 = 0x00, ub_1 = 0x10, ub_2 = 0x20, ub_3 = 0x30
    uint8_t bank = (ub << 4);
    
    // Создание буфера для передачи: первый байт — адрес регистра REG_BANK_SEL, второй — значение банка
    // REG_BANK_SEL находится по адресу 0x7F в банке 0
    uint8_t data[2];
    data[0] = REG_BANK_SEL; // Адрес регистра REG_BANK_SEL (0x7F)
    data[1] = bank;            // Значение для записи (номер банка)
    
    // Выполнение передачи данных по I2C
    // HAL_I2C_Master_Transmit отправляет данные на устройство с адресом ICM20948_I2C_ADDRESS
    // Параметры: I2C handle, адрес устройства, буфер данных, длина (2 байта), тайм-аут (100 мс)
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(ICM20948_I2C, ICM20948_I2C_ADDRESS, data, 2, 100);
    
    // Проверка статуса передачи для отладки
    // Если передача не удалась (например, устройство не отвечает), выводим ошибку
    if (status != HAL_OK) {
        // Для отладки можно вывести сообщение об ошибке через UART
        DEBUG_PRINTF("I2C select_user_bank error: %d\n", status);
    }
}

static void write_single_icm20948_reg(userbank ub, uint8_t reg, uint8_t val)
{
	// Выбор банка регистров (0, 1, 2 или 3) для доступа к нужному регистру
	// ICM-20948 использует банки регистров для организации своих настроек
	select_user_bank(ub);

	// Создание буфера для передачи: первый байт — адрес регистра, второй — значение для записи
	// В I2C для записи в регистр сначала отправляется адрес регистра, затем данные
	uint8_t data[2];
	data[0] = reg; // Адрес регистра, в который будет произведена запись
	data[1] = val; // Значение, которое нужно записать в регистр

	// Выполнение передачи данных по I2C
	// HAL_I2C_Master_Transmit отправляет данные на устройство с адресом ICM20948_I2C_ADDRESS
	// Параметры: I2C handle, адрес устройства, буфер данных, длина (2 байта), тайм-аут (100 мс)
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(ICM20948_I2C, ICM20948_I2C_ADDRESS, data, 2, 100);

	// Проверка статуса передачи для отладки
	// Если передача не удалась (например, устройство не отвечает), можно добавить обработку ошибки
	if (status != HAL_OK)
	{
		// Для отладки можно вывести ошибку через UART или зажечь светодиод
		DEBUG_PRINTF("I2C read_single_icm20948_reg write error: %d\n", status);
	}
}

static uint8_t read_single_icm20948_reg(userbank ub, uint8_t reg)
{
	// Выбор банка регистров для доступа к нужному регистру
	// Это необходимо, так как ICM-20948 организует регистры по банкам
	select_user_bank(ub);

	// Буфер для хранения прочитанного значения (1 байт)
	uint8_t reg_val;

	// Чтение данных из регистра через I2C
	// HAL_I2C_Mem_Read отправляет адрес регистра и читает 1 байт данных
	// Параметры: I2C handle, адрес устройства, адрес регистра, размер адреса (8 бит), буфер для данных, длина (1 байт), тайм-аут (100 мс)
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(ICM20948_I2C, ICM20948_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &reg_val, 1, 100);

	// Проверка статуса чтения для отладки
	// Если чтение не удалось, возвращаем 0 или можно добавить обработку ошибки
	if (status != HAL_OK)
	{
		DEBUG_PRINTF("I2C read_single_icm20948_reg read error: %d\n", status);
		return 0; // Возвращаем 0 в случае ошибки (можно изменить на другой подход)
	}

	// Возврат прочитанного значения
	return reg_val;
}

static uint8_t *read_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t len)
{
	// Выбор банка регистров для доступа к нужным регистрам
	// Это необходимо перед началом чтения
	select_user_bank(ub);

	// Статический буфер для хранения прочитанных данных
	// Размер 22 байта достаточен для большинства операций чтения (например, 6 байт акселерометра + 6 байт гироскопа + запас)
	static uint8_t reg_val[22];

	// Очистка буфера перед чтением для предотвращения использования старых данных
	memset(reg_val, 0, sizeof(reg_val));

	// Чтение нескольких байт из регистра через I2C
	// HAL_I2C_Mem_Read отправляет адрес начального регистра и читает len байт
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(ICM20948_I2C, ICM20948_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, reg_val, len, 100);

	// Проверка статуса чтения для отладки
	if (status != HAL_OK)
	{
		DEBUG_PRINTF("I2C read_multiple_icm20948_reg error: %d\n", status);
		// Возвращаем указатель на пустой буфер в случае ошибки
		return reg_val;
	}

	// Возврат указателя на буфер с прочитанными данными
	return reg_val;
}

static void write_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t *val, uint8_t len)
{
	// Выбор банка регистров для доступа к нужным регистрам
	select_user_bank(ub);

	// Создание буфера для передачи: первый байт — адрес начального регистра, затем данные
	uint8_t data[len + 1];
	data[0] = reg;				// Адрес начального регистра
	memcpy(&data[1], val, len); // Копирование входных данных в буфер после адреса регистра

	// Выполнение передачи данных по I2C
	// HAL_I2C_Master_Transmit отправляет адрес регистра и данные
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(ICM20948_I2C, ICM20948_I2C_ADDRESS, data, len + 1, 100);

	// Проверка статуса передачи для отладки
	if (status != HAL_OK)
	{
		DEBUG_PRINTF("I2C write_multiple_icm20948_reg error: %d\n", status);
	}
}

// Вариант для SPI

// static void cs_high()
// {
// 	HAL_GPIO_WritePin(ICM20948_SPI_CS_PIN_PORT, ICM20948_SPI_CS_PIN_NUMBER, SET);
// }

// static void cs_low()
// {
// 	HAL_GPIO_WritePin(ICM20948_SPI_CS_PIN_PORT, ICM20948_SPI_CS_PIN_NUMBER, RESET);
// }

// static void select_user_bank(userbank ub)
// {
// 	uint8_t write_reg[2];
// 	write_reg[0] = WRITE | REG_BANK_SEL;
// 	write_reg[1] = ub;

// 	cs_low();
// 	HAL_SPI_Transmit(ICM20948_SPI, write_reg, 2, 10);
// 	cs_high();
// }

// static uint8_t read_single_icm20948_reg(userbank ub, uint8_t reg)
// {
// 	uint8_t read_reg = READ | reg;
// 	uint8_t reg_val;
// 	select_user_bank(ub);

// 	cs_low();
// 	HAL_SPI_Transmit(ICM20948_SPI, &read_reg, 1, 1000);
// 	HAL_SPI_Receive(ICM20948_SPI, &reg_val, 1, 1000);
// 	cs_high();

// 	return reg_val;
// }

// static void write_single_icm20948_reg(userbank ub, uint8_t reg, uint8_t val)
// {
// 	uint8_t write_reg[2];
// 	write_reg[0] = WRITE | reg;
// 	write_reg[1] = val;

// 	select_user_bank(ub);

// 	cs_low();
// 	HAL_SPI_Transmit(ICM20948_SPI, write_reg, 2, 1000);
// 	cs_high();
// }

// static uint8_t* read_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t len)
// {
// 	uint8_t read_reg = READ | reg;
// 	static uint8_t reg_val[6];
// 	select_user_bank(ub);

// 	cs_low();
// 	HAL_SPI_Transmit(ICM20948_SPI, &read_reg, 1, 1000);
// 	HAL_SPI_Receive(ICM20948_SPI, reg_val, len, 1000);
// 	cs_high();

// 	return reg_val;
// }

// static void write_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t* val, uint8_t len)
// {
// 	uint8_t write_reg = WRITE | reg;
// 	select_user_bank(ub);

// 	cs_low();
// 	HAL_SPI_Transmit(ICM20948_SPI, &write_reg, 1, 1000);
// 	HAL_SPI_Transmit(ICM20948_SPI, val, len, 1000);
// 	cs_high();
// }

static uint8_t read_single_ak09916_reg(uint8_t reg)
{
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG, reg);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x81);

	HAL_Delay(1);
	return read_single_icm20948_reg(ub_0, B0_EXT_SLV_SENS_DATA_00);
}

static void write_single_ak09916_reg(uint8_t reg, uint8_t val)
{
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, WRITE | MAG_SLAVE_ADDR);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG, reg);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_DO, val);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x81);
}

static uint8_t *read_multiple_ak09916_reg(uint8_t reg, uint8_t len)
{
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG, reg);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x80 | len);

	HAL_Delay(1);
	return read_multiple_icm20948_reg(ub_0, B0_EXT_SLV_SENS_DATA_00, len);
}