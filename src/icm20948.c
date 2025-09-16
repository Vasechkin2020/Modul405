/*
 * icm20948.c
 *
 *  Created on: Dec 26, 2020
 *      Author: mokhwasomssi
 */

#include "icm20948.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

static float gyro_scale_factor;
static float accel_scale_factor;

#include "..\lib\FATFS\fatfs.h"
extern FRESULT writeUint8ToFile(uint8_t *values, uint8_t size, const char *filename);  // Функция записи массива uint8_t в файл
extern FRESULT writeFloatToFile(float *values, uint8_t size, const char *filename);	   // Функция записи массива float в файл
extern FRESULT readUint8FromFile(uint8_t *values, uint8_t size, const char *filename); // Функция считывания uint8_t из файла в массив
extern FRESULT readFloatFromFile(float *values, uint8_t size, const char *filename);   // Функция считывания float из файла в массив

// Калибровочные коэффициенты
StructBias gBias; // для гироскопа
StructBias aBias; // для акселерометра
StructBias mBias; // для магнетрометра

StructScale aScale; // для акселерометра масштабные коефициенты
StructScale mScale; // для магнетрометра масштабные коефициенты

/* Static Functions */
// static void cs_high();
// static void cs_low();

static void select_user_bank(userbank ub);

static uint8_t read_single_icm20948_reg(userbank ub, uint8_t reg);							  // Чтение одного регистра ICM20948
static void write_single_icm20948_reg(userbank ub, uint8_t reg, uint8_t val);				  // Запись одного регистра ICM20948
static void write_single_icm20948_reg2(uint8_t reg, uint8_t val);							  // Запись одного регистра ICM20948 без указания userbank
static uint8_t *read_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t len);			  // Чтение нескольких регистров ICM20948
static void write_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t *val, uint8_t len); // Запись нескольких регистров ICM20948

static uint8_t read_single_ak09916_reg(uint8_t reg);			// Чтение одного регистра AK09916
static void write_single_ak09916_reg(uint8_t reg, uint8_t val); //	 Запись одного регистра AK09916
// static uint8_t *read_multiple_ak09916_reg(uint8_t reg, uint8_t len);

void calibrate_accelerometer(void); // Функция калибровки (шестипозиционная)

extern void print_binary(int num);

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
	HAL_Delay(2000);
}

void icm20948_init()
{
	printf("+++ icm20948_init \n");
	icm20948_device_reset();
	icm20948_who_am_i();
	ICM20948_DisableLPMMode(); // Отключение режима низкого энергопотребления (LPM) для нормальной работы датчика
	HAL_Delay(500);			   // Задержка

	// enable_i2c_mode();					   // Отключение SPI и включение I2C

	// Не нужно уже в LP все выбрали icm20948_clock_source(1);	 // Используется для выбора лучшего источника тактирования (обычно авто-выбор для минимального дрейфа)
	icm20948_odr_align_enable(); // Используется для синхронизации частоты вывода данных (ODR) между акселерометром и гироскопом

	// icm20948_spi_slave_enable(); Мне не нужно

	//*********************** ГИРОСКОП **************************
	/*
	ICM-20948 настройки:Гироскоп: GYRO_DLPFCFG = 5 (3DB BW = 11,6 Гц, NBW = 17,8 Гц), FSR = ±250 dps, ODR ≈ 102,27 Гц (GYRO_SMPLRT_DIV = 10).
	Акселерометр: ACCEL_DLPFCFG = 5 (3DB BW = 11,5 Гц, NBW = 13,1 Гц), FSR = ±2g, ODR ≈ 102,27 Гц (ACCEL_SMPLRT_DIV = 10).
	*/
	/*Ваша максимальная скорость поворота — 360 градусов за 2 секунды. Давайте определим, как это переводится в частоту (в герцах), чтобы понять, подходит ли выбранная настройка.Угловая скорость:Угловая скорость=360∘2 с=180∘
	Это соответствует 180 градусов в секунду, что укладывается в выбранную чувствительность гироскопа 250 dps (±250°/с).
	Частота вращения в герцах:
	Частота в герцах (Гц) для вращения связана с периодом одного полного оборота (360°). Если полный оборот занимает 2 секунды, то:Частота=1Период=12 с=0,5 Гц\text{Частота} = \frac{1}{\text{Период}} = \frac{1}{2 \, \text{с}} = 0,5 \, \text{Гц}\text{Частота} = \frac{1}{\text{Период}} = \frac{1}{2 \, \text{с}} = 0,5 \, \text{Гц}
	Это означает, что основная частота вашего вращения — 0,5 Гц (один полный оборот каждые 2 секунды).*/
	icm20948_gyro_full_scale_select(_250dps); // Выбор полной шкалы для гироскопа //  - full_scale: Значение шкалы (например, ±250 dps, ±500 dps, см. datasheet, раздел 6.8) // Используется для установки диапазона измерений гироскопа (влияет на чувствительность)

	/*Установите GYRO_DLPFCFG = 5:
	3DB BW = 11,6 Гц идеально подходит для захвата сигналов ваших медленных движений (0–5 Гц, с запасом до 10 Гц).
	NBW = 17,8 Гц обеспечивает отличное подавление шума, что повышает точность данных, особенно при высокой чувствительности (250 dps).
	Частота подготовки данных (RATE) при GYRO_SMPLRT_DIV ≤ 10 (например, ~102,27 Гц при GYRO_SMPLRT_DIV = 10) совместима с вашим опросом 100 Гц.*/
	icm20948_gyro_low_pass_filter(5); // Настройка низкочастотного фильтра для гироскопа //  - config: Значение от 0 до 7, задающее частоту среза фильтра (см. datasheet, раздел 6.8) // Используется для уменьшения шума в данных гироскопа

	icm20948_gyro_sample_rate_divider(10); // Установка делителя частоты дискретизации для гироскопа // - divider: Значение делителя (Output Data Rate = 1.125 кГц / (1 + divider))// Используется для настройки частоты вывода данных гироскопа (например, 100 Гц при divider = 10)

	//*********************** АКСЕЛЬРОМЕТР**************************
	icm20948_accel_full_scale_select(_2g);	// Выбор полной шкалы для акселерометра  // - full_scale: Значение шкалы (например, ±2g, ±4g, см. datasheet, раздел 6.8) // Используется для установки диапазона измерений акселерометра (влияет на чувствительность)
	icm20948_accel_low_pass_filter(5);		// Настройка низкочастотного фильтра для акселрометра //  - config: Значение от 0 до 7, задающее частоту среза фильтра (см. datasheet, раздел 6.8) // Используется для уменьшения шума в данных гироскопа
	icm20948_accel_sample_rate_divider(10); // Установка делителя частоты дискретизации для акселрометра  //  - divider: Значение делителя (Output Data Rate = 1.125 кГц / (1 + divider)) // Используется для настройки частоты вывода данных гироскопа (например, 100 Гц при divider = 10)

	icm20948_wakeup();

	float icm20948OffSet[9] = {0.0}; // Массив с 9 значениями калибровки датчика

	// icm20948_accel_calibration(); // Старая версия калибровки акселерометра, которая не учитывает смещение и масштабирование

	printf("    readFloatFromFile icm20948.cfg \n");
	readFloatFromFile(icm20948OffSet, 9, "icm20948.cfg");

	// icm20948_gyro_calibration();

	// printf("    writeFloatToFile icm20948.cfg \n");
	// icm20948OffSet[0] = gBias.b_x;
	// icm20948OffSet[1] = gBias.b_y;
	// icm20948OffSet[2] = gBias.b_z;
	// printf("gBias.b_x= %.3f gBias.b_y= %.3f gBias.b_z= %.3f | ", gBias.b_x, gBias.b_y, gBias.b_z);

	// calibrate_accelerometer();

	// icm20948OffSet[3] = aBias.b_x;
	// icm20948OffSet[4] = aBias.b_y;
	// icm20948OffSet[5] = aBias.b_z;
	// printf("aBias.b_x= %.3f aBias.b_y= %.3f aBias.b_z= %.3f | ", aBias.b_x, aBias.b_y, aBias.b_z);

	// icm20948OffSet[6] = aScale.s_x;
	// icm20948OffSet[7] = aScale.s_y;
	// icm20948OffSet[8] = aScale.s_z;
	// printf("aScale.b_x= %.3f aScale.b_y= %.3f aScale.b_z= %.3f \n", aScale.s_x, aScale.s_y, aScale.s_z);

	// Ручной подбор для гироскопа глядя на плотджагер
	icm20948OffSet[0] = 8;	 // 15
	icm20948OffSet[1] = 235; // 245
	icm20948OffSet[2] = 60;	 // 66

	writeFloatToFile(icm20948OffSet, 9, "icm20948.cfg");
	// while (1)
	// {
	// }

	gBias.b_x = icm20948OffSet[0];
	gBias.b_y = icm20948OffSet[1];
	gBias.b_z = icm20948OffSet[2];
	printf("gBias.b_x= %.3f gBias.b_y= %.3f gBias.b_z= %.3f | ", gBias.b_x, gBias.b_y, gBias.b_z);

	aBias.b_x = icm20948OffSet[3];
	aBias.b_y = icm20948OffSet[4];
	aBias.b_z = icm20948OffSet[5];
	printf("aBias.b_x= %.3f aBias.b_y= %.3f aBias.b_z= %.3f | ", aBias.b_x, aBias.b_y, aBias.b_z);

	aScale.s_x = icm20948OffSet[6];
	aScale.s_y = icm20948OffSet[7];
	aScale.s_z = icm20948OffSet[8];
	printf("aScale.b_x= %.3f aScale.b_y= %.3f aScale.b_z= %.3f \n", aScale.s_x, aScale.s_y, aScale.s_z);

	printf("    End icm20948_init \n");
	HAL_Delay(2000);
}

void ak09916_init()
{
	printf("+++ ak09916_init \n");
	// write_single_icm20948_reg(ub_0, B0_LP_CONFIG, 0x40);		  // I2C_MST_CYCLE = 1
	// write_single_icm20948_reg(ub_3, B3_I2C_MST_ODR_CONFIG, 0x03); // ~134 Гц

	// Включение I2C-мастера и инициализация AK09916
	icm20948_i2c_master_reset();	// Непонеятно зачем тут он это написал.
	icm20948_i2c_master_clk_frq(7); // Частота ~400 кГц
	icm20948_i2c_master_enable();	// Включение I2C-мастера
	// icm20948_i2c_master_disable();

	// Настройки частоты с какой внутренний I2C опрашивает подчиненное утройство. Если не задат ьтоработает на частоте гироскопа в 1125 Герц
	write_single_icm20948_reg(ub_0, B0_LP_CONFIG, 0x40);		  // I2C_MST_CYCLE = 1
	write_single_icm20948_reg(ub_3, B3_I2C_MST_ODR_CONFIG, 0x01); //

	// icm20948_bypass_en(); // Эта функция включает напрямую доступ к магнетрометру через мультиплексор и можно микроконтроллером управлять магнетрометром
	// I2C_ScanDevices(&hi2c1);

	ak09916_who_am_i();
	ak09916_soft_reset();										  // Настройка магнитометра
	ak09916_operation_mode_setting(continuous_measurement_100hz); // Частота с которой магнетометр готовит данные
	// Настройки какое устройство и из какого регистра начиная считывать
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR); // AK09916, чтение
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG, MAG_ST1);				  // ST1  Читаем статус готовности данных и считвания вовремя
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x00);				  // Выключаем пока
	HAL_Delay(100);

	mBias.b_x = 0; // Значения перед калибровкой
	mBias.b_y = 0;
	mBias.b_z = 0;

	mScale.s_x = 1;
	mScale.s_y = 1;
	mScale.s_z = 1;

	// float FLT_MAX = 256;   // Максимально возможное значение заведомо большое
	// float MAX_Vibros = 84; // ВЫше этого значения считаем выбросом и не учитываем
	// //*******************
	// float x_max = -FLT_MAX, x_min = FLT_MAX;
	// float y_max = -FLT_MAX, y_min = FLT_MAX;
	// float z_max = -FLT_MAX, z_min = FLT_MAX;

	// for (size_t i = 0; i < 18000; i++)
	// {
	// 	ak09916_mag_read_uT(&my_mag);
	// 	DEBUG_PRINTF("Magn X= %.3f y= %.3f z= %.3f ", my_mag.x, my_mag.y, my_mag.z);
	// 	// DEBUG_PRINTF("%.1f %.1f %.1f \n", my_mag.x, my_mag.y, my_mag.z);

	// 	// Обновление максимумов и минимумов с ограничением и учетом выбросов
	// 	if (my_mag.x < MAX_Vibros && my_mag.x > -MAX_Vibros)
	// 	{
	// 		if (my_mag.x > x_max)
	// 			x_max = my_mag.x;
	// 		if (my_mag.x < x_min)
	// 			x_min = my_mag.x;
	// 	}
	// 	if (my_mag.y < MAX_Vibros && my_mag.y > -MAX_Vibros)
	// 	{
	// 		if (my_mag.y > y_max)
	// 			y_max = my_mag.y;
	// 		if (my_mag.y < y_min)
	// 			y_min = my_mag.y;
	// 	}
	// 	if (my_mag.z < MAX_Vibros && my_mag.z > -MAX_Vibros)
	// 	{
	// 		if (my_mag.z > z_max)
	// 			z_max = my_mag.z;
	// 		if (my_mag.z < z_min)
	// 			z_min = my_mag.z;
	// 	}
	// 	HAL_Delay(10);
	// DEBUG_PRINTF(" | %.3f  %.3f / ", x_max, x_min);
	// DEBUG_PRINTF(" %.3f  %.3f / ", y_max, y_min);
	// DEBUG_PRINTF(" %.3f  %.3f \n ", z_max, z_min);
	// }
	// // Расчет bias
	// mBias.b_x = (x_max + x_min) / 2.0f;
	// mBias.b_y = (y_max + y_min) / 2.0f;
	// mBias.b_z = (z_max + z_min) / 2.0f;

	// DEBUG_PRINTF("x_max= %.3f x_min= %.3f mBias.b_x= %.3f \n", x_max, x_min, mBias.b_x);
	// DEBUG_PRINTF("y_max= %.3f y_min= %.3f mBias.b_y= %.3f \n", y_max, y_min, mBias.b_y);
	// DEBUG_PRINTF("z_max= %.3f z_min= %.3f mBias.b_z= %.3f \n", z_max, z_min, mBias.b_z);
	// HAL_Delay(100000);

	mBias.b_x = 7.506; // Это средние или медианные значения на основании 8 калибровок и усреднения.
	mBias.b_y = 0.251;
	mBias.b_z = 35.245;

	mScale.s_x = 1.029; // Это средние или медианные значения на основании 8 калибровок и усреднения.
	mScale.s_y = 0.986;
	mScale.s_z = 0.987;

	printf("    End ak09916_init \n");
}

void icm20948_gyro_read(axises *data)
{
	uint8_t *temp = read_multiple_icm20948_reg(ub_0, B0_GYRO_XOUT_H, 6);

	data->x = (int16_t)(temp[0] << 8 | temp[1]);
	data->y = (int16_t)(temp[2] << 8 | temp[3]);
	data->z = (int16_t)(temp[4] << 8 | temp[5]);
}
// // Чтение необработанных данных акселерометра (16-битные значения с АЦП)
void icm20948_accel_read(axises *data)
{
	uint8_t *temp = read_multiple_icm20948_reg(ub_0, B0_ACCEL_XOUT_H, 6);
	data->x = (int16_t)(temp[0] << 8 | temp[1]);
	data->y = (int16_t)(temp[2] << 8 | temp[3]);
	data->z = (int16_t)(temp[4] << 8 | temp[5]);
}

bool ak09916_mag_read(axises *data)
{
	// DEBUG_PRINTF("+\n");
	// uint8_t *mag_data;
	// uint8_t drdy, hofl; // data ready, overflow

	// drdy = read_single_ak09916_reg(MAG_ST1) & 0x01;
	// if (drdy == 0x10) DEBUG_PRINTF("--- drdy error DOR (Data Overrun Error)! %u \n",drdy); // DOR bit turns to “1” when data has been skipped in Continuous measurement mode 1, 2, 3, 4. It returns to “0” when any one of ST2 register or measurement data register (HXL to TMPS) is read.

	// if (drdy == 0x00)
	// {
	// 	// DEBUG_PRINTF("--- drdy Data not Ready! %u \n",drdy);
	// 	DEBUG_PRINTF("drdy\n");
	// 	// print_binary(drdy);
	// 	return false;
	// }

	// temp = read_multiple_ak09916_reg(MAG_HXL, 6);
	// temp = read_multiple_icm20948_reg(ub_0, B0_EXT_SLV_SENS_DATA_01, 6);

	// // hofl = read_single_ak09916_reg(MAG_ST2) & 0x08;
	// // if (hofl)
	// // {
	// // 	DEBUG_PRINTF("--- hofl error Magnetic Sensor Overflow! \n");
	// // 	return false;
	// // }

	// data->x = (int16_t)(temp[1] << 8 | temp[0]);
	// data->y = (int16_t)(temp[3] << 8 | temp[2]);
	// data->z = (int16_t)(temp[5] << 8 | temp[4]);

	// uint8_t *mag_data = read_multiple_icm20948_reg(ub_0, B0_EXT_SLV_SENS_DATA_00, 9);

	// uint8_t st1 = mag_data[0]; // ST1
	// if (st1 == 0x02)
	// {
	// 	DEBUG_PRINTF("Data Overrun Error!\n");
	// 	return;
	// }
	// if (!(st1 != 0x01))
	// {
	// 	DEBUG_PRINTF("Data NOT READY !\n");
	// 	return; // Данные не готовы
	// }
	uint8_t *mag_data;
	uint8_t st1, st2;

	select_user_bank(ub_3);
	// write_single_icm20948_reg2(B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR);
	// write_single_icm20948_reg2(B3_I2C_SLV0_REG, MAG_ST1);
	write_single_icm20948_reg2(B3_I2C_SLV0_CTRL, 0x80 | 9); // Считываем 9 байт ST2 считываем

	HAL_Delay(1);
	mag_data = read_multiple_icm20948_reg(ub_0, B0_EXT_SLV_SENS_DATA_00, 9);

	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x00); // Выключаем опрос Снова перключаемся на 3 банк

	st1 = mag_data[0];
	// uint8_t drdy_bit = st1 & 0x01;		 // Бит 0 (DRDY)// Выделение битов ST1
	// uint8_t dor_bit = (st1 >> 1) & 0x01; // Бит 1 (DOR)
	// DEBUG_PRINTF("DRDY= %u DOR= %u | ", drdy_bit, dor_bit);
	// DEBUG_PRINTF("%u %u | ", drdy_bit, dor_bit);
	st2 = mag_data[8];
	uint8_t hofl_bit = (st2 >> 3) & 0x01; // Бит 3 (HOFL)// Проверка HOFL в ST2
	if (hofl_bit == 1)
	{
		printf("Magnetic Sensor Overflow!\n");
		return false; // Прерываем, данные недостоверны
	}

	// uint8_t st1 = read_single_ak09916_reg(MAG_ST1);
	// st1 = read_single_icm20948_reg(ub_0, B0_EXT_SLV_SENS_DATA_00);

	// HAL_Delay(1);															 // За это время там внутренний опрос успеет еще раз считать данные

	// if (drdy_bit == 1 || dor_bit == 1)	 // Проверка битов ST1 // ЕСли данные готовы или уже переполнились
	// {
	// 	// mag_data = read_multiple_ak09916_reg(MAG_ST1, 9); // Считываем 9 байт ST2 считываем
	// 	if (dor_bit == 1)
	// 		DEBUG_PRINTF(" (DOR = %u)!\n",dor_bit); // Можно учитывать что не успеваем считывать данные
	// }
	// else // Если новые данные не готовы по повторяем считывание через 1 милисекунду
	// {
	// 	// DEBUG_PRINTF("not Ready! DRDY = %u DOR = %u\n", drdy_bit, dor_bit);
	// 	// HAL_Delay(1);															 // За это время там внутренний опрос успеет еще раз считать данные
	// 	// mag_data = read_multiple_icm20948_reg(ub_0, B0_EXT_SLV_SENS_DATA_00, 9); // Вторая попытка
	// 	// st1 = mag_data[0];
	// 	// drdy_bit = st1 & 0x01; // Бит 0 (DRDY)
	// 	// if (drdy_bit == 0)	   // Если новые данные снова не готовы то выдаем информацию об ошибке, больше времени нет повторять и считаем те что есть данные
	// 	// 	DEBUG_PRINTF("DRDY Error = %u!\n",drdy_bit);
	// }
	// st2 = read_single_ak09916_reg(MAG_ST2); // Считываем ST2
	// write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR); // AK09916, чтение
	// write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG, MAG_ST2);				  // ST2  Читаем статус считывания данных
	// write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x80 | 1);			  // Включить, 1 байт
	// HAL_Delay(1);

	// write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR); // AK09916, чтение
	// write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG, MAG_ST1);				  // ST1  Читаем статус готовности данных и считвания вовремя
	// write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x80 | 1);			  // Включить, 1 байт
	// write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x00); // Выключаем опрос

	data->x = (int16_t)(mag_data[2] << 8 | mag_data[1]); // HXL, HXH
	data->y = (int16_t)(mag_data[4] << 8 | mag_data[3]); // HYL, HYH
	data->z = (int16_t)(mag_data[6] << 8 | mag_data[5]); // HZL, HZH

	return true;
}

void icm20948_gyro_read_dps(axises *data)
{
	icm20948_gyro_read(data);

	data->x = (data->x - gBias.b_x) / gyro_scale_factor; // Считаем учитвая bias и установку
	data->y = (data->y - gBias.b_y) / gyro_scale_factor;
	data->z = (data->z - gBias.b_z) / gyro_scale_factor;

	// DEBUG_PRINTF("Gyro gyro_scale_factor = %+8.3f | ", gyro_scale_factor);

	static axises smoothed_data = {0, 0, 0}; // Начальные значения
	float const ALPHA = 0.5;
	smoothed_data.x = ALPHA * data->x + (1 - ALPHA) * smoothed_data.x; // Экспоненциальное сглаживание везде по всем осям используем один коефициент
	smoothed_data.y = ALPHA * data->y + (1 - ALPHA) * smoothed_data.y;
	smoothed_data.z = ALPHA * data->z + (1 - ALPHA) * smoothed_data.z;

	// DEBUG_PRINTF("Gyro raw = %+8.3f %+8.3f %+8.3f smoothed= %+8.3f %+8.3f %+8.3f | ", data->x, data->y, data->z, smoothed_data.x, smoothed_data.y, smoothed_data.z);
	// DEBUG_PRINTF("Gyro raw = %.3f smoothed= %.3f \n",data->x,smoothed_data.x);

	*data = smoothed_data;
}
// Считывание акселерометра и преобразование в g делением на акселерационный коэффициент accel_scale_factor
void icm20948_accel_read_g(axises *data)
{
	icm20948_accel_read(data); // Считывание необработанных данных акселерометра
	static float g = 9.80665;  // Ускорение свободного падения в м/с²

	data->x = ((data->x - aBias.b_x) * aScale.s_x) / accel_scale_factor * g; // Преобразование в g делением на акселерационный коэффициент Вычитаем bias и умножаем на масштабный коефициент
	data->y = ((data->y - aBias.b_y) * aScale.s_y) / accel_scale_factor * g; // Преобразование в g делением на акселерационный коэффициент Вычитаем bias и умножаем на масштабный коефициент
	data->z = ((data->z - aBias.b_z) * aScale.s_z) / accel_scale_factor * g; // Преобразование в g делением на акселерационный коэффициент Вычитаем bias и умножаем на масштабный коефициент

	static axises smoothed_data = {0, 0, 1}; // Начальные значения
	float const ALPHA = 0.5;

	smoothed_data.x = ALPHA * data->x + (1 - ALPHA) * smoothed_data.x; // Экспоненциальное сглаживание везде по всем осям используем один коефициент
	smoothed_data.y = ALPHA * data->y + (1 - ALPHA) * smoothed_data.y;
	smoothed_data.z = ALPHA * data->z + (1 - ALPHA) * smoothed_data.z;
	// DEBUG_PRINTF("Norm (g): %.3f",norm);

	// DEBUG_PRINTF("Accel raw = %+8.4f %+8.4f %+8.4f smoothed= %+8.4f %+8.4f %+8.4f | ", data->x, data->y, data->z, smoothed_data.x, smoothed_data.y, smoothed_data.z);

	*data = smoothed_data;
}

// Простая функция сортировки и получения медианы (W=3)
float get_median2(float *buffer)
{
	float a = buffer[0];
	float b = buffer[1];
	float c = buffer[2];

	if (a > b)
	{
		float t = a;
		a = b;
		b = t;
	}
	if (b > c)
	{
		float t = b;
		b = c;
		c = t;
	}
	if (a > b)
	{
		float t = a;
		a = b;
		b = t;
	}

	return b; // Медиана
}

#define ALPHA_X 0.20
#define ALPHA_Y 0.20
#define ALPHA_Z 0.05

// Буфер для медианного фильтра (по 3 точки на ось)
#define WINDOW_SIZE 3 // Размер окна медианы
static float x_buffer[WINDOW_SIZE] = {0.0f, 0.0f, 0.0f};
static float y_buffer[WINDOW_SIZE] = {0.0f, 0.0f, 0.0f};
static float z_buffer[WINDOW_SIZE] = {50.0f, 50.0f, 50.0f};
static uint8_t buffer_index = 0;

#define FILTER_WINDOW_SIZE 16		   // Размер окна скользящего среднего
float fir_bufferX[FILTER_WINDOW_SIZE]; // Буфер FIR
float fir_bufferY[FILTER_WINDOW_SIZE]; // Буфер FIR
float iir_filteredX = 0.0f;
float iir_filteredY = 0.0f;
uint8_t fir_indexX = 0; // Индекс в буфере
uint8_t fir_indexY = 0; // Индекс в буфере
float fir_sumX = 0.0f;	// Сумма значений
float fir_sumY = 0.0f;	// Сумма значений

bool ak09916_mag_read_uT(axises *data)
{
	axises temp;
	static axises smoothed_data = {0, 0, 35};
	ak09916_mag_read(&temp);

	if (isnan(data->x) || isinf(data->x))
		DEBUG_PRINTF("X isnan isinf \n");
	if (isnan(data->y) || isinf(data->y))
		DEBUG_PRINTF("Y isnan isinf\n");
	if (isnan(data->z) || isinf(data->z))
		DEBUG_PRINTF("Z isnan isinf\n");

	data->x = (((float)(temp.x * 0.15) - mBias.b_x) * mScale.s_x);
	data->y = (((float)(temp.y * 0.15) - mBias.b_y) * mScale.s_y);
	data->z = (((float)(temp.z * 0.15) - mBias.b_z) * mScale.s_z);

	// Добавляем новые данные в буфер
	x_buffer[buffer_index] = data->x;
	y_buffer[buffer_index] = data->y;
	z_buffer[buffer_index] = data->z;
	// Обновляем индекс буфера Строка buffer_index = (buffer_index + 1) % WINDOW_SIZE; обеспечивает циклическое переключение индекса в пределах [0, WINDOW_SIZE-1], что необходимо для работы кольцевого буфера в медианном фильтре.
	buffer_index = (buffer_index + 1) % 3;

	// Вычисляем медиану Это для убирания выбросов
	float x_median, y_median, z_median;
	x_median = get_median2(x_buffer);
	y_median = get_median2(y_buffer);
	z_median = get_median2(z_buffer);

	// data->x = x_median;
	// data->y = y_median;
	// data->z = z_median;

	// DEBUG_PRINTF("raw= %.3f x_median= %.3f | ", data->x, x_median);

	// Экспоненциальное сглаживание
	smoothed_data.x = ALPHA_X * x_median + (1 - ALPHA_X) * smoothed_data.x;
	smoothed_data.y = ALPHA_Y * y_median + (1 - ALPHA_Y) * smoothed_data.y;
	smoothed_data.z = ALPHA_Z * z_median + (1 - ALPHA_Z) * smoothed_data.z;
	// DEBUG_PRINTF("smoothed_data =%.3f | ", smoothed_data.x);

	// === Шаг 2: FIR-фильтр X (скользящее среднее) ===
	fir_sumX -= fir_bufferX[fir_indexX]; // Убираем старое
	fir_sumX += smoothed_data.x;		 // Добавляем новое
	fir_bufferX[fir_indexX] = smoothed_data.x;
	; // Обновляем буфер
	fir_indexX = (fir_indexX + 1) % FILTER_WINDOW_SIZE;
	smoothed_data.x = fir_sumX / FILTER_WINDOW_SIZE;
	// === Шаг 2: FIR-фильтр Y (скользящее среднее) ===
	fir_sumY -= fir_bufferY[fir_indexY]; // Убираем старое
	fir_sumY += smoothed_data.y;		 // Добавляем новое
	fir_bufferY[fir_indexY] = smoothed_data.y;
	; // Обновляем буфер
	fir_indexY = (fir_indexY + 1) % FILTER_WINDOW_SIZE;
	smoothed_data.y = fir_sumY / FILTER_WINDOW_SIZE;

	// DEBUG_PRINTF("");

	// Возвращаем сглаженные и отфильтрованные данные
	*data = smoothed_data;

	return true;
}

/* Sub Functions */
bool icm20948_who_am_i()
{
	printf("+++ icm20948_who_am_i \n");
	uint8_t icm20948_id = read_single_icm20948_reg(ub_0, B0_WHO_AM_I);
	if (icm20948_id == ICM20948_ID)
	{
		printf("    ICM-20948 connected successfully WHO_AM_I = 0x%02X\n", ICM20948_ID);
		return true;
	}
	else
	{
		printf("    ICM-20948 connection failed: 0x%02X real= 0x%02X\n ", ICM20948_ID, icm20948_id);
		return false;
	}
}

bool ak09916_who_am_i()
{
	printf("+++ ak09916_who_am_i \n");
	uint8_t ak09916_id = read_single_ak09916_reg(MAG_WIA2);

	if (ak09916_id == AK09916_ID)
	{
		printf("    ak09916_who_am_i connected successfully 0x%02X \n", AK09916_ID);
		return true;
	}
	else
	{
		printf("    ak09916_who_am_i connection failed: 0x%02X real= 0x%02X \n ", AK09916_ID, ak09916_id);
		return false;
	}
}

void icm20948_device_reset()
{
	// write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, 0x80 | 0x41);
	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, 0x80);
	printf("+++ icm20948_device_reset \n");
	HAL_Delay(100);
}

// Включаем bypass режим для первоначальной настройки магнитометра
void icm20948_bypass_en()
{
	printf("+++ icm20948_bypass_en \n");
	write_single_icm20948_reg(ub_0, B0_INT_PIN_CFG, 0x02);
	HAL_Delay(100);
}
// Выключаем bypass режим для первоначальной настройки магнитометра
void icm20948_bypass_disable()
{
	printf("+++ icm20948_bypass_disable \n");
	write_single_icm20948_reg(ub_0, B0_INT_PIN_CFG, 0x00);
	HAL_Delay(100);
}

void ak09916_soft_reset()
{
	printf("+++ ak09916_soft_reset \n");
	write_single_ak09916_reg(MAG_CNTL3, 0x01);
	HAL_Delay(100);
}
//
void icm20948_wakeup()
{
	printf("+++ icm20948_wakeup \n");
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
	new_val &= 0xBF;
	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, new_val);
	HAL_Delay(100);
}

HAL_StatusTypeDef ICM20948_DisableLPMMode()
{
	HAL_StatusTypeDef status;
	uint8_t data;

	select_user_bank(0); // Выбор банка 0 для работы с регистрами PWR_MGMT_1

	// Отключение LP_EN в PWR_MGMT_1 (запись 0x01: LP_EN=0, CLKSEL=001)
	data = 0x01; // LP_EN=0, CLKSEL=001, остальные биты по умолчанию
	status = HAL_I2C_Mem_Write(ICM20948_I2C, ICM20948_I2C_ADDRESS, B0_PWR_MGMT_1, 1, &data, 1, 100);
	if (status != HAL_OK)
	{
		return status; // Возврат ошибки, если запись не удалась
	}

	// Проверка, что PWR_MGMT_1 записался корректно
	data = 0xFF; // Сбрасываем переменную для чтения
	status = HAL_I2C_Mem_Read(ICM20948_I2C, ICM20948_I2C_ADDRESS, B0_PWR_MGMT_1, 1, &data, 1, 100);

	printf("B0_PWR_MGMT_1 registr: 0x%02X ", data); // Вывод регистра
	print_binary(data);								// Вывод

	if (status != HAL_OK || data != 0x01)
	{
		printf("    B0_PWR_MGMT_1 registr ERROR !!!! 0x%02X ", data); // Вывод выбранного банка
		print_binary(data);											  // Вывод выбранного банка в бинарном виде
		return HAL_ERROR;											  // Ошибка, если значение не 0x01
	}

	return HAL_OK; // Успех
}

void icm20948_sleep()
{
	printf("+++ icm20948_sleep \n");
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
	printf("+++ icm20948_i2c_master_reset \n");
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
	new_val |= 0x02;

	write_single_icm20948_reg(ub_0, B0_USER_CTRL, new_val);
}

void icm20948_i2c_master_enable()
{
	printf("+++ icm20948_i2c_master_enable \n");
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
	printf("    1 icm20948_i2c_master_enable B0_USER_CTRL: 0x%02X\n", new_val); //
	new_val |= 0x20;

	write_single_icm20948_reg(ub_0, B0_USER_CTRL, new_val);
	HAL_Delay(100);
	new_val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
	printf("    2 icm20948_i2c_master_enable B0_USER_CTRL: 0x%02X\n", new_val); //
}
void icm20948_i2c_master_disable()
{
	printf("+++ icm20948_i2c_master_disable \n");
	write_single_icm20948_reg(ub_0, B0_USER_CTRL, 0x00);
	HAL_Delay(100);
}

void icm20948_i2c_master_clk_frq(uint8_t config)
{
	uint8_t new_val = read_single_icm20948_reg(ub_3, B3_I2C_MST_CTRL);
	printf("    1 icm20948_i2c_master_clk_frq I2C_MST_CTRL: 0x%02X\n", new_val); // Вывод I2C_MST_CTRL
	new_val |= config;
	printf("    2 icm20948_i2c_master_clk_frq I2C_MST_CTRL: 0x%02X\n", new_val); // Вывод I2C_MST_CTRL
	// new_val = config;

	write_single_icm20948_reg(ub_3, B3_I2C_MST_CTRL, new_val);

	new_val = read_single_icm20948_reg(ub_3, B3_I2C_MST_CTRL);
	printf("    3 icm20948_i2c_master_clk_frq I2C_MST_CTRL: 0x%02X\n", new_val); // Вывод I2C_MST_CTRL
}

void icm20948_clock_source(uint8_t source)
{
	printf("+++ icm20948_clock_source \n");
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
	new_val |= source;

	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, new_val);

	HAL_Delay(100);
	new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
	printf("    ITOG  B0_PWR_MGMT_1: 0x%02X ", new_val); // Вывод B2_GYRO_CONFIG_1
	print_binary(new_val);

	printf("    End icm20948_clock_source ****************************************************** \n");
}

void icm20948_odr_align_enable()
{
	printf("+++ icm20948_odr_align_enable \n");
	write_single_icm20948_reg(ub_2, B2_ODR_ALIGN_EN, 0x01);
}

void icm20948_gyro_low_pass_filter(uint8_t config)
{
	printf("+++ icm20948_gyro_low_pass_filter \n");

	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1);
	printf("    In B2_GYRO_CONFIG_1: 0x%02X ", new_val); // Вывод B2_GYRO_CONFIG_1
	print_binary(new_val);
	printf("    Config = %d ", config);
	print_binary(config);
	new_val |= config << 3;
	printf("    Out B2_GYRO_CONFIG_1: 0x%02X ", new_val); // Вывод B2_GYRO_CONFIG_1
	print_binary(new_val);

	write_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1, new_val);

	HAL_Delay(100);
	new_val = read_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1);
	printf("    ITOG  B2_GYRO_CONFIG_1: 0x%02X ", new_val); // Вывод B2_GYRO_CONFIG_1
	print_binary(new_val);

	printf("    End icm20948_gyro_low_pass_filter ****************************************************** \n");
	HAL_Delay(100);
}

void icm20948_accel_low_pass_filter(uint8_t config)
{
	printf("+++ icm20948_accel_low_pass_filter \n");

	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG);

	printf("    In B2_ACCEL_CONFIG: 0x%02X ", new_val); //
	print_binary(new_val);

	printf("    Config = %d ", config);
	print_binary(config);

	new_val |= config << 3;

	printf("    Out B2_ACCEL_CONFIG: 0x%02X ", new_val); // Вывод B2_ACCEL_CONFIG
	print_binary(new_val);

	write_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG, new_val);

	HAL_Delay(100);
	new_val = read_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG);
	printf("    ITOG  B2_ACCEL_CONFIG: 0x%02X ", new_val); // Вывод B2_GYRO_CONFIG_1
	print_binary(new_val);

	printf("    End icm20948_accel_low_pass_filter ****************************************************** \n");
}

void icm20948_gyro_sample_rate_divider(uint8_t divider)
{
	printf("+++ icm20948_gyro_sample_rate_divider \n");
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_GYRO_SMPLRT_DIV);
	printf("    In B2_GYRO_SMPLRT_DIV: 0x%02X ", new_val); // Вывод B2_GYRO_CONFIG_1
	print_binary(new_val);

	printf("    Divider = %d ", divider);
	print_binary(divider);

	write_single_icm20948_reg(ub_2, B2_GYRO_SMPLRT_DIV, divider);
	HAL_Delay(100);

	new_val = read_single_icm20948_reg(ub_2, B2_GYRO_SMPLRT_DIV);
	printf("    ITOG B2_GYRO_SMPLRT_DIV: 0x%02X ", new_val); // Вывод B2_GYRO_CONFIG_1
	print_binary(new_val);

	printf("    End icm20948_gyro_sample_rate_divider ****************************************************** \n");
}

void icm20948_accel_sample_rate_divider(uint16_t divider)
{
	printf("+++ icm20948_accel_sample_rate_divider \n");
	printf("    Divider = %d ", divider);
	print_binary(divider);

	uint8_t divider_1 = (uint8_t)(divider >> 8);
	uint8_t divider_2 = (uint8_t)(0x0F & divider);

	printf("    Divider1 = %d ", divider_1);
	print_binary(divider_1);
	printf("    Divider2 = %d ", divider_2);
	print_binary(divider_2);

	write_single_icm20948_reg(ub_2, B2_ACCEL_SMPLRT_DIV_1, divider_1);
	write_single_icm20948_reg(ub_2, B2_ACCEL_SMPLRT_DIV_2, divider_2);

	uint8_t new_val1 = read_single_icm20948_reg(ub_2, B2_ACCEL_SMPLRT_DIV_1);
	uint8_t new_val2 = read_single_icm20948_reg(ub_2, B2_ACCEL_SMPLRT_DIV_2);

	printf("    ITOG B2_ACCEL_SMPLRT_DIV_1: 0x%02X ", new_val1); // Вывод
	print_binary(new_val1);
	printf("    ITOG B2_ACCEL_SMPLRT_DIV_2: 0x%02X ", new_val2); // Вывод
	print_binary(new_val2);

	printf("    End icm20948_accel_sample_rate_divider ****************************************************** \n");
}

void ak09916_operation_mode_setting(operation_mode mode)
{
	printf("+++ ak09916_operation_mode_setting \n");
	write_single_ak09916_reg(MAG_CNTL2, mode);
	HAL_Delay(100);
}

// Функция калибровки гироскопа
void icm20948_gyro_calibration()
{
	printf("+++ icm20948_gyro_calibration \n");
	for (int j = 0; j < 5; j++)
	{
		printf("%d ", 5 - j); // Отсчет времени
		fflush(stdout);		  // Принудительный сброс буфера
		HAL_Delay(1000);	  // Задержка 1 секунда
	}
	printf("Start... | ");
	fflush(stdout); // Принудительный сброс буфера

	axises temp;
	int32_t gyro_bias[3] = {0};
	int32_t gyro_max[3] = {0};
	int32_t gyro_min[3] = {0};
	int count = 1000;

	for (int i = 0; i < count; i++)
	{
		icm20948_gyro_read(&temp);
		// DEBUG_PRINTF("    tempX = %.3f  tempY = %.3f  tempZ = %.3f \n", temp.x, temp.y, temp.z);
		gyro_bias[0] += temp.x;
		gyro_bias[1] += temp.y;
		gyro_bias[2] += temp.z;

		if (i == 0)
		{
			gyro_max[0] = temp.x;
			gyro_min[0] = temp.x;
			gyro_max[1] = temp.y;
			gyro_min[1] = temp.y;
			gyro_max[2] = temp.z;
			gyro_min[2] = temp.z;
		}
		else
		{
			if (temp.x > gyro_max[0])
				gyro_max[0] = temp.x;
			if (temp.x < gyro_min[0])
				gyro_min[0] = temp.x;

			if (temp.y > gyro_max[1])
				gyro_max[1] = temp.y;
			if (temp.y < gyro_min[1])
				gyro_min[1] = temp.y;

			if (temp.z > gyro_max[2])
				gyro_max[2] = temp.z;
			if (temp.z < gyro_min[2])
				gyro_min[2] = temp.z;
		}
		HAL_Delay(10);
	}

	int32_t gyro_bias_razbros[3] = {0};
	for (int i = 0; i < 3; i++)
	{
		gyro_bias_razbros[i] = gyro_max[i] - gyro_min[i];
		printf("    gyro_max[%d] = %li  gyro_min[%d] = %li  razbros = %li diapazon = %+8.3f \n", i, gyro_max[i], i, gyro_min[i], gyro_bias_razbros[i], gyro_bias_razbros[i] / gyro_scale_factor);
	}

	printf("    SUM gyroBias0 = %li  gyroBias1 = %li  gyroBias2 = %li \n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);

	gyro_bias[0] = gyro_bias[0] / count;
	gyro_bias[1] = gyro_bias[1] / count;
	gyro_bias[2] = gyro_bias[2] / count;

	printf("    gyroBias0 = %li  gyroBias1 = %li gyroBias2 = %li \n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);

	gBias.b_x = gyro_bias[0];
	gBias.b_y = gyro_bias[1];
	gBias.b_z = gyro_bias[2];

	// Construct the gyro biases for push to the hardware gyro bias registers,
	// which are reset to zero upon device startup.
	// Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format.
	// Biases are additive, so change sign on calculated average gyro biases
	// gyro_offset[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF;
	// gyro_offset[1] = (-gyro_bias[0] / 4) & 0xFF;
	// gyro_offset[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
	// gyro_offset[3] = (-gyro_bias[1] / 4) & 0xFF;
	// gyro_offset[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
	// gyro_offset[5] = (-gyro_bias[2] / 4) & 0xFF;

	// write_multiple_icm20948_reg(ub_2, B2_XG_OFFS_USRH, gyro_offset, 6);
	HAL_Delay(500);
}

// Функция для получения усредненных данных акселерометра
void get_averaged_data(axises *data, uint16_t samples)
{
	axises temp = {0.0f, 0.0f, 0.0f};
	for (uint16_t i = 0; i < samples; i++)
	{
		axises raw;
		icm20948_accel_read(&raw); // Здесь вызывается ваша функция для получения сырых данных
		temp.x += raw.x;
		temp.y += raw.y;
		temp.z += raw.z;
		HAL_Delay(10); // Задержка между выборками (настройте по датчику)
	}
	data->x = temp.x / samples;
	data->y = temp.y / samples;
	data->z = temp.z / samples;
}

// Функция калибровки (шестипозиционная)
void calibrate_accelerometer(void)
{
	axises data[6]; // Данные для шести положений
	const char *positions[6] = {
		"Place sensor with +X axis UP (strictly vertical). Starting in 10 seconds...\r\n",
		"Place sensor with -X axis UP (strictly vertical). Starting in 10 seconds...\r\n",
		"Place sensor with +Y axis UP (strictly vertical). Starting in 10 seconds...\r\n",
		"Place sensor with -Y axis UP (strictly vertical). Starting in 10 seconds...\r\n",
		"Place sensor with +Z axis UP (strictly vertical). Starting in 10 seconds...\r\n",
		"Place sensor with -Z axis UP (strictly vertical). Starting in 10 seconds...\r\n"};

	// Сбор данных для каждого положения
	for (int i = 0; i < 6; i++)
	{
		printf("Position %d: %s", i + 1, positions[i]); // Вывод инструкции пользователю
		for (int j = 0; j < 10; j++)
		{
			printf("%d ", 10 - j); // Отсчет времени
			fflush(stdout);		   // Принудительный сброс буфера
			HAL_Delay(1000);	   // Задержка 1 секунда
		}
		printf("Start... | ");
		fflush(stdout);																			   // Принудительный сброс буфера
		get_averaged_data(&data[i], 1000);														   // Сбор усредненных данных (1000 выборок)
		printf("Position %d: X=%.4f, Y=%.4f, Z=%.4f\r\n", i + 1, data[i].x, data[i].y, data[i].z); // Вывод собранных данных для проверки
	}

	// Расчет смещений и масштабных коэффициентов
	aBias.b_x = (data[0].x + data[1].x) / 2.0f; // (+X, -X)
	aBias.b_y = (data[2].y + data[3].y) / 2.0f; // (+Y, -Y)
	aBias.b_z = (data[4].z + data[5].z) / 2.0f; // (+Z, -Z)

	aScale.s_x = (2.0 * accel_scale_factor) / (data[0].x - data[1].x); // Масштабный коэффициент для X
	aScale.s_y = (2.0 * accel_scale_factor) / (data[2].y - data[3].y); // Масштабный коэффициент для Y
	aScale.s_z = (2.0 * accel_scale_factor) / (data[4].z - data[5].z); // Масштабный коэффициент для Z

	// Вывод параметров калибровки
	printf("Calibration done: Bias: X=%.4f, Y=%.4f, Z=%.4f | Scale: X=%.4f, Y=%.4f, Z=%.4f\r\n",
		   aBias.b_x, aBias.b_y, aBias.b_z,
		   aScale.s_x, aScale.s_y, aScale.s_z);
}

// Старая Функция калибровки акселерометра с записью bias в датчик
void icm20948_accel_calibration()
{
	printf("+++ icm20948_accel_calibration \n");
	axises temp;
	uint8_t *temp2;
	uint8_t *temp3;
	uint8_t *temp4;

	int count = 333;

	int32_t accel_bias[3] = {0};

	int32_t accel_max[3] = {0};
	int32_t accel_min[3] = {0};

	int32_t accel_bias_reg[3] = {0};
	uint8_t accel_offset[6] = {0};

	for (int i = 0; i < count; i++)
	{
		icm20948_accel_read(&temp);
		// printf("    tempX = %.3f  tempY = %.3f  tempZ = %.3f \n", temp.x, temp.y, temp.z);
		accel_bias[0] += temp.x;
		accel_bias[1] += temp.y;
		accel_bias[2] += (temp.z - accel_scale_factor); // Отнимаем чтобы получить bias без гравитации

		if (i == 0)
		{
			accel_max[0] = temp.x;
			accel_min[0] = temp.x;
			accel_max[1] = temp.y;
			accel_min[1] = temp.y;
			accel_max[2] = temp.z;
			accel_min[2] = temp.z;
		}
		else
		{
			if (temp.x > accel_max[0])
				accel_max[0] = temp.x;
			if (temp.x < accel_min[0])
				accel_min[0] = temp.x;

			if (temp.y > accel_max[1])
				accel_max[1] = temp.y;
			if (temp.y < accel_min[1])
				accel_min[1] = temp.y;

			if (temp.z > accel_max[2])
				accel_max[2] = temp.z;
			if (temp.z < accel_min[2])
				accel_min[2] = temp.z;
		}

		HAL_Delay(10);
	}

	int32_t accel_bias_razbros[3] = {0};
	for (int i = 0; i < 3; i++)
	{
		accel_bias_razbros[i] = accel_max[i] - accel_min[i];
		printf("    accel_max[%d] = %li  accel_min[%d] = %li  razbros = %li diapazon = %+8.3f \n", i, accel_max[i], i, accel_min[i], accel_bias_razbros[i], accel_bias_razbros[i] / accel_scale_factor);
	}

	printf("    SUM accel_bias0 = %li  accel_bias1 = %li  accel_bias2 = %li \n", accel_bias[0], accel_bias[1], accel_bias[2]);
	accel_bias[0] /= count;
	accel_bias[1] /= count;
	accel_bias[2] /= count;
	printf("    accel_bias0 = %li  accel_bias1 = %li accel_bias2 = %li \n", accel_bias[0], accel_bias[1], accel_bias[2]);

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
	HAL_Delay(500);
}

void icm20948_gyro_full_scale_select(gyro_full_scale full_scale)
{
	printf("+++ icm20948_gyro_full_scale_select \n");
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1);
	printf("    IN  B2_GYRO_CONFIG_1: 0x%02X ", new_val); // Вывод B2_GYRO_CONFIG_1
	print_binary(new_val);

	printf("    Full scale = %d ", full_scale);
	print_binary(new_val);

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
	printf("    OUT  B2_GYRO_CONFIG_1: 0x%02X ", new_val); // Вывод B2_GYRO_CONFIG_1
	print_binary(new_val);

	write_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1, new_val);
	HAL_Delay(100);

	new_val = read_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1); // Считываем значение регистра B2_GYRO_CONFIG_1 после записи
	printf("    ITOG  B2_GYRO_CONFIG_1: 0x%02X ", new_val);		// Вывод B2_GYRO_CONFIG_1
	print_binary(new_val);
	printf("    End icm20948_gyro_full_scale_select ****************************************************** \n");
}

void icm20948_accel_full_scale_select(accel_full_scale full_scale)
{

	printf("+++ icm20948_accel_full_scale_select \n");
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG);
	printf("    IN  B2_ACCEL_CONFIG: 0x%02X ", new_val); // Вывод B2_GYRO_CONFIG_1
	print_binary(new_val);

	printf("    Full scale = %d ", full_scale);
	print_binary(new_val);

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
	printf("    OUT  B2_ACCEL_CONFIG: 0x%02X ", new_val); // Вывод B2_GYRO_CONFIG_1
	print_binary(new_val);

	write_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG, new_val);
	HAL_Delay(100);

	new_val = read_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG); // Считываем значение регистра B2_GYRO_CONFIG_1 после записи
	printf("    ITOG  B2_ACCEL_CONFIG: 0x%02X ", new_val);	   // Вывод B2_GYRO_CONFIG_1
	print_binary(new_val);
	printf("    End icm20948_accel_full_scale_select ****************************************************** \n");
}

/* Static Functions */
// Вариант для I2C
static void select_user_bank(userbank ub)
{
	HAL_StatusTypeDef status;
	uint8_t data;

	// Выбор банка 0
	data = ub; // Банк 0
	status = HAL_I2C_Mem_Write(ICM20948_I2C, ICM20948_I2C_ADDRESS, REG_BANK_SEL, 1, &data, 1, 100);
	if (status != HAL_OK)
	{
		// return status; // Возврат ошибки, если выбор банка не удался
	}

	// Проверка, что банк 0 выбран
	// data = 0xFF; // Сбрасываем переменную для чтения
	// status = HAL_I2C_Mem_Read(ICM20948_I2C, ICM20948_I2C_ADDRESS, REG_BANK_SEL, 1, &data, 1, 100);
	// printf("    bank: 0x%02X \n", data); // Вывод выбранного банка

	if (status != HAL_OK || data != ub)
	{
		printf("    bank: ERROR !!!! 0x%02X \n", data); // Вывод выбранного банка
														// return HAL_ERROR;									  // Ошибка, если банк не 0
	}

	// // Создание буфера для передачи: первый байт — адрес регистра REG_BANK_SEL, второй — значение банка
	// // REG_BANK_SEL находится по адресу 0x7F в банке 0
	// uint8_t data[2];
	// data[0] = REG_BANK_SEL; // Адрес регистра REG_BANK_SEL (0x7F)
	// data[1] = ub;			// Значение для записи (номер банка)

	// // Выполнение передачи данных по I2C
	// // HAL_I2C_Master_Transmit отправляет данные на устройство с адресом ICM20948_I2C_ADDRESS
	// // Параметры: I2C handle, адрес устройства, буфер данных, длина (2 байта), тайм-аут (100 мс)
	// HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(ICM20948_I2C, ICM20948_I2C_ADDRESS, data, 2, 100);

	// // Проверка статуса передачи для отладки
	// // Если передача не удалась (например, устройство не отвечает), выводим ошибку
	// if (status != HAL_OK)
	// {
	// 	// Для отладки можно вывести сообщение об ошибке через UART
	// 	printf("I2C select_user_bank error: %d\n", status);
	// }
}

extern volatile uint8_t i2cTransferComplete; // Флаг завершения операции
extern volatile uint8_t i2cReceiveComplete;	 // Флаг завершения операции

//***************************** ЧЕРЕЗ ПРЕРЫВАНИЯ **************************************
// Функция для чтения данных из ICM20948 используя прерывание
void ICM20948_Transmit_IT(uint8_t _reg)
{
	i2cTransferComplete = 0;
    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) // Проверяем, готов ли I2C
    {
        printf("ICM20948_Transmit_IT I2C not ready, resetting...\n");
        HAL_I2C_DeInit(&hi2c1); // Деинициализация I2C
        HAL_I2C_Init(&hi2c1);   // Повторная инициализация I2C
    }

	static uint8_t reg = B0_ACCEL_XOUT_H;														  // Статическая переменная для хранения регистра
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit_IT(&hi2c1, ICM20948_I2C_ADDRESS, &reg, 1); // Отправляем адрес регистра
	if (status != HAL_OK)
	{
		printf("ICM20948_Transmit_IT ERROR\n");
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

	// HAL_Delay(1);													   // Ждем завершения передачи.Обязательно!!! Нельзя выходитьт из функции пока поманда не передастся.
	// DEBUG_PRINTF("ICM20948_Transmit_IT \n");
}

// Функция для чтения данных из ICM20948 используя прерывание
void ICM20948_Receive_IT(uint8_t *buffer, uint16_t size)
{
	i2cReceiveComplete = 0;
	if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) // Проверяем, готов ли I2C
		printf("ICM20948_Receive_IT I2C not ready, resetting...\n");

	HAL_StatusTypeDef status = HAL_I2C_Master_Receive_IT(&hi2c1, ICM20948_I2C_ADDRESS, buffer, size); // Запускаем чтение данных из регистра
	if (status != HAL_OK)
	{
		printf("ICM20948_Transmit_IT ERROR\n");
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
// Функция для расчета буфера ICM20948
void calcBufferICM(uint8_t *buffer, axises *dataAccel, axises *dataGyro)
{
	// DEBUG_PRINTF("ICM20948 buffer");
	dataAccel->x = (int16_t)(buffer[0] << 8 | buffer[1]);
	dataAccel->y = (int16_t)(buffer[2] << 8 | buffer[3]);
	dataAccel->z = (int16_t)(buffer[4] << 8 | buffer[5]);

	dataGyro->x = (int16_t)(buffer[6] << 8 | buffer[7]);
	dataGyro->y = (int16_t)(buffer[8] << 8 | buffer[9]);
	dataGyro->z = (int16_t)(buffer[10] << 8 | buffer[11]);

	// for (int i = 0; i < 12; i++)
	// {
	// 	DEBUG_PRINTF(" = 0x%02X", buffer[i]);
	// }
	// DEBUG_PRINTF(" \n");
}
static void write_single_icm20948_reg(userbank ub, uint8_t reg, uint8_t val)
{
	HAL_StatusTypeDef status;
	// Выбор банка регистров (0, 1, 2 или 3) для доступа к нужному регистру 	// ICM-20948 использует банки регистров для организации своих настроек
	select_user_bank(ub);

	// Создание буфера для передачи: первый байт — адрес регистра, второй — значение для записи	// В I2C для записи в регистр сначала отправляется адрес регистра, затем данные
	uint8_t data[2];
	data[0] = reg; // Адрес регистра, в который будет произведена запись
	data[1] = val; // Значение, которое нужно записать в регистр

	uint8_t data2 = val;

	// status = HAL_I2C_Master_Transmit(ICM20948_I2C, ICM20948_I2C_ADDRESS, data, 2, 100);

	status = HAL_I2C_Mem_Write(ICM20948_I2C, ICM20948_I2C_ADDRESS, reg, 1, &data2, 1, 100);

	if (status != HAL_OK) // Проверка статуса передачи для отладки	// Если передача не удалась (например, устройство не отвечает), можно добавить обработку ошибки
	{
		// Для отладки можно вывести ошибку через UART или зажечь светодиод
		printf("I2C read_single_icm20948_reg write error: %d\n", status);
	}
}
static void write_single_icm20948_reg2(uint8_t reg, uint8_t val)
{
	// Создание буфера для передачи: первый байт — адрес регистра, второй — значение для записи 	// В I2C для записи в регистр сначала отправляется адрес регистра, затем данные
	uint8_t data[2];
	data[0] = reg; // Адрес регистра, в который будет произведена запись
	data[1] = val; // Значение, которое нужно записать в регистр

	// Выполнение передачи данных по I2C  HAL_I2C_Master_Transmit отправляет данные на устройство с адресом ICM20948_I2C_ADDRESS  Параметры: I2C handle, адрес устройства, буфер данных, длина (2 байта), тайм-аут (100 мс)
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(ICM20948_I2C, ICM20948_I2C_ADDRESS, data, 2, 100);

	if (status != HAL_OK) // Проверка статуса передачи для отладки 	// Если передача не удалась (например, устройство не отвечает), можно добавить обработку ошибки
		printf("I2C read_single_icm20948_reg write error: %d\n", status);
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
		printf("I2C read_single_icm20948_reg read error: %d\n", status);
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
		printf("I2C read_multiple_icm20948_reg error: %d\n", status);
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
		printf("I2C write_multiple_icm20948_reg error: %d\n", status);
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

// static uint8_t *read_multiple_ak09916_reg(uint8_t reg, uint8_t len)
// {
// 	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR);
// 	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG, reg);
// 	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x80 | len);

// 	HAL_Delay(1);
// 	return read_multiple_icm20948_reg(ub_0, B0_EXT_SLV_SENS_DATA_00, len);
// }