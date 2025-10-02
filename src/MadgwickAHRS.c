//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MadgwickAHRS.h"
#include "main.h"
#include "config.h"
#include <math.h>
#include <string.h> // Для memcpy, чтобы избежать проблем с выравниванием на 64-битных системах

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq 100.0f // sample frequency in Hz Я использую 100 HZ
#define betaDef 0.1f	  // параметр beta — это коэффициент обратной связи (gain), который управляет тем, насколько сильно фильтр будет «доверять» акселерометру (и магнитометру, если он есть), когда он корректирует ориентацию, полученную интегрированием гироскопа.
// 🔁 Чем выше beta → тем сильнее и быстрее фильтр использует акселерометр для коррекции ориентации.
// 🔁 Чем ниже beta → тем слабее и медленнее коррекция от акселерометра. 

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float beta = betaDef;							   // 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update
/*
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	// printf("MadgwickAHRSupdate IN gx= %.3f gy= %.3f gz= %.3f ax= %.3f ay= %.3f az= %.3f mx= %.3f my= %.3f mz= %.3f \n", gx, gy, gz, ax, ay, az, mx, my, mz);
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	// DEBUG_PRINTF(" | q0=%.3f q1=%.3f q2=%.3f q3=%.3f | ", q0, q1, q2, q3);
	// ***************** МОЕ ДОПОЛНЕНИЕ *************** 3 варианта расчетов
	roll_Mad = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
	pitch_Mad = asinf(-2.0f * (q1 * q3 - q0 * q2));
	yaw_Mad = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);

	roll_Mad = RAD2DEG(roll_Mad);	// Перевод в градусы
	pitch_Mad = RAD2DEG(pitch_Mad); // Перевод в градусы
	yaw_Mad = RAD2DEG(yaw_Mad);		//	 Перевод в градусы

	// // Преобразование кватерниона в углы Эйлера
	// roll_M = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
	// pitch_M = asinf(2.0f * (q0 * q2 - q3 * q1));
	// yaw_M = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));

	// roll_M = atan2f(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
	// pitch_M = asinf(-2.0f * (q1 * q3 - q0 * q2));
	// yaw_M = atan2f(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
}

*/
//---------------------------------------------------------------------------------------------------
// IMU algorithm update
volatile float roll_Mad = 0.0f, pitch_Mad = 0.0f, yaw_Mad = 0.0f; // Углы считаем каждый раз из кватерниона.
float gravity_x, gravity_y, gravity_z;							  // Гравитация в кватернионе

//*******************
// Глобальные переменные для фильтра
static double x_f = 1.0;
static double y_f = 0.0;
static int initialized = 0;

// Функция сглаживания yaw
double smoothYaw(double yaw_rad, double alpha)
{
	if (!initialized)
	{
		x_f = cos(yaw_rad);
		y_f = sin(yaw_rad);
		initialized = 1;
		return yaw_rad;
	}

	double x = cos(yaw_rad);
	double y = sin(yaw_rad);

	// EMA по векторам
	x_f = (1.0 - alpha) * x_f + alpha * x;
	y_f = (1.0 - alpha) * y_f + alpha * y;

	// Нормализация
	double norm = sqrt(x_f * x_f + y_f * y_f);
	if (norm > 1e-9)
	{
		x_f /= norm;
		y_f /= norm;
	}

	// Восстановление угла
	return atan2(y_f, x_f);
}

double to360(double angle)
{
	angle = fmod(angle, 360.0);
	if (angle < 0.0)
		angle += 360.0;
	return angle;
}
double to360_cw(double angle)
{
	// для по часовой стрелке используем обратный знак
	angle = -angle;
	angle = fmod(angle, 360.0);
	if (angle < 0.0)
		angle += 360.0;
	return angle;
}

///****************

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Нормализация для Маджвика аксельрометра. Да, данные акселерометра для фильтра Маджвика необходимо нормализовать. Нормализация означает, что вектор ускорения должен быть приведён к единичной длине (магнитуда вектора равна 1).
	float norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm > 0)
	{
		ax = ax / norm;
		ay = ay / norm;
		az = az / norm;
	}

	// Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	// DEBUG_PRINTF(" | q0=%.3f q1=%.3f q2=%.3f q3=%.3f | ", q0, q1, q2, q3);
	//***************** МОЕ ДОПОЛНЕНИЕ *************** 3 варианта расчетов
	roll_Mad = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
	pitch_Mad = asinf(-2.0f * (q1 * q3 - q0 * q2));
	yaw_Mad = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);

	// Фильтруем значения после Маджика
	float const ALPHA = 0.25;				 //
	static axises smoothed_data = {0, 0, 0}; // Начальные значения

	// smoothed_data.x = roll_Mad; //
	// smoothed_data.y = pitch_Mad; //
	smoothed_data.x = ALPHA * roll_Mad + (1 - ALPHA) * smoothed_data.x; // Экспоненциальное сглаживание везде по всем осям используем один коефициент
	smoothed_data.y = ALPHA * pitch_Mad + (1 - ALPHA) * smoothed_data.y;
	// smoothed_data.z = ALPHA * yaw_Mad + (1 - ALPHA) * smoothed_data.z;

	yaw_Mad = smoothYaw(yaw_Mad, ALPHA); // тут сглаживание по yaw, но чтобы избежать проблем +-180 то через углы

	// DEBUG_PRINTF("Norm (g): %.3f",norm);

	// DEBUG_PRINTF("Madgw raw = %+8.4f %+8.4f %+8.4f smoothed= %+8.4f %+8.4f %+8.4f | ",roll_Mad, pitch_Mad, yaw_Mad, smoothed_data.x, smoothed_data.y, yaw_Mad);

	Madgw.roll = RAD2DEG(smoothed_data.x);
	Madgw.pitch = RAD2DEG(smoothed_data.y);
	Madgw.yaw = to360_cw(RAD2DEG(yaw_Mad));

	//************** ГРАВИТАЦИЯ ****************

	gravity_x = 2.0f * (q1 * q3 - q0 * q2);
	gravity_y = 2.0f * (q0 * q1 + q2 * q3);
	gravity_z = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
	// DEBUG_PRINTF(" | gravity_x= %+6.3f gravity_y= %+6.3f gravity_z= %+6.3f | ", gravity_x, gravity_y, gravity_z);

	const float g = 9.80665f;					   // Ускорение свободного падения в м/с²
											   // 1. Вычитаем гравитацию (оба значения в g)
	static axises smoothed_linAcc = {0, 0, 0}; // Начальные значениям

	Madgw.linAcc.x = (ax - gravity_x) * g; // Вычитаем гравитацию из акселерометра b Переводим в м/с²
	Madgw.linAcc.y = (ay - gravity_y) * g;
	Madgw.linAcc.z = (az - gravity_z) * g; // Вычитаем гравитацию из акселерометра b Переводим в м/с²

	float const ALPHA2 = 0.25;				 //
	smoothed_linAcc.x = ALPHA2 * Madgw.linAcc.x + (1 - ALPHA2) * smoothed_linAcc.x; // Экспоненциальное сглаживание везде по всем осям используем один коефициент
	smoothed_linAcc.y = ALPHA2 * Madgw.linAcc.y + (1 - ALPHA2) * smoothed_linAcc.y; // Экспоненциальное сглаживание везде по всем осям используем один коефициент
	smoothed_linAcc.z = ALPHA2 * Madgw.linAcc.z + (1 - ALPHA2) * smoothed_linAcc.z; // Экспоненциальное сглаживание везде по всем осям используем один коефициент

	Madgw.linAcc = smoothed_linAcc;

	// DEBUG_PRINTF("lin_x= %+6.3f m/s² lin_y= %+6.3f m/s² lin_z= %+6.3f m/s² \n", linearAcc_x, linearAcc_y, linearAcc_z);
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root // See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long *)&y;
	// long i;
	// memcpy(&i, &y, sizeof(i));     // Безопасный способ
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	// memcpy(&y, &i, sizeof(y));     // безопасный способ преобразовать обратно в float
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y)); // Нет, это не ошибка, а улучшение точности. Метод invSqrt реализует быстрый алгоритм обратного квадратного корня, основанный на приближении с использованием магической константы 0x5f3759df. Первая итерация метода Ньютона (y = y * (1.5f - (halfx * y * y))) дает грубое приближение, а вторая итерация улучшает точность результата.

	return y;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
