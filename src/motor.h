#ifndef MOTOR_H
#define MOTOR_H

const int accel_speed = 20; // Ускорение в микросекундах
#define MAX_PID_SPEED 360   // Угловая скорость в градус на секунду.

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "config.h"

//---------------------------------------------------------------------------------------
#define SPEED 360.0   // Скорость на всех моторах одинаковая максимально возможная, что-бы перемещаться как можно быстрее Градус за секунду. Точность - одня десятая
#define MICROSTEP 16  // Микрошаг на драйверах
#define STEPMOTOR 0.9 // Параметры градусы на шаг

#define JUMP_THRESHOLD 10.0f // Порог, выше которого считаем, что это "скачок", а не вращение (градусы)

int statusTestMotor = 0;                    // Статус теста мотора для отладки
const float sector = STEPMOTOR / MICROSTEP; // Столько градусов приходится на 1 импульс

extern float normalizeAngle(float angle);

bool flagMode9 = false;
bool flagMode91 = false;
bool flagMicric = false;
uint32_t timerMode9 = 0;
extern uint32_t millis(); // Длинна полученных данных в буфере
extern uint64_t micros(void);

void setMotorSpeed(int num_, float _speed);                  // Функция устанавлявающая скорость вращения на ВСЕХ моторах, задается в оборотах за секунду rps
float calcSpeedMotor(int num, float dt_);                    // Расчет скорости для мотора.
float calcAngleSpeedMotor(int num, float angle_, float dt_); // Расчет угловой скорости для мотора в градусах в секунду dps
void setPeriod(int num_);                                    // Установка периода следующего срабатывания таймера
void initMotor();                                            // Функция инциализации моторов
void timerMotor(int i);                                      // Обработчик прерывания таймера TIM7
void Set_Timer7_Period(uint32_t new_period);                 // Функция для изменения периода таймера TIM7
void Set_Timer10_Period(uint32_t new_period);                // Функция для изменения периода таймера TIM7
void Set_Timer11_Period(uint32_t new_period);                // Функция для изменения периода таймера TIM7
void Set_Timer13_Period(uint32_t new_period);                // Функция для изменения периода таймера TIM7
void testMotorRun();                                         // Запуск моторов на тест
void testMotorStop();                                        // Остановка моторов после теста
void disableMotor();                                         // Отключение моторов в простое
int32_t getPulse(float angle_);                              // Пересчет градусов в импульсы
float getAngle(int32_t _pulse);                              // Пересчет импульсов в градусы
void setMotorAngle(int num, float _angle);                   // Установка мотора в нужное положение в градусах локальной системы
void setZeroMotor();                                         // Функция установки в ноль всех моторов

void isrMicMotor0(); // Обработка прерывания по микрику на пине для мотора0
void isrMicMotor1(); // Обработка прерывания по микрику на пине для мотора1
void isrMicMotor2(); // Обработка прерывания по микрику на пине для мотора2
void isrMicMotor3(); // Обработка прерывания по микрику на пине для мотора3

// struct motorStruct // Структура для локального управления и сбора данных по моторам
// {
//     int status;        // Передаются импульсы на мотор или нет в данный момент, вращается или нет
//     int position;      // Текущая позиция в импульсах
//     int destination;   // Цель назначение в позиции в импульсах
//     int dir;           // Направление вращения мотора 1 - по часовой 0 - против часовой
//     int dir_port;      // Пин определяющий направление вращения
//     uint16_t dir_pin;  // Пин определяющий направление вращения
//     int step_port;     // Пин определяющий импульс
//     uint16_t step_pin; // Пин определяющий импульс
//     int micric_port;   // Пин определяющий концевик
//     int micric_pin;    // Пин определяющий концевик
// };
// struct motorStruct motor[4]; // Все локальные данные по моторам

float calcSpeedMotor(int num, float dt_) // Расчет скорости для мотора. в градусах в секунду dps
{
    float P = 10.0; // Коефициент Р ПИД регулятора
    float I = 0.0;  // Коефициент I ПИД регулятора
    float D = 0.0;  // Коефициент D ПИД регулятора
    float PError, IError, DError;

    PError = getAngle(motor[num].destination - motor[num].position); // Считаем ошибку по положению

    IError = motor[num].IError + PError; // Интегральная ошибка. Суммируем ошибки.
    if (IError > MAX_PID_SPEED)
        IError = MAX_PID_SPEED; // Ограничение интегральной ошибки
    if (IError < -MAX_PID_SPEED)
        IError = -MAX_PID_SPEED; // Ограничение интегральной ошибки
    motor[num].IError = IError;  // Запоминаем интегральную ошибку для следующего расчета

    // DEBUG_PRINTF("PError old = %.2f | ", motor[num].PError);
    DError = (PError - motor[num].PError); // Дифференциалная ошибка
    // DEBUG_PRINTF("DError = %.2f | ", DError);
    DError = (PError - motor[num].PError) / dt_; // Дифференциалная ошибка
    // DEBUG_PRINTF("DError2 = %.2f | ", DError);
    motor[num].PError = PError; // Запоминаем ошибку по положению для следующего расчета

    DEBUG_PRINTF("PError = %.2f IError = %.2f DError = %.2f | ", PError, IError, DError);

    float pidSpeed = (PError * P) + (IError * I) + (DError * D); // Считаем скорость вращения мотора ПИД регулятором
    if (pidSpeed > MAX_PID_SPEED)
        pidSpeed = MAX_PID_SPEED; // Ограничение максимальной скорости
    if (pidSpeed < -MAX_PID_SPEED)
        pidSpeed = -MAX_PID_SPEED; // Ограничение максимальной скорости

    DEBUG_PRINTF("pidSpeed = %.4f dps (gradus/sec) \n", pidSpeed);

    float sumSpeed = motor[num].angleSpeed + pidSpeed; // к скорости мотора угловой прибавляем ошибку по положению
    DEBUG_PRINTF("sumSpeed = %.4f dps (gradus/sec) \n", sumSpeed);

    return sumSpeed;
}

// Функция устанавлявающая скорость вращения на ВСЕХ моторах, задается в градусах за секунду dps
void setMotorSpeed(int num_, float speed_)
{
    DEBUG_PRINTF("num %i  speed dps = %.2f | ", num_, speed_);

    if (speed_ == 0) // Теперь все наши действия зависят от скорости если скорость 0 то и мотор не крутится и не нужно ничего делать
    {
        motor[num_].status = 0; // Выключаем действия в прерывании, не делаем шаги больше
        DEBUG_PRINTF("STOP.");
    }
    else // Если скорость не ноль то
    {
        // Устанавливаем направление вращения
        if (speed_ > 0) // Если цель больше 0 то вращение по часовой 1
        {
            HAL_GPIO_WritePin(motor[num_].dir_port, motor[num_].dir_pin, GPIO_PIN_SET); // Установить пин HGH GPIO_PIN_SET — установить HIGH,  GPIO_PIN_RESET — установить LOW.
            motor[num_].dir = 1;
        }
        if (speed_ < 0) // Если цель меньше 0 то вращение против часовой 0
        {
            HAL_GPIO_WritePin(motor[num_].dir_port, motor[num_].dir_pin, GPIO_PIN_RESET); // Установить пин HGH GPIO_PIN_SET — установить HIGH,  GPIO_PIN_RESET — установить LOW.
            motor[num_].dir = 0;
        }

        speed_ = speed_ / 360.0; // Переводим из градусов в секунду в оборотs за секунду
        DEBUG_PRINTF(" rps = %.2f |", speed_);
        // Скорость в оборотах их умножаем на градусы и делим на градус на 1 шаг получаем нужное число полных шагов для такой скорости за секунду
        float step_za_sec = abs(speed_ * 100) / 100.0 * (360.0 / STEPMOTOR) * MICROSTEP; // Умножаем и делим на 100 чтобы учесть знаки после запятой при округлении?
        // printf("step_za_sec= %.1f | ", (float)step_za_sec);

        // Микросекунды в секунде делим на число шагов которые надо успеть сделать за секунду и делим на микрошаги делим на микросекунды за 1 шаг с учетом предделителя таймера
        int timeingStep = (float)1000000.0 / step_za_sec; // Таймер по 1 микросекунде // Число тактов для таймера через которое нужно дать новый имульс мотору
        // printf("timeingStep= %i |", timeingStep);
        motor[num_].speedNeed = timeingStep; // Запоминаем скорость какую надо достичь


        if (motor[num_].status == 0) // Если мотор стоит без движения
        {
            float MIN_SPEED = 1000; // Интервал скорости в микросекундах. Чем больше интревал тем медленне вращение
            motor[num_].speedNow = MIN_SPEED; // Устанавливаем Минимальную скорость чтобы тронуться
            DEBUG_PRINTF(" Start from min speed %i | ", motor[num_].speedNow);
        }
        // else
        // motor[num_].speedNow = timeingStep; // Устанавливаем сразу нужную скорость
        motor[num_].status = 1;                                              // Включаем действия в прерывании, делаем шаги
        HAL_GPIO_WritePin(En_Motor_GPIO_Port, En_Motor_Pin, GPIO_PIN_RESET); // Включаем драйвер. Установить пин HGH GPIO_PIN_SET — установить HIGH,  GPIO_PIN_RESET — установить LOW.
        setPeriod(num_);                                                     //
        DEBUG_PRINTF(" speedNow = %i speedNeed = %i microsecond ", motor[num_].speedNow, motor[num_].speedNeed);
    }
    DEBUG_PRINTF("\n");
}

// Установка периода следующего срабатывания таймера
void setPeriod(int num_)
{
    switch (num_)
    {
    case 0:
        Set_Timer7_Period(motor[num_].speedNow); //  Устаналиваем время на таймере
        break;
    case 1:
        Set_Timer10_Period(motor[num_].speedNow); //  Устаналиваем время на таймере
        break;
    case 2:
        Set_Timer11_Period(motor[num_].speedNow); //  Устаналиваем время на таймере
        break;
    case 3:
        Set_Timer13_Period(motor[num_].speedNow); //  Устаналиваем время на таймере
        break;
    }
    // DEBUG_PRINTF("setPeriod %i  = %i \n", num_, motor[num_].speedNow);
}
// Функция инциализации моторов
void initMotor()
{
    printf("initMotor... \n");
    //**************************************************************   Мотор на колеса
    // microStep = MICROSTEP; // Так распаяно на плате для драйверов 2208 и 2209
    // printf("Init StepMotor. Set microstep = %i \n", microStep);

    // Установка пина разрешающего работу драйвероы 0- Разрешена работа 1- запрещена работа драйвера
    HAL_GPIO_WritePin(En_Motor_GPIO_Port, En_Motor_Pin, GPIO_PIN_SET); // Установить пин HGH GPIO_PIN_SET — установить HIGH,  GPIO_PIN_RESET — установить LOW.

    motor[0].step_port = Step_Motor0_GPIO_Port;
    motor[0].step_pin = Step_Motor0_Pin;
    motor[0].dir_port = Dir_Motor0_GPIO_Port;
    motor[0].dir_pin = Dir_Motor0_Pin;

    motor[0].micric_pin = micMotor0_Pin;
    motor[0].micric_port = micMotor0_GPIO_Port;
    motor[0].htim = &htim7;
    //----
    motor[1].step_port = Step_Motor1_GPIO_Port;
    motor[1].step_pin = Step_Motor1_Pin;
    motor[1].dir_port = Dir_Motor1_GPIO_Port;
    motor[1].dir_pin = Dir_Motor1_Pin;

    motor[1].micric_pin = micMotor1_Pin;
    motor[1].micric_port = micMotor1_GPIO_Port;
    motor[1].htim = &htim10;

    motor[2].step_port = Step_Motor2_GPIO_Port;
    motor[2].step_pin = Step_Motor2_Pin;
    motor[2].dir_port = Dir_Motor2_GPIO_Port;
    motor[2].dir_pin = Dir_Motor2_Pin;

    motor[2].micric_pin = micMotor2_Pin;
    motor[2].micric_port = micMotor2_GPIO_Port;
    motor[2].htim = &htim11;

    motor[3].step_port = Step_Motor3_GPIO_Port;
    motor[3].step_pin = Step_Motor3_Pin;
    motor[3].dir_port = Dir_Motor3_GPIO_Port;
    motor[3].dir_pin = Dir_Motor3_Pin;

    motor[3].micric_pin = micMotor3_Pin;
    motor[3].micric_port = micMotor3_GPIO_Port;
    motor[3].htim = &htim13;

    motor[0].status = 0; // Флаг ставим что мотор не работает, просто запрещаем делать импульсы
    motor[1].status = 0; // Флаг ставим что мотор не работает, просто запрещаем делать импульсы
    motor[2].status = 0; // Флаг ставим что мотор не работает, просто запрещаем делать импульсы
    motor[3].status = 0; // Флаг ставим что мотор не работает, просто запрещаем делать импульсы

    // setMotorSpeed(0, SPEED); // Устанавливаем скорость вращения моторов и в дальнейшем только флагами включаем или отключаем вращение
    // setMotorSpeed(1, SPEED); // Устанавливаем скорость вращения моторов и в дальнейшем только флагами включаем или отключаем вращение
    // setMotorSpeed(2, SPEED); // Устанавливаем скорость вращения моторов и в дальнейшем только флагами включаем или отключаем вращение
    // setMotorSpeed(3, SPEED); // Устанавливаем скорость вращения моторов и в дальнейшем только флагами включаем или отключаем вращение
}

// Функция для изменения периода таймера TIM7
void Set_Timer7_Period(uint32_t new_period)
{
    HAL_TIM_Base_Stop_IT(&htim7);                 // Останавливаем таймер перед изменением
    __HAL_TIM_SET_AUTORELOAD(&htim7, new_period); // Задаем новый период таймера (в пересчете на такты таймера)
    HAL_TIM_Base_Start_IT(&htim7);                // Запускаем таймер с новым периодом
}
// Функция для изменения периода таймера TIM10
void Set_Timer10_Period(uint32_t new_period)
{
    HAL_TIM_Base_Stop_IT(&htim10);                 // Останавливаем таймер перед изменением
    __HAL_TIM_SET_AUTORELOAD(&htim10, new_period); // Задаем новый период таймера (в пересчете на такты таймера)
    HAL_TIM_Base_Start_IT(&htim10);                // Запускаем таймер с новым периодом
}
// Функция для изменения периода таймера TIM11
void Set_Timer11_Period(uint32_t new_period)
{
    HAL_TIM_Base_Stop_IT(&htim11);                 // Останавливаем таймер перед изменением
    __HAL_TIM_SET_AUTORELOAD(&htim11, new_period); // Задаем новый период таймера (в пересчете на такты таймера)
    HAL_TIM_Base_Start_IT(&htim11);                // Запускаем таймер с новым периодом
}
// Функция для изменения периода таймера TIM13
void Set_Timer13_Period(uint32_t new_period)
{
    HAL_TIM_Base_Stop_IT(&htim13);                 // Останавливаем таймер перед изменением
    __HAL_TIM_SET_AUTORELOAD(&htim13, new_period); // Задаем новый период таймера (в пересчете на такты таймера)
    HAL_TIM_Base_Start_IT(&htim13);                // Запускаем таймер с новым периодом
}

void timerMotor(int i) // Обработчик прерывания всех 4 таймеров. В зависимости от таймера приходит номер мотора
{
    if (statusTestMotor) // Если тестовый режим то только делаем шаги не обращая внимание ни на что
    {
        // HAL_GPIO_WritePin(motor[i].step_port, motor[i].step_pin, GPIO_PIN_SET); // Установить пин HGH GPIO_PIN_SET — установить HIGH,  GPIO_PIN_RESET — установить LOW.
        // // delayMicroseconds(1);                                                  // На stm32 не нужно и так 5 микросекунд выходит импульс
        // HAL_GPIO_WritePin(motor[i].step_port, motor[i].step_pin, GPIO_PIN_RESET); // Установить пин HGH GPIO_PIN_SET — установить HIGH,  GPIO_PIN_RESET — установить LOW.
    }
    else // Если не тестовый режим то работаем по нормальному алгоритму движения ПИД регулятором
    {
        if (motor[i].status) // Если статус работать
        {
            // if (motor[i].position == motor[i].destination) // Статус statusTestMotor только для отладки чтобы включить моторы на постоянное вращение
            // {
            //     motor[i].status = 0;
            // }
            // else
            // {
            HAL_GPIO_WritePin(motor[i].step_port, motor[i].step_pin, GPIO_PIN_SET); // Установить пин HGH GPIO_PIN_SET — установить HIGH,  GPIO_PIN_RESET — установить LOW.
            (motor[i].dir == 1) ? motor[i].position++ : motor[i].position--;        // Считаем шаги мотора, положение
            // }

            // УСКОРЕНИЕ. Тут управление интервалом. Поэтому чем интервал меньше тем быстрее вращение
            if (motor[i].speedNeed < motor[i].speedNow - accel_speed) // Если скорость получается быстре чем (текущая + ускорение) то используем минимальную
                motor[i].speedNow = motor[i].speedNow - accel_speed;
            else
                motor[i].speedNow = motor[i].speedNeed; // Устанавливаем Новую скорость
            setPeriod(i);                               // Устанавливает период таймера motor[i].speedNow
            // delayMicroseconds(1); // На stm32 не нужно и так 5 микросекунд выходит импульс
            HAL_GPIO_WritePin(motor[i].step_port, motor[i].step_pin, GPIO_PIN_RESET); // Установить пин HGH GPIO_PIN_SET — установить HIGH,  GPIO_PIN_RESET — установить LOW.
        }
    }
}

// Запуск моторов на тест
void testMotorRun()
{
    printf("testMotorRun...\r\n");
    // for (int i = 0; i < 4; i++)
    //     motor[i].status = 1; // Включаем действия в прерывании, делаем шаги
    setMotorSpeed(0, 360);  // Устанавливаем скорость вращения моторов и в дальнейшем только флагами включаем или отключаем вращение
    setMotorSpeed(1, 60);   // Устанавливаем скорость вращения моторов и в дальнейшем только флагами включаем или отключаем вращение
    setMotorSpeed(2, -360); // Устанавливаем скорость вращения моторов и в дальнейшем только флагами включаем или отключаем вращение
    setMotorSpeed(3, -60);  // Устанавливаем скорость вращения моторов и в дальнейшем только флагами включаем или отключаем вращение

    HAL_GPIO_WritePin(En_Motor_GPIO_Port, En_Motor_Pin, GPIO_PIN_RESET); // Включаем драйвера Установить пин HGH GPIO_PIN_SET — установить HIGH,  GPIO_PIN_RESET — установить LOW.
    // statusTestMotor = 1;                                                 // Статус теста мотора Включаем что тест
    while (1)
        ;
}

// Остановка моторов после теста
void testMotorStop()
{
    motor[0].status = 0;
    motor[1].status = 0;
    motor[2].status = 0;
    motor[3].status = 0;
    HAL_GPIO_WritePin(En_Motor_GPIO_Port, En_Motor_Pin, GPIO_PIN_SET); // Установить пин HGH GPIO_PIN_SET — установить HIGH,  GPIO_PIN_RESET — установить LOW.
    // Serial.println("testMotorStop...");
}

// Отключение моторов в простое
void disableMotor()
{
    if (motor[0].status == 0 && // Если все статусы выключены, значит моторы не двигаются и можно выключать
        motor[1].status == 0 &&
        motor[2].status == 0 &&
        motor[3].status == 0)
    {
        HAL_GPIO_WritePin(En_Motor_GPIO_Port, En_Motor_Pin, GPIO_PIN_SET); // Установить пин HGH GPIO_PIN_SET — установить HIGH,  GPIO_PIN_RESET — установить LOW.
    }
}

// Пересчет градусов в импульсы
int32_t getPulse(float _angle)
{
    return round(_angle / sector);
}

// Пересчет импульсов в градусы
float getAngle(int32_t _pulse)
{
    return _pulse * sector;
}

// Установка мотора в нужное положение в градусах локальной системы
void setMotorAngle(int num, float angle_)
{
    if (angle_ < 1)
        angle_ = 1;   // Защита от отрицательного градуса угла
    if (angle_ > 179) // Защита от отклонения больше предела
        angle_ = 179;

    motor[num].destination = getPulse(angle_); // Получаем в какую позицию должен встать мотор наиболее близкую к требуемому градусу
    DEBUG_PRINTF("+++ num = %i ", num);
    DEBUG_PRINTF(" pos= %i ", motor[num].position);
    DEBUG_PRINTF(" dest= %i \n", motor[num].destination);
    if (motor[num].position == motor[num].destination) // Если текущая позиция и так равна цели то ничего не делаем и выходим из функции
        return;
}
// Расчет угловой скорости для мотора в градусах в секунду dps
float calcAngleSpeedMotor(int num, float angle_, float dt_) // num - номер мотора, angle_ - текущий угол в градусах, dt_ - время прошедшее с прошлого расчета в секундах
{
    float deltaAngle = normalizeAngle(angle_ - motor[num].predAngle); // Находим разницу углов Анализ изменения цели (Feed-Forward)

    float angleSpeed = 0.0f; // По умолчанию 0

    if (deltaAngle != 0) // Если есть изменение угла, иначе скорость 0
    {
        // === ЗАЩИТА ОТ СКАЧКА ===
        if (fabs(deltaAngle) < JUMP_THRESHOLD) // Если изменение меньше порога - значит это плавное вращение платформы
        {
            float rawSpeed = deltaAngle / dt_; // Считаем "сырую" скорость изменения угла в градусах в секунду

            float alpha = 0.5f; // Коэффициент сглаживания (0 < alpha < 1)
            motor[num].filteredSpeed = (motor[num].filteredSpeed * (1.0f - alpha)) + (rawSpeed * alpha);
            angleSpeed = motor[num].filteredSpeed;
        }
        else
        {
            motor[num].filteredSpeed = 0; // Это резкий скачок и фильтр тут вреден. Сбрасываем фильтр, чтобы он не "помнил" этот рывок.
            angleSpeed = 0;
        }
    }
    else
        motor[num].filteredSpeed = 0.0f; // deltaAngle == 0. Цель стоит. обнуляем фильтр

    DEBUG_PRINTF("num = %i NEW angle_ = %f predAngle = %f deltaAngle = %f dt= %f angleSpeed = %f  ", num, angle_, motor[num].predAngle, deltaAngle, dt_, angleSpeed);
    motor[num].predAngle = angle_; // Запоминаем текущий угол для следующего расчета
    return angleSpeed;             // Возвращаем угловую скорость
}

// Обработка прерывания по микрику на пине для мотора0
void isrMicMotor0()
{
    if (flagMicric)
    {
        DEBUG_PRINTF("isrMicMotor0 \n");
        motor[0].status = false;
        motor[0].position = 0;    // Устанавливаем начальную позицию
        motor[0].destination = 0; // Устанавливаем начальную позицию
    }
}

// Обработка прерывания по микрику на пине для мотора1
void isrMicMotor1()
{
    if (flagMicric)
    {
        DEBUG_PRINTF("isrMicMotor1 \n");
        motor[1].status = false;
        motor[1].position = 0;    // Устанавливаем начальную позицию
        motor[1].destination = 0; // Устанавливаем начальную позицию
    }
}

// Обработка прерывания по микрику на пине для мотора2
void isrMicMotor2()
{
    if (flagMicric)
    {
        DEBUG_PRINTF("isrMicMotor2 \n");
        motor[2].status = false;
        motor[2].position = 0;    // Устанавливаем начальную позицию
        motor[2].destination = 0; // Устанавливаем начальную позицию
    }
}

// Обработка прерывания по микрику на пине для мотора3
void isrMicMotor3()
{
    if (flagMicric)
    {
        DEBUG_PRINTF("isrMicMotor3 \n");
        motor[3].status = false;
        motor[3].position = 0;    // Устанавливаем начальную позицию
        motor[3].destination = 0; // Устанавливаем начальную позицию
    }
}

// Функция установки в 10 градусов
void rotationRight()
{
    DEBUG_PRINTF("rotationRight \n");
    flagMicric = false;                                                  // Микрики не реагируют
    HAL_GPIO_WritePin(En_Motor_GPIO_Port, En_Motor_Pin, GPIO_PIN_RESET); // Установить пин HGH GPIO_PIN_SET — установить HIGH,  GPIO_PIN_RESET — установить LOW.
    for (int i = 0; i < 4; i++)                                          // Сначала отводим немного на случай если уже в нуле
    {
        setMotorSpeed(i, 90); // Устанавливаем скорость вращения моторов и в дальнейшем только флагами включаем или отключаем вращение
    }
}

// Функция установки в ноль всех моторов
void rotationLeft()
{
    DEBUG_PRINTF("rotationLeft \n");
    flagMicric = true;                                                   // Микрики реагируют
    HAL_GPIO_WritePin(En_Motor_GPIO_Port, En_Motor_Pin, GPIO_PIN_RESET); // Установить пин HGH GPIO_PIN_SET — установить HIGH,  GPIO_PIN_RESET — установить LOW.
    for (int i = 0; i < 4; i++)
    {
        setMotorSpeed(i, -60); // Устанавливаем скорость вращения моторов и в дальнейшем только флагами включаем или отключаем вращение
    }
}

void workingMotor() // Отработка действий по флагам и таймерам для моторов
{
    if (timerMode9 + 500 < millis() && flagMode9) // Ждем пока на 10 градусов отьедет и запускаем калибровку к нулю включив микрики
    {
        flagMode9 = false;
        flagMode91 = true;
        timerMode9 = millis(); // Снова запоминаем время
        flagMicric = true;     // Микрики включаем реакцию
        rotationLeft();        // Вращаем пока микрики не сработают и это будет нулевой позицией
    }
    if (timerMode9 + 5000 < millis() && flagMode91) // Закончили колибровку
    {
        DEBUG_PRINTF("End colibrovka \n");
        flagMode91 = false;
        flagMicric = false;                                                // Микрики выключаем реакцию
        HAL_GPIO_WritePin(En_Motor_GPIO_Port, En_Motor_Pin, GPIO_PIN_SET); // Выключаем драйвер. Установить пин HGH GPIO_PIN_SET — установить HIGH,  GPIO_PIN_RESET — установить LOW.
        for (int i = 0; i < 4; i++)
        {
            motor[i].status = false; // Отключаем все
            motor[i].position = 0;   // Устанавливаем начальную позицию
        }
    }
}

// Функция установки в ноль всех моторов
void setZeroMotor()
{
    flagMicric = true; // Включаем микрики
    rotationRight();   // Вращение по часовой вправо
    uint32_t timeStart = millis();
    while (timeStart + 500 > millis()) // Ждем
    {
        HAL_Delay(10); // Ждем 100 миллисекунд
    }
    HAL_GPIO_WritePin(En_Motor_GPIO_Port, En_Motor_Pin, GPIO_PIN_SET); // Выключаем драйвер. Установить пин HGH GPIO_PIN_SET — установить HIGH,  GPIO_PIN_RESET — установить LOW.
    DEBUG_PRINTF("------------------------------ End setMotor_10 \n");
    HAL_Delay(1000);

    rotationLeft(); // Вращение против часовой влево
    timeStart = millis();
    while (timeStart + 5000 > millis()) // Ждем 5 секунд пока доедут до концевика
    {
        HAL_Delay(10); // Ждем 100 миллисекунд
    }
    flagMicric = false;                                                // Отключаем микрики от случайных срабатываний
    HAL_GPIO_WritePin(En_Motor_GPIO_Port, En_Motor_Pin, GPIO_PIN_SET); // Выключаем драйвер. Установить пин HGH GPIO_PIN_SET — установить HIGH,  GPIO_PIN_RESET — установить LOW.
    DEBUG_PRINTF("------------------------------ End setMotor_0 \n");
    HAL_Delay(1000);
}

#endif
