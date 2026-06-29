#include "main.h"
#include "motor.h"
#include "tim.h"
#include <stdio.h>
extern TIM_HandleTypeDef htim1;

void Motor_Update(int speed, int direction)
{
    // 1. STBY 핀이 High인지 확인 (드라이버 활성화)
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);

    // 2. 속도가 0일 때 (정지)
    if (speed <= 0)
    {
        HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); // PWM도 0으로
        return; // 여기서 종료
    }

    // 3. 방향 설정 (정방향/역방향)
    if (direction == 1) {
        HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_SET);
    }

    // 4. PWM 설정
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
}
