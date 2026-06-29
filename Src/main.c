#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "LSM6DS3.h"
#include <stdio.h>
#include "tim.h"
#include "motor.h"
#include <math.h>
#include "drv_i2c_recovery.h"
#include "stm32f4xx_hal.h"

/* --- 상수 및 설정 --- */
#define FILTER_ALPHA    0.98f
#define FILTER_BETA     0.02f
#define ACC_LPF_ALPHA   0.95f
#define KI              0.23f

/* --- 데이터 타입 정의 --- */
typedef struct {
    float f_pitch, f_roll;
    float acc_pitch, acc_roll; // 구조체에 넣기
} IMU_Angle_t;

/* --- 튜닝 파라미터 구조체 정의 --- */
typedef struct {
    float K1;
    float K2;
    float Ki;
} Controller_Config_t;

/* --- 전역 변수 (구조체로 관리) --- */
// 튜닝 파라미터: 여기서 값을 변경하세요
static Controller_Config_t ctrl = {
    .K1 = 3.0f,
    .K2 = 0.35f,
    .Ki = 0.23f
};

// 제어 상태 변수
float integral = 0.0f;
static float prev_theta = 0.0f;

/* --- 전역 변수 --- */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim1;

volatile uint8_t timer_flag = 0;
static int16_t acc[3], gyro[3];
static uint8_t sensor_lock = 0;
static uint8_t acc_init = 0;

float angle_x = 0.0f, angle_y = 0.0f;
IMU_Angle_t imu_angle = { 0 };

static float acc_x_f = 0.0f;
static float acc_y_f = 0.0f;
static float acc_z_f = 0.0f;

/* --- 함수 프로토타입 --- */
void SystemClock_Config(void);
int LSM6DS3_Init_With_Retry(int retries);
int Sensor_Task(float dt);
void Apply_Complementary_Filter(int16_t *acc, int16_t *gyro, IMU_Angle_t *angle, float dt);
float State_Compute(float theta, float theta_dot, float dt);
void Motor_Task(float pid_out);
void I2C_Bus_Recovery(void);

/* --- 구현부 --- */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
    return ch;
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_TIM2_Init();
    MX_TIM1_Init();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();

    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_MOE_ENABLE(&htim1);

    printf("BOOT START\r\n");

    if (LSM6DS3_Init_With_Retry(5) != 0) {
        printf("Critical Error: Sensor Not Responding!\r\n");
        while (1) {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            HAL_Delay(200);
        }
    }

    uint32_t last_tick = 0;
    int print_counter = 0;
    float scaled_u = 0;

    while (1) {
        if (timer_flag == 1) {
            timer_flag = 0;

            uint32_t current_tick = HAL_GetTick();
            float dt = (float)(current_tick - last_tick) / 1000.0f;
            last_tick = current_tick;
            float u = 0.0f;

            if (Sensor_Task(dt) == 0) {
                Apply_Complementary_Filter(acc, gyro, &imu_angle, dt);

                float theta = imu_angle.f_roll;
                float theta_dot = gyro[0] * 0.00875f;

                u = State_Compute(theta, theta_dot, dt);
                scaled_u = u * 2.0f; // 게인 스케일링
                Motor_Task(scaled_u);
            }

            if (++print_counter >= 10) {
                printf("Roll: %.2f | AccRoll: %.2f | Gyro:%d |I_Term: %.4f | STATE:%.2f\r\n",
                        imu_angle.f_roll, imu_angle.acc_roll,
                        gyro[0], (ctrl.Ki * integral), scaled_u); // ctrl.Ki 사용
                print_counter = 0;
            }
        }
    }
}

float State_Compute(float theta, float theta_dot, float dt) {
    // 이제 전역 구조체 ctrl의 값을 사용합니다
    integral += theta * dt;
    integral *= 0.995f;                 // 누설(Leak)

    if (theta * prev_theta < 0.0f) {    // 영점 통과 시 적분 초기화
        integral = 0.0f;
    }
    prev_theta = theta;                 // 이전 값 갱신

    // Anti-Windup
    if (integral > 1000.0f) integral = 1000.0f;
    if (integral < -1000.0f) integral = -1000.0f;

    // 제어 출력 계산
    return -(ctrl.K1 * theta + ctrl.K2 * theta_dot + ctrl.Ki * integral);
}

void Apply_Complementary_Filter(int16_t *acc, int16_t *gyro, IMU_Angle_t *angle, float dt) {
    if (!acc_init) {
        acc_x_f = (float)acc[0];
        acc_y_f = (float)acc[1];
        acc_z_f = (float)acc[2];
        acc_init = 1;
    }
    acc_x_f = ACC_LPF_ALPHA * acc_x_f + (1.0f - ACC_LPF_ALPHA) * (float)acc[0];
    acc_y_f = ACC_LPF_ALPHA * acc_y_f + (1.0f - ACC_LPF_ALPHA) * (float)acc[1];
    acc_z_f = ACC_LPF_ALPHA * acc_z_f + (1.0f - ACC_LPF_ALPHA) * (float)acc[2];

    float acc_roll = atan2f(acc_y_f, acc_z_f) * 57.2957795f;
    float gyro_roll_rate = (float)gyro[0] * 0.00875f;

    angle->acc_roll = atan2f(acc_y_f, acc_z_f) * 57.2957795f;
    angle->f_roll = FILTER_ALPHA * (angle->f_roll + gyro_roll_rate * dt) + FILTER_BETA * acc_roll;
}

void Motor_Task(float pid_out) {
    int dir = (pid_out >= 0) ? 1 : 2;     // 방향 설정
    float abs_pid = fabsf(pid_out);
    float pwm_f = (dir == 2) ? (abs_pid * 2.0f) : abs_pid; // 역방향 시 스케일 조절

    if (pwm_f > 99.0f) pwm_f = 99.0f;     // PWM 상한 제한
    Motor_Update((int)pwm_f, dir);
}

void I2C_Bus_Recovery(void) {
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    for (int i = 0; i < 9; i++) {         // 클럭 펄스 생성
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_Delay(1);
    }
    MX_I2C1_Init();
}

int Sensor_Task(float dt) {
    if (sensor_lock) return -1;
    sensor_lock = 1;
    int status = LSM6DS3_ReadIMU(&hi2c1, &acc[0], &acc[1], &acc[2], &gyro[0], &gyro[1], &gyro[2]);
    sensor_lock = 0;

    if (status != 1) {
        I2C_Bus_Recovery();
        return -1;
    }
    return 0;
}

int LSM6DS3_Init_With_Retry(int retries) {
    for (int i = 0; i < retries; i++) {
        uint8_t who = LSM6DS3_WhoAmI(&hi2c1);
        if (who == 0x69 || who == 0x6A) {
            LSM6DS3_Init(&hi2c1);
            printf("Sensor Found! ID: 0x%02X\r\n", who);
            return 0;
        }
        printf("Sensor Init Failed (ID: 0x%02X). Retry %d/%d...\r\n", who, i + 1, retries);
        I2C_Bus_Recovery();
        HAL_Delay(500);
    }
    return -1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) timer_flag = 1;
}


void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };


	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);


	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

void Error_Handler(void) {
	__disable_irq();
	while (1) {
	}
}
#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line) {
    printf("Wrong parameters value: file %s on line %d\r\n", file, line);
    while (1);
}
#endif
