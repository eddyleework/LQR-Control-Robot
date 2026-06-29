#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "drv_i2c_recovery.h"
#include <stdio.h>

void I2C_Recovery(I2C_HandleTypeDef *hi2c)
{
    printf("[I2C Recovery] Start\r\n");

    // 1. I2C disable
    HAL_I2C_DeInit(hi2c);

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    gpio.Pin   = GPIO_PIN_6 | GPIO_PIN_7;   // ✔ 올바른 I2C1 핀
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOB, &gpio);

    // idle HIGH
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);

    // 9 clock pulses
    for (int i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        for (volatile int d = 0; d < 200; d++);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        for (volatile int d = 0; d < 200; d++);
    }

    // stop condition
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);

    // re-init I2C
    MX_I2C1_Init();

    HAL_Delay(50);

    printf("[I2C Recovery] Done\r\n");
}
