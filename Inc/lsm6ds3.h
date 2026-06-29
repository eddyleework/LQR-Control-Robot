#ifndef LSM6DS3_H
#define LSM6DS3_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ================= I2C ADDRESS ================= */
/*
 * SDO 상태에 따라 달라짐:
 * 0x6A or 0x6B (7-bit)
 */
#define LSM6DS3_ADDR_0   (0x6A << 1)
#define LSM6DS3_ADDR_1   (0x6B << 1)

/* 기본 사용 주소 (필요시 변경) */
#define LSM6DS3_ADDR     LSM6DS3_ADDR_0

/* ================= REGISTERS ================= */
#define WHO_AM_I         0x0F

#define CTRL1_XL         0x10
#define CTRL2_G          0x11
#define CTRL3_C          0x12

#define OUTX_L_XL        0x28
#define OUTX_L_G         0x22

/* ================= WHO_AM_I VALUE ================= */
#define LSM6DS3_WHOAMI_VALUE   0x69

/* ================= API ================= */

/* Initialize sensor */
void LSM6DS3_Init(I2C_HandleTypeDef *hi2c);

/* Read WHO_AM_I register */
uint8_t LSM6DS3_WhoAmI(I2C_HandleTypeDef *hi2c);

/* Read full IMU (Accel + Gyro) */
int8_t LSM6DS3_ReadIMU(I2C_HandleTypeDef *hi2c,
                       int16_t *ax, int16_t *ay, int16_t *az,
                       int16_t *gx, int16_t *gy, int16_t *gz);

/* Optional separated APIs */
int8_t LSM6DS3_ReadAccel(I2C_HandleTypeDef *hi2c,
                         int16_t *ax, int16_t *ay, int16_t *az);

int8_t LSM6DS3_ReadGyro(I2C_HandleTypeDef *hi2c,
                        int16_t *gx, int16_t *gy, int16_t *gz);

#endif /* LSM6DS3_H */
