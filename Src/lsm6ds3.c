#include "LSM6DS3.h"

/* 내부 helper */
static void write_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val)
{
    HAL_I2C_Mem_Write(hi2c, LSM6DS3_ADDR, reg,
                      I2C_MEMADD_SIZE_8BIT,
                      &val, 1, 100);
}

static void read_reg(I2C_HandleTypeDef *hi2c, uint8_t reg,
                      uint8_t *data, uint16_t len)
{
    HAL_I2C_Mem_Read(hi2c, LSM6DS3_ADDR, reg,
                     I2C_MEMADD_SIZE_8BIT,
                     data, len, 100);
}

/* ================= INIT ================= */
void LSM6DS3_Init(I2C_HandleTypeDef *hi2c)
{
    write_reg(hi2c, CTRL1_XL, 0xA0); // accel 416Hz ±2g
    write_reg(hi2c, CTRL2_G,  0x60); // gyro 416Hz
    write_reg(hi2c, CTRL3_C,  0x44); // BDU + auto-increment
}

/* ================= WHO_AM_I ================= */
uint8_t LSM6DS3_WhoAmI(I2C_HandleTypeDef *hi2c)
{
    uint8_t who = 0;
    read_reg(hi2c, WHO_AM_I, &who, 1);
    return who;
}

/* ================= ACCEL READ ================= */
int8_t LSM6DS3_ReadIMU(I2C_HandleTypeDef *hi2c,
                       int16_t *ax, int16_t *ay, int16_t *az,
                       int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t acc[6];
    uint8_t gyr[6];

    /* ACCEL */
    if (HAL_I2C_Mem_Read(hi2c, LSM6DS3_ADDR,
                         OUTX_L_XL,
                         I2C_MEMADD_SIZE_8BIT,
                         acc, 6, 100) != HAL_OK)
        return -1;

    /* GYRO */
    if (HAL_I2C_Mem_Read(hi2c, LSM6DS3_ADDR,
                         OUTX_L_G,
                         I2C_MEMADD_SIZE_8BIT,
                         gyr, 6, 100) != HAL_OK)
        return -2;

    /* parse accel */
    *ax = (int16_t)(acc[1] << 8 | acc[0]);
    *ay = (int16_t)(acc[3] << 8 | acc[2]);
    *az = (int16_t)(acc[5] << 8 | acc[4]);

    /* parse gyro */
    *gx = (int16_t)(gyr[1] << 8 | gyr[0]);
    *gy = (int16_t)(gyr[3] << 8 | gyr[2]);
    *gz = (int16_t)(gyr[5] << 8 | gyr[4]);

    return 1;
}


