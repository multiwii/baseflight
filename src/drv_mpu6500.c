/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include "board.h"

// MPU6500 (WHO_AM_I 0x70) on SPI bus

#define MPU6500_RA_WHOAMI                   (0x75)
#define MPU6500_RA_ACCEL_XOUT_H             (0x3B)
#define MPU6500_RA_GYRO_XOUT_H              (0x43)
#define MPU6500_RA_BANK_SEL                 (0x6D)
#define MPU6500_RA_MEM_RW                   (0x6F)
#define MPU6500_RA_GYRO_CFG                 (0x1B)
#define MPU6500_RA_PWR_MGMT_1               (0x6B)
#define MPU6500_RA_ACCEL_CFG                (0x1C)
#define MPU6500_RA_LPF                      (0x1A)
#define MPU6500_RA_RATE_DIV                 (0x19)

#define MPU6500_WHO_AM_I_CONST              (0x70)
#define BIT_RESET                           (0x80)

enum lpf_e {
    INV_FILTER_256HZ_NOLPF2 = 0,
    INV_FILTER_188HZ,
    INV_FILTER_98HZ,
    INV_FILTER_42HZ,
    INV_FILTER_20HZ,
    INV_FILTER_10HZ,
    INV_FILTER_5HZ,
    INV_FILTER_2100HZ_NOLPF,
    NUM_FILTER
};

enum gyro_fsr_e {
    INV_FSR_250DPS = 0,
    INV_FSR_500DPS,
    INV_FSR_1000DPS,
    INV_FSR_2000DPS,
    NUM_GYRO_FSR
};

enum clock_sel_e {
    INV_CLK_INTERNAL = 0,
    INV_CLK_PLL,
    NUM_CLK
};

enum accel_fsr_e {
    INV_FSR_2G = 0,
    INV_FSR_4G,
    INV_FSR_8G,
    INV_FSR_16G,
    NUM_ACCEL_FSR
};

static uint8_t mpuLowPassFilter = INV_FILTER_42HZ;
static sensor_align_e gyroAlign = CW0_DEG;
static sensor_align_e accAlign = CW0_DEG;

static void mpu6500AccInit(sensor_align_e align);
static void mpu6500AccRead(int16_t *accData);
static void mpu6500GyroInit(sensor_align_e align);
static void mpu6500GyroRead(int16_t *gyroData);

extern uint16_t acc_1G;

static void mpu6500WriteRegister(uint8_t reg, uint8_t data)
{
    spiSelect(true);
    spiTransferByte(reg);
    spiTransferByte(data);
    spiSelect(false);
}

static void mpu6500ReadRegister(uint8_t reg, uint8_t *data, int length)
{
    spiSelect(true);
    spiTransferByte(reg | 0x80); // read transaction
    spiTransfer(data, NULL, length);
    spiSelect(false);
}

bool mpu6500Detect(sensor_t *acc, sensor_t *gyro, uint16_t lpf)
{
    uint8_t tmp;

    mpu6500ReadRegister(MPU6500_RA_WHOAMI, &tmp, 1);
    if (tmp != MPU6500_WHO_AM_I_CONST)
        return false;

    acc->init = mpu6500AccInit;
    acc->read = mpu6500AccRead;
    gyro->init = mpu6500GyroInit;
    gyro->read = mpu6500GyroRead;

    // 16.4 dps/lsb scalefactor
    gyro->scale = (4.0f / 16.4f) * (M_PI / 180.0f) * 0.000001f;

    // default lpf is 42Hz
    if (lpf >= 188)
        mpuLowPassFilter = INV_FILTER_188HZ;
    else if (lpf >= 98)
        mpuLowPassFilter = INV_FILTER_98HZ;
    else if (lpf >= 42)
        mpuLowPassFilter = INV_FILTER_42HZ;
    else if (lpf >= 20)
        mpuLowPassFilter = INV_FILTER_20HZ;
    else if (lpf >= 10)
        mpuLowPassFilter = INV_FILTER_10HZ;
    else
        mpuLowPassFilter = INV_FILTER_5HZ;

    return true;
}

static void mpu6500AccInit(sensor_align_e align)
{
    acc_1G = 512 * 8;

    if (align > 0)
        accAlign = align;
}

static void mpu6500AccRead(int16_t *accData)
{
    uint8_t buf[6];
    int16_t data[3];

    mpu6500ReadRegister(MPU6500_RA_ACCEL_XOUT_H, buf, 6);

    data[0] = (int16_t)((buf[0] << 8) | buf[1]);
    data[1] = (int16_t)((buf[2] << 8) | buf[3]);
    data[2] = (int16_t)((buf[4] << 8) | buf[5]);

    alignSensors(data, accData, accAlign);
}

static void mpu6500GyroInit(sensor_align_e align)
{
    gpio_config_t gpio;

    // MPU_INT output on rev5 hardware (PC13). rev4 was on PB13, conflicts with SPI devices
    if (hw_revision >= NAZE32_REV5) {
        gpio.pin = Pin_13;
        gpio.speed = Speed_2MHz;
        gpio.mode = Mode_IN_FLOATING;
        gpioInit(GPIOC, &gpio);
    }

    mpu6500WriteRegister(MPU6500_RA_PWR_MGMT_1, BIT_RESET);
    delay(100);
    mpu6500WriteRegister(MPU6500_RA_PWR_MGMT_1, 0);
    delay(100);
    mpu6500WriteRegister(MPU6500_RA_PWR_MGMT_1, INV_CLK_PLL);
    mpu6500WriteRegister(MPU6500_RA_GYRO_CFG, INV_FSR_2000DPS << 3);
    mpu6500WriteRegister(MPU6500_RA_ACCEL_CFG, INV_FSR_8G << 3);
    mpu6500WriteRegister(MPU6500_RA_LPF, mpuLowPassFilter);
    mpu6500WriteRegister(MPU6500_RA_RATE_DIV, 0); // 1kHz S/R

    if (align > 0)
        gyroAlign = align;
}

static void mpu6500GyroRead(int16_t *gyroData)
{
    uint8_t buf[6];
    int16_t data[3];

    mpu6500ReadRegister(MPU6500_RA_GYRO_XOUT_H, buf, 6);

    data[0] = (int16_t)((buf[0] << 8) | buf[1]) / 4;
    data[1] = (int16_t)((buf[2] << 8) | buf[3]) / 4;
    data[2] = (int16_t)((buf[4] << 8) | buf[5]) / 4;

    alignSensors(data, gyroData, gyroAlign);
}
