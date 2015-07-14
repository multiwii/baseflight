/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include "board.h"

/* Generic driver for invensense gyro/acc devices.
 *
 * Supported hardware:
 * MPU3050 (gyro only)
 * MPU6050 (gyro + acc)
 * MPU6500 (gyro + acc)
 *
 * AUX_I2C is enabled on devices which have bypass, to allow forwarding to compass in MPU9150-style devices
 */

// This is generally where all Invensense devices are at, for default (AD0 down) I2C address
#define MPU_ADDRESS                         (0x68)
#define GYRO_INT_GPIO                       (GPIOC)
#define GYRO_INT_PIN                        (Pin_13)

#define MPU_RA_WHO_AM_I                     (0x75)
#define MPU_RA_WHO_AM_I_LEGACY              (0x00)
#define MPU_RA_GYRO_XOUT_H                  (0x43)
#define MPU_RA_GYRO_XOUT_LEGACY             (0x1D)
#define MPU_RA_ACCEL_XOUT_H                 (0x3B)
// For debugging/identification purposes
#define MPU_RA_XA_OFFS_H                    (0x06)    //[15:0] XA_OFFS
#define MPU_RA_PRODUCT_ID                   (0x0C)    // Product ID Register

// WHO_AM_I register contents for MPU3050, 6050 and 6500
#define MPU6500_WHO_AM_I_CONST              (0x70)
#define MPUx0x0_WHO_AM_I_CONST              (0x68)

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


// Hardware access functions
static bool mpuReadRegisterI2C(uint8_t reg, uint8_t *data, int length);
static bool mpuWriteRegisterI2C(uint8_t reg, uint8_t data);
static bool mpuReadRegisterSPI(uint8_t reg, uint8_t *data, int length);
static bool mpuWriteRegisterSPI(uint8_t reg, uint8_t data);
static void mpu3050Init(sensor_t *acc, sensor_t *gyro);
static void mpu6050Init(sensor_t *acc, sensor_t *gyro);
static void mpu6500Init(sensor_t *acc, sensor_t *gyro);

// Hardware access funcptrs
typedef bool (*mpuReadRegPtr)(uint8_t reg, uint8_t *data, int length);
typedef bool (*mpuWriteRegPtr)(uint8_t reg, uint8_t data);
typedef void (*mpuInitPtr)(sensor_t *acc, sensor_t *gyro);
// General forward declarations
static void mpu6050CheckRevision(void);
static void dummyInit(sensor_align_e align);
static void dummyRead(int16_t *data);
static void mpuAccInit(sensor_align_e align);
static void mpuAccRead(int16_t *accData);
static void mpuGyroInit(sensor_align_e align);
static void mpuGyroRead(int16_t *gyroData);

typedef struct mpu_access_t {
    mpuReadRegPtr read;
    mpuWriteRegPtr write;
    mpuInitPtr init;

    uint8_t acc_xout;
    uint8_t gyro_xout;
} mpu_access_t;

// Needed for MPU6050 half-scale acc bug
extern uint16_t acc_1G;
// Hardware access function
static mpu_access_t mpu;
// Default orientation
static sensor_align_e gyroAlign = CW0_DEG;
static sensor_align_e accAlign = CW0_DEG;
// Lowpass
static uint8_t mpuLowPassFilter = INV_FILTER_42HZ;

bool mpuDetect(sensor_t *acc, sensor_t *gyro, mpu_params_t *init)
{
    bool ack, useSpi = false;
    uint8_t sig = 0, legacy = 0;
    mpu_hardware_e hw = MPU_NONE;
    gpio_config_t gpio;

    // Set acc_1G. Modified once by mpu6050CheckRevision for old (hopefully nonexistent outside of clones) parts
    acc_1G = 512 * 8;

    // Try I2C access first
    mpu.read = mpuReadRegisterI2C;
    mpu.write = mpuWriteRegisterI2C;
    // Default gyro/acc read offsets (overwritten only by legacy MPU3xxx hardware)
    mpu.gyro_xout = MPU_RA_GYRO_XOUT_H;
    mpu.acc_xout = MPU_RA_ACCEL_XOUT_H;

    ack = mpu.read(MPU_RA_WHO_AM_I, &sig, 1);

    if (!ack) {
        // Let's see if there's something on SPI
        mpu.read = mpuReadRegisterSPI;
        mpu.write = mpuWriteRegisterSPI;
        useSpi = true;

        mpu.read(MPU_RA_WHO_AM_I, &sig, 1);
    } else {
        // Special case for MPU30x0, WHO_AM_I is in a different place
        mpu.read(MPU_RA_WHO_AM_I_LEGACY, &legacy, 1);
        legacy &= 0x7E;
        if (legacy == MPUx0x0_WHO_AM_I_CONST) {
            hw = MPU_3050;
            mpu.init = mpu3050Init;
            mpu.gyro_xout = MPU_RA_GYRO_XOUT_LEGACY;
        }
    }

    sig &= 0x7E; // mask the lower/upper bits per MPUxxxx spec

    if (sig == MPUx0x0_WHO_AM_I_CONST) {
        hw = MPU_60x0;
        mpu6050CheckRevision();
        mpu.init = mpu6050Init;
    } else if (sig == MPU6500_WHO_AM_I_CONST) {
        if (useSpi)
            hw = MPU_65xx_SPI;
        else
            hw = MPU_65xx_I2C;
        mpu.init = mpu6500Init;
    }

    // We're done. Nothing was found on any bus.
    if (hw == MPU_NONE)
        return false;

    // 16.4 dps/lsb scalefactor for all Invensense devices
    gyro->scale = (4.0f / 16.4f) * (M_PI / 180.0f) * 0.000001f;

    // default lpf is 42Hz, 255 is special case of nolpf
    if (init->lpf == 255)
        mpuLowPassFilter = INV_FILTER_256HZ_NOLPF2;
    else if (init->lpf >= 188)
        mpuLowPassFilter = INV_FILTER_188HZ;
    else if (init->lpf >= 98)
        mpuLowPassFilter = INV_FILTER_98HZ;
    else if (init->lpf >= 42)
        mpuLowPassFilter = INV_FILTER_42HZ;
    else if (init->lpf >= 20)
        mpuLowPassFilter = INV_FILTER_20HZ;
    else if (init->lpf >= 10)
        mpuLowPassFilter = INV_FILTER_10HZ;
    else
        mpuLowPassFilter = INV_FILTER_5HZ;

    // MPU_INT output on rev5+ hardware (PC13). rev4 was on PB13, conflicts with SPI devices
    if (hw_revision >= NAZE32_REV5) {
        gpio.pin = GYRO_INT_PIN;
        gpio.speed = Speed_2MHz;
        gpio.mode = Mode_IN_FLOATING;
        gpioInit(GYRO_INT_GPIO, &gpio);
    }

    // initialize the device
    mpu.init(acc, gyro);
    // return detected hardware
    init->deviceType = hw;

    return true;
}

// MPU3050 registers
#define MPU3050_SMPLRT_DIV      0x15
#define MPU3050_DLPF_FS_SYNC    0x16
#define MPU3050_INT_CFG         0x17
#define MPU3050_USER_CTRL       0x3D
#define MPU3050_PWR_MGM         0x3E
// MPU3050 bits
#define MPU3050_FS_SEL_2000DPS  0x18
#define MPU3050_USER_RESET      0x01
#define MPU3050_CLK_SEL_PLL_GX  0x01

static void mpu3050Init(sensor_t *acc, sensor_t *gyro)
{
    mpu.write(MPU3050_SMPLRT_DIV, 0);
    mpu.write(MPU3050_DLPF_FS_SYNC, MPU3050_FS_SEL_2000DPS | mpuLowPassFilter);
    mpu.write(MPU3050_INT_CFG, 0);
    mpu.write(MPU3050_USER_CTRL, MPU3050_USER_RESET);
    mpu.write(MPU3050_PWR_MGM, MPU3050_CLK_SEL_PLL_GX);

    acc->init = dummyInit;
    acc->read = dummyRead;
    gyro->init = mpuGyroInit;
    gyro->read = mpuGyroRead;
}

// MPU6xxx registers
#define MPU_RA_SMPLRT_DIV       0x19
#define MPU_RA_CONFIG           0x1A
#define MPU_RA_GYRO_CONFIG      0x1B
#define MPU_RA_ACCEL_CONFIG     0x1C
#define MPU_RA_INT_PIN_CFG      0x37
#define MPU_RA_INT_ENABLE       0x38
#define MPU_RA_SIGNAL_PATH_RST  0x68
#define MPU_RA_PWR_MGMT_1       0x6B
// MPU6050 bits
#define MPU6050_INV_CLK_GYROZ   0x03

static void mpu6050Init(sensor_t *acc, sensor_t *gyro)
{
    // Device reset
    mpu.write(MPU_RA_PWR_MGMT_1, 0x80); // Device reset
    delay(100);
    // Gyro config
    mpu.write(MPU_RA_SMPLRT_DIV, 0x00); // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    mpu.write(MPU_RA_PWR_MGMT_1, MPU6050_INV_CLK_GYROZ); // Clock source = 3 (PLL with Z Gyro reference)
    delay(10);
    mpu.write(MPU_RA_CONFIG, mpuLowPassFilter); // set DLPF
    mpu.write(MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3); // full-scale 2kdps gyro range
    // Accel scale 8g (4096 LSB/g)
    mpu.write(MPU_RA_ACCEL_CONFIG, INV_FSR_8G << 3);

    // Data ready interrupt configuration
    mpu.write(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);  // INT_RD_CLEAR_DIS, I2C_BYPASS_EN
    mpu.write(MPU_RA_INT_ENABLE, 0x01); // DATA_RDY_EN interrupt enable

    acc->init = mpuAccInit;
    acc->read = mpuAccRead;
    gyro->init = mpuGyroInit;
    gyro->read = mpuGyroRead;
}

static void mpu6500Init(sensor_t *acc, sensor_t *gyro)
{
    // Device reset
    mpu.write(MPU_RA_PWR_MGMT_1, 0x80); // Device reset
    delay(100);
    mpu.write(MPU_RA_SIGNAL_PATH_RST, 0x07); // Signal path reset
    delay(100);
    // Gyro config
    mpu.write(MPU_RA_PWR_MGMT_1, INV_CLK_PLL); // Clock source = 1 (Auto-select PLL or else intrc)
    delay(10);
    mpu.write(MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
    mpu.write(MPU_RA_CONFIG, mpuLowPassFilter); // set DLPF
    mpu.write(MPU_RA_SMPLRT_DIV, 0); // 1kHz S/R
    // Accel config
    mpu.write(MPU_RA_ACCEL_CONFIG, INV_FSR_8G << 3);

    // Data ready interrupt configuration
    mpu.write(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR, BYPASS_EN
    mpu.write(MPU_RA_INT_ENABLE, 0x01); // RAW_RDY_EN interrupt enable

    acc->init = mpuAccInit;
    acc->read = mpuAccRead;
    gyro->init = mpuGyroInit;
    gyro->read = mpuGyroRead;
}

static void dummyInit(sensor_align_e align)
{

}

static void dummyRead(int16_t *data)
{

}

static void mpuAccInit(sensor_align_e align)
{
    if (align > 0)
        accAlign = align;
}

static void mpuGyroInit(sensor_align_e align)
{
    if (align > 0)
        gyroAlign = align;
}

static void mpuAccRead(int16_t *accData)
{
    uint8_t buf[6];
    int16_t data[3];

    mpu.read(mpu.acc_xout, buf, 6);
    data[0] = (int16_t)((buf[0] << 8) | buf[1]);
    data[1] = (int16_t)((buf[2] << 8) | buf[3]);
    data[2] = (int16_t)((buf[4] << 8) | buf[5]);

    alignSensors(data, accData, accAlign);
}

static void mpuGyroRead(int16_t *gyroData)
{
    uint8_t buf[6];
    int16_t data[3];

    mpu.read(mpu.gyro_xout, buf, 6);
    data[0] = (int16_t)((buf[0] << 8) | buf[1]) / 4;
    data[1] = (int16_t)((buf[2] << 8) | buf[3]) / 4;
    data[2] = (int16_t)((buf[4] << 8) | buf[5]) / 4;

    alignSensors(data, gyroData, gyroAlign);
}

static void mpu6050CheckRevision(void)
{
    uint8_t rev;
    uint8_t tmp[6];
    int half = 0;

    // determine product ID and accel revision
    mpu.read(MPU_RA_XA_OFFS_H, tmp, 6);
    rev = ((tmp[5] & 0x01) << 2) | ((tmp[3] & 0x01) << 1) | (tmp[1] & 0x01);
    if (rev) {
        // Congrats, these parts are better
        if (rev == 1) {
            half = 1;
        } else if (rev == 2) {
            half = 0;
        } else {
            failureMode(5);
        }
    } else {
        mpu.read(MPU_RA_PRODUCT_ID, &rev, 1);
        rev &= 0x0F;
        if (!rev) {
            failureMode(5);
        } else if (rev == 4) {
            half = 1;
        } else {
            half = 0;
        }
    }

    // All this just to set the value
    if (half)
        acc_1G = 255 * 8;
}

static bool mpuReadRegisterI2C(uint8_t reg, uint8_t *data, int length)
{
    return i2cRead(MPU_ADDRESS, reg, length, data);
}

static bool mpuWriteRegisterI2C(uint8_t reg, uint8_t data)
{
    return i2cWrite(MPU_ADDRESS, reg, data);
}

static bool mpuReadRegisterSPI(uint8_t reg, uint8_t *data, int length)
{
#ifndef CJMCU
    spiSelect(true);
    spiTransferByte(reg | 0x80); // read transaction
    spiTransfer(data, NULL, length);
    spiSelect(false);
#endif
    return true;
}

static bool mpuWriteRegisterSPI(uint8_t reg, uint8_t data)
{
#ifndef CJMCU
    spiSelect(true);
    spiTransferByte(reg);
    spiTransferByte(data);
    spiSelect(false);
#endif
    return true;
}
