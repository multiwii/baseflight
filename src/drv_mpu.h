/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#pragma once

typedef enum mpu_hardware_e {
    MPU_NONE,       // Nothing was found
    MPU_3050,       // Naze32 rev0..4
    MPU_60x0,       // Naze32 rev5
    MPU_65xx_I2C,   // Naze32 rev6
    MPU_65xx_SPI,   // Naze32SP
} mpu_hardware_e;

typedef struct mpu_params_t {
    // Input
    uint8_t lpf;

    // Output
    mpu_hardware_e deviceType;  // Detection result
} mpu_params_t;

bool mpuDetect(sensor_t *acc, sensor_t *gyro, mpu_params_t *init);
