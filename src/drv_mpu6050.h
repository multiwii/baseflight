/**
 * Copyright (C) 2012-2015 baseflight
 *
 * License: http://www.gnu.org/licenses/gpl.html GPL version 3 or higher
 */

#pragma once

bool mpu6050Detect(sensor_t * acc, sensor_t * gyro, uint16_t lpf, uint8_t *scale);
void mpu6050DmpLoop(void);
void mpu6050DmpResetFifo(void);
