/**
 * Copyright (C) 2012-2015 baseflight
 *
 * License: http://www.gnu.org/licenses/gpl.html GPL version 3 or higher
 */

#pragma once

typedef struct drv_adxl345_config_t {
    bool useFifo;
    uint16_t dataRate;
} drv_adxl345_config_t;

bool adxl345Detect(drv_adxl345_config_t *init, sensor_t *acc);
