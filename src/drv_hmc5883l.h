/**
 * Copyright (C) 2012-2015 baseflight
 *
 * License: http://www.gnu.org/licenses/gpl.html GPL version 3 or higher
 */

#pragma once

bool hmc5883lDetect(sensor_t *mag);
void hmc5883lInit(sensor_align_e align);
void hmc5883lRead(int16_t *magData);
