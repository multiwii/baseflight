/**
 * Copyright (C) 2012-2015 baseflight
 *
 * License: http://www.gnu.org/licenses/gpl.html GPL version 3 or higher
 */

#pragma once

bool ak8975detect(sensor_t *mag);
void ak8975Init(sensor_align_e align);
void ak8975Read(int16_t *magData);
