#pragma once

bool hcsr04_init(SonarHardware config);
void hcsr04_get_distance(volatile int32_t *distance);
