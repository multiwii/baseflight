#pragma once

typedef enum {
    ADC_BATTERY = 0,
    ADC_EXTERNAL_PAD = 1,
    ADC_EXTERNAL_CURRENT = 2,
    ADC_CHANNEL_MAX = 3
} AdcChannel;

typedef struct drv_adc_config_t {
    uint8_t powerAdcChannel;     // which channel used for current monitor, allowed PA1, PB1 (ADC_Channel_1, ADC_Channel_9)
} drv_adc_config_t;

void adcInit(drv_adc_config_t *init);
uint16_t adcGetChannel(uint8_t channel);
