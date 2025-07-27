#ifndef ADC_LOW_LEVEL_H
#define ADC_LOW_LEVEL_H

#include <esp_err.h>
#include <stdbool.h>

typedef struct {
    int gpio_pin;         // GPIO pin for ADC (e.g., 2 for ADC1_CH1)
    uint8_t channel;      // ADC channel (e.g., 1 for ADC1_CH1)
    uint8_t atten;        // Attenuation (0: 0 dB, 1: 2.5 dB, 2: 6 dB, 3: 11 dB)
    uint8_t bit_width;    // Must be 12 for ESP32-S3 ADC1
    uint32_t vref_mv;     // Reference voltage for calibration (e.g., 1100 mV)
} adc_low_level_config_t;

esp_err_t adc_low_level_init(adc_low_level_config_t *config);
esp_err_t adc_low_level_deinit(void);
esp_err_t adc_low_level_read(uint16_t *raw_value);
esp_err_t adc_read_voltage(float *voltage_mv);
esp_err_t adc_read_temperature(float *temperature_c);

#endif // ADC_LOW_LEVEL_H
