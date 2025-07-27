#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include "adc_low_level.h"

static const char *TAG = "MAIN";

void adc_task(void *pvParameters) {
    adc_low_level_config_t adc_config = {
        .gpio_pin = 2,      // GPIO2 (ADC1_CH1)
        .channel = 1,       // ADC1 channel 1
        .atten = 3,         // 11 dB attenuation (0–3.3V)
        .bit_width = 12,    // Fixed 12-bit resolution
        .vref_mv = 1100     // Default Vref
    };

    // Initialize ADC
    esp_err_t ret = adc_low_level_init(&adc_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC initialization failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
    }

    while (1) {
        // Read raw ADC value
        uint16_t raw_value;
        ret = adc_low_level_read(&raw_value);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ADC read failed: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Read voltage
        float voltage_mv;
        ret = adc_read_voltage(&voltage_mv);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Voltage read failed: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Read temperature
        float temperature_c;
        ret = adc_read_temperature(&temperature_c);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Temperature read failed: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Log results
        ESP_LOGI(TAG, "Raw: %u, Voltage: %.2f mV, Temperature: %.2f °C",
                 raw_value, voltage_mv, temperature_c);

        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second
    }
}

void app_main(void) {
    // Create ADC task
    xTaskCreate(adc_task, "adc_task", 4096, NULL, 5, NULL);
}
