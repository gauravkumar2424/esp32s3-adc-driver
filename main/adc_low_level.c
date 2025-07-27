#include "adc_low_level.h"
#include <soc/soc.h>
#include <soc/apb_saradc_reg.h>
#include <soc/system_reg.h>
#include <driver/gpio.h>
#include <esp_rom_sys.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <math.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))

static const char *TAG = "ADC_LOW_LEVEL";

#define ADC_TIMEOUT_MS 100    // ADC conversion timeout
#define ADC_MAX_RETRIES 3     // Number of retries for failed conversions

// ADC register definitions
#define ADC1_DATA_REG (APB_SARADC_SAR1_STATUS_REG) // ADC1 data
#define ADC_INT_RAW_REG (APB_SARADC_INT_RAW_REG)   // Interrupt raw status
#define ADC_INT_CLR_REG (APB_SARADC_INT_CLR_REG)   // Interrupt clear
#define ADC_CTRL_REG (APB_SARADC_CTRL_REG)         // ADC control
#define ADC_CTRL2_REG (APB_SARADC_CTRL2_REG)       // ADC control 2

static adc_low_level_config_t *adc_config = NULL;
static SemaphoreHandle_t adc_mutex = NULL;

static void gpio_set_analog(int pin) {
    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&conf));
}

esp_err_t adc_low_level_init(adc_low_level_config_t *config) {
    if (!config || config->gpio_pin < 0 || config->channel > 9 || config->atten > 3) {
        ESP_LOGE(TAG, "Invalid config");
        return ESP_ERR_INVALID_ARG;
    }
    if (config->bit_width != 12) {
        ESP_LOGE(TAG, "ESP32-S3 ADC1 only supports 12-bit resolution");
        return ESP_ERR_INVALID_ARG;
    }

    adc_mutex = xSemaphoreCreateMutex();
    if (!adc_mutex) {
        ESP_LOGE(TAG, "Failed to create ADC mutex");
        return ESP_ERR_NO_MEM;
    }

    adc_config = config;

    // Enable ADC peripheral clock
    SET_PERI_REG_MASK(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_APB_SARADC_CLK_EN);
    CLEAR_PERI_REG_MASK(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_APB_SARADC_RST);

    // Configure GPIO as analog
    gpio_set_analog(config->gpio_pin);

    // Configure ADC1 in one-shot mode
    REG_WRITE(ADC_CTRL_REG, 0); // Clear control register
    REG_SET_FIELD(ADC_CTRL_REG, APB_SARADC_SAR1_PATT_LEN, 0); // Single pattern
    REG_WRITE(APB_SARADC_SAR1_PATT_TAB1_REG, (config->channel << 6) | (config->atten << 3)); // Channel and attenuation
    REG_SET_FIELD(ADC_CTRL_REG, APB_SARADC_START_FORCE, 1); // Enable software trigger

    ESP_LOGI(TAG, "ADC1 initialized: GPIO=%d, Channel=%d, Atten=%d, Bit Width=12",
             config->gpio_pin, config->channel, config->atten);
    return ESP_OK;
}

esp_err_t adc_low_level_read(uint16_t *raw_value) {
    if (!raw_value) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(adc_mutex, pdMS_TO_TICKS(ADC_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire ADC mutex");
        return ESP_ERR_TIMEOUT;
    }

    int retries = ADC_MAX_RETRIES;
    esp_err_t ret = ESP_ERR_TIMEOUT;
    while (retries-- > 0) {
        // Start conversion
        REG_SET_FIELD(ADC_CTRL_REG, APB_SARADC_START, 1);
        vTaskDelay(pdMS_TO_TICKS(2)); // Increased to 2 ms

        // Check if conversion is done
        if (REG_GET_FIELD(ADC_INT_RAW_REG, APB_SARADC_ADC1_DONE_INT_RAW)) {
            *raw_value = REG_READ(ADC1_DATA_REG) & 0xFFF; // 12-bit mask
            REG_SET_FIELD(ADC_INT_CLR_REG, APB_SARADC_ADC1_DONE_INT_CLR, 1); // Clear interrupt
            ESP_LOGD(TAG, "ADC read successful: raw=%u", *raw_value); // Debug log
            ret = ESP_OK;
            break;
        }
        ESP_LOGW(TAG, "ADC read failed, retrying (%d left)", retries);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    xSemaphoreGive(adc_mutex);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC read failed after %d retries", ADC_MAX_RETRIES);
    }
    return ret;
}

esp_err_t adc_read_voltage(float *voltage_mv) {
    uint16_t raw_value;
    esp_err_t ret = adc_low_level_read(&raw_value);
    if (ret != ESP_OK) {
        return ret;
    }

    // Convert raw value to voltage (simplified linear model)
    float vref = adc_config->vref_mv;
    float max_value = 4095.0; // 12-bit max
    float atten_scale;
    switch (adc_config->atten) {
        case 0: atten_scale = 1.0; break; // 0 dB: 0–1.1V
        case 1: atten_scale = 1.34; break; // 2.5 dB: 0–1.5V
        case 2: atten_scale = 2.0; break; // 6 dB: 0–2.2V
        case 3: atten_scale = 3.6; break; // 11 dB: 0–3.3V
        default: atten_scale = 3.6;
    }
    *voltage_mv = (raw_value / max_value) * vref * atten_scale;
    return ESP_OK;
}

esp_err_t adc_read_temperature(float *temperature_c) {
    float voltage_mv;
    esp_err_t ret = adc_read_voltage(&voltage_mv);
    if (ret != ESP_OK) {
        return ret;
    }

    // Assume NTC thermistor in voltage divider with 10k resistor
    float vref = adc_config->vref_mv;
    float r_fixed = 10000.0; // 10k resistor
    if (voltage_mv >= vref) {
        ESP_LOGE(TAG, "Invalid voltage: %.2f mV exceeds Vref", voltage_mv);
        return ESP_ERR_INVALID_STATE;
    }
    float r_ntc = r_fixed * (voltage_mv / (vref - voltage_mv));

    // Simplified NTC temperature calculation (Steinhart-Hart)
    float beta = 3950.0;
    float r25 = 10000.0; // Resistance at 25°C
    float t0 = 298.15;   // 25°C in Kelvin
    float temperature_k = 1.0 / ((1.0 / t0) + (1.0 / beta) * log(r_ntc / r25));
    *temperature_c = temperature_k - 273.15; // Convert to Celsius

    return ESP_OK;
}

esp_err_t adc_low_level_deinit(void) {
    if (adc_mutex) {
        vSemaphoreDelete(adc_mutex);
        adc_mutex = NULL;
    }

    // Disable ADC peripheral
    REG_WRITE(ADC_CTRL_REG, 0); // Clear control register
    CLEAR_PERI_REG_MASK(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_APB_SARADC_CLK_EN);
    SET_PERI_REG_MASK(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_APB_SARADC_RST);

    // Reset GPIO to input
    ESP_ERROR_CHECK(gpio_set_direction(adc_config->gpio_pin, GPIO_MODE_INPUT));

    ESP_LOGI(TAG, "ADC1 deinitialized");
    return ESP_OK;
}
