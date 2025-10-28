#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

#define TAG "ADC_POT"
#define POT_CHANNEL ADC_CHANNEL_6   // GPIO34 = ADC1_CH6
#define NO_OF_SAMPLES 64

void app_main(void)
{
    // ---------- 1. สร้าง handle สำหรับ ADC ----------
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    // ---------- 2. กำหนด channel ----------
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,   // รองรับแรงดัน ~3.3V
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, POT_CHANNEL, &config));

    // ---------- 3. Calibration ----------
    adc_cali_handle_t cali_handle = NULL;
    bool cali_enable = false;

    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    if (adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle) == ESP_OK) {
        cali_enable = true;
        ESP_LOGI(TAG, "ใช้การปรับเทียบแบบ Line Fitting");
    } else {
        ESP_LOGW(TAG, "ไม่สามารถเปิดการปรับเทียบ ADC ได้");
    }

    ESP_LOGI(TAG, "เริ่มอ่านค่า Potentiometer (GPIO34)");

    // ---------- 4. Loop อ่านค่า ----------
    while (1) {
        int adc_raw = 0;
        int sum = 0;

        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, POT_CHANNEL, &adc_raw));
            sum += adc_raw;
        }
        adc_raw = sum / NO_OF_SAMPLES;

        int voltage_mv = 0;
        if (cali_enable) {
            adc_cali_raw_to_voltage(cali_handle, adc_raw, &voltage_mv);
        }

        float voltage = voltage_mv / 1000.0f;
        float percent = (adc_raw / 4095.0f) * 100.0f;

        ESP_LOGI(TAG, "ADC: %d | Voltage: %.2fV | %.1f%%", adc_raw, voltage, percent);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
