// การทดลองที่ 2: เซนเซอร์แสง LDR (เวอร์ชันใหม่สำหรับ ESP-IDF v5.4)
// ใช้ ADC One-shot API และ ADC Calibration API

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

#define LDR_CHANNEL     ADC_CHANNEL_7       // GPIO35 (ADC1_CH7)
#define LDR_UNIT        ADC_UNIT_1
#define DEFAULT_VREF    1100
#define NO_OF_SAMPLES   64

static const char *TAG = "LDR_SENSOR";

// ฟังก์ชันสร้าง calibration handle
static bool adc_calibration_init(adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret;

    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = LDR_UNIT,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "สร้าง calibration scheme แบบ Line Fitting สำเร็จ");
        *out_handle = handle;
        return true;
    } else {
        ESP_LOGW(TAG, "ไม่สามารถสร้าง calibration handle ได้");
        return false;
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "เริ่มต้นระบบอ่านค่า LDR ด้วย ADC (ESP-IDF v5.4)");

    // ---------- 1. สร้าง ADC one-shot handle ----------
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = LDR_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    // ---------- 2. กำหนด channel ----------
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,   // 0–3.3V
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, LDR_CHANNEL, &config));

    // ---------- 3. สร้าง calibration handle ----------
    adc_cali_handle_t cali_handle = NULL;
    bool do_calibration = adc_calibration_init(&cali_handle);

    // ---------- 4. อ่านค่า ADC แบบต่อเนื่อง ----------
    while (1) {
        uint32_t adc_sum = 0;
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            int raw;
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, LDR_CHANNEL, &raw));
            adc_sum += raw;
        }
        uint32_t adc_reading = adc_sum / NO_OF_SAMPLES;

        // ---------- 5. แปลงเป็นแรงดัน ----------
        int voltage_mv = 0;
        if (do_calibration) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, adc_reading, &voltage_mv));
        } else {
            // fallback: คำนวณโดยประมาณ
            voltage_mv = (adc_reading * 3300) / 4095;
        }
        float voltage = voltage_mv / 1000.0f;

        // ---------- 6. ประเมินระดับแสง ----------
        float light_percent = (adc_reading / 4095.0f) * 100.0f;
        const char *light_status;
        if (light_percent < 20)
            light_status = "มืด";
        else if (light_percent < 50)
            light_status = "แสงน้อย";
        else if (light_percent < 80)
            light_status = "แสงปานกลาง";
        else
            light_status = "แสงจ้า";

        // ---------- 7. แสดงผล ----------
        ESP_LOGI(TAG, "ADC: %lu | แรงดัน: %.2fV | ระดับแสง: %.1f%% | สถานะ: %s",
                 (unsigned long)adc_reading, voltage, light_percent, light_status);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // ---------- 8. ปล่อยทรัพยากร ----------
    if (do_calibration) {
        adc_cali_delete_scheme_line_fitting(cali_handle);
    }
    adc_oneshot_del_unit(adc1_handle);
}
