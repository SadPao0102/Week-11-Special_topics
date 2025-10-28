#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define LDR_CHANNEL ADC1_CHANNEL_6   // GPIO34
#define BUZZER_PIN  GPIO_NUM_25      // ขาที่ต่อ Base ผ่าน R1 (Q1 ควบคุม Buzzer)
#define LED_PIN     GPIO_NUM_26      // LED แสดงสถานะ
#define THRESHOLD   1000             // เกณฑ์แสง

static const char *TAG = "LDR_BUZZER";

void app_main(void)
{
    // กำหนด ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LDR_CHANNEL, ADC_ATTEN_DB_11);

    // กำหนด GPIO สำหรับ Buzzer และ LED
    gpio_reset_pin(BUZZER_PIN);
    gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);

    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    while (1) {
        int ldr_value = adc1_get_raw(LDR_CHANNEL);
        ESP_LOGI(TAG, "LDR Value: %d", ldr_value);

        if (ldr_value < THRESHOLD) {
            gpio_set_level(BUZZER_PIN, 1); // เปิด Buzzer
            gpio_set_level(LED_PIN, 1);    // เปิด LED
            ESP_LOGI(TAG, "ALERT: Dark! Buzzer ON, LED ON");
        } else {
            gpio_set_level(BUZZER_PIN, 0); // ปิด Buzzer
            gpio_set_level(LED_PIN, 0);    // ปิด LED
            ESP_LOGI(TAG, "Bright: Buzzer OFF, LED OFF");
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}