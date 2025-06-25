#include <stdint.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "sf8xxx_nm.h"
#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "sf8xxx_nm_defs.h"

static const char *TAG = "ST8XXX_NM main";

#define DELAY_BTW_CMDS vTaskDelay(pdMS_TO_TICKS(150))

void ST8XXX_NM_task(void *pvParams) {
    sf8xxx_nm_err_t err = SF8XXX_NM_OK;
    sf8xxx_nm_init();
    gpio_pulldown_en(CONFIG_SF8XXX_NM_UART_RXD);
    gpio_pulldown_en(CONFIG_SF8XXX_NM_UART_TXD);

    uint16_t serial_number, device_id;
    err = sf8xxx_nm_get_serial_number(&serial_number);
    if (err != SF8XXX_NM_OK) {
        ESP_LOGE(TAG, "Device not connected");
        vTaskDelete(NULL);
    }

    sf8xxx_nm_get_device_id(&device_id);
    ESP_LOGI(TAG, "Serial number: %d", serial_number);
    ESP_LOGI(TAG, "Device id: %d", device_id);

    while (1) {
        float frequency_hz;
        ESP_LOGI(TAG, "Set frequency 0.0 Hz");
        sf8xxx_nm_set_frequency(0.0);

        DELAY_BTW_CMDS;

        sf8xxx_nm_get_frequency(&frequency_hz);
        ESP_LOGI(TAG, "Measured Frequency: %.1f Hz", frequency_hz);

        DELAY_BTW_CMDS;

        ESP_LOGI(TAG, "Set frequency 10.2 Hz");
        sf8xxx_nm_set_frequency(10.2);

        DELAY_BTW_CMDS;

        sf8xxx_nm_get_frequency(&frequency_hz);
        ESP_LOGI(TAG, "Measured Frequency: %.1f Hz", frequency_hz);

        vTaskDelay(pdMS_TO_TICKS(1500));
    }

    // never reach here
    vTaskDelete(NULL);
}

void app_main(void)
{
    xTaskCreate(ST8XXX_NM_task, "ST8XXX_NM_task", 3 * 1024, NULL, 5, NULL);
}
