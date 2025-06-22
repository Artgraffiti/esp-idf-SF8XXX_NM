#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sf8xxx_nm_driver.h"
#include <string.h>
#include "esp_log.h"

static const char *TAG = "ST8XXX_NM main";

void ST8XXX_NM_task(void *pvParams) {
    sf8xxx_nm_init();

    while (1) {
        float frequency_hz;
        ESP_LOGI(TAG, "Set frequency 65.8 Hz");
        sf8xxx_nm_set_frequency(65.8);

        vTaskDelay(pdMS_TO_TICKS(100));

        sf8xxx_nm_get_frequency(&frequency_hz);
        ESP_LOGI(TAG, "Measured Frequency: %.1f Hz", frequency_hz);

        vTaskDelay(pdMS_TO_TICKS(1500));

        ESP_LOGI(TAG, "Set frequency 10.2 Hz");
        sf8xxx_nm_set_frequency(10.2);

        vTaskDelay(pdMS_TO_TICKS(100));

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
