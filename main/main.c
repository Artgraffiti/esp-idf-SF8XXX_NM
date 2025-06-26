#include <stdint.h>
#include <stdio.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "sf8xxx_nm.h"
#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "sf8xxx_nm_defs.h"

static const char *TAG = "ST8XXX_NM main";

#define DELAY_BTW_CMDS vTaskDelay(pdMS_TO_TICKS(150))

void print_driver_state(sf8xxx_nm_driver_state_info_t driver_state) {
    ESP_LOGI(TAG, "--- Состояние драйвера ---");
    ESP_LOGI(TAG, "Драйвер ВКЛ: %s", driver_state.is_powered_on ? "ДА" : "НЕТ");
    ESP_LOGI(TAG, "Драйвер ЗАПУЩЕН: %s", driver_state.is_started ? "ДА" : "НЕТ");
    ESP_LOGI(TAG, "Ток установлен внутренним: %s", driver_state.current_set_internal ? "ДА" : "НЕТ");
    ESP_LOGI(TAG, "Включение внутреннее: %s", driver_state.enable_internal ? "ДА" : "НЕТ");
    ESP_LOGI(TAG, "Интерлок внешнего NTC отклонен: %s", driver_state.ext_ntc_interlock_denied ? "ДА" : "НЕТ");
    ESP_LOGI(TAG, "Интерлок отклонен: %s", driver_state.interlock_denied ? "ДА" : "НЕТ");
}

void print_tec_state(const sf8xxx_nm_tec_state_info_t tec_state) {
    ESP_LOGI(TAG, "--- Состояние TEC ---");
    ESP_LOGI(TAG, "TEC ЗАПУЩЕН: %s", tec_state.is_started ? "ДА" : "НЕТ");
    ESP_LOGI(TAG, "Температура TEC установлена внутренним: %s", tec_state.temp_set_internal ? "ДА" : "НЕТ");
    ESP_LOGI(TAG, "TEC включен внутренним: %s", tec_state.enable_internal ? "ДА" : "НЕТ");
}

void ST8XXX_NM_task(void *pvParams) {
    sf8xxx_nm_err_t err = SF8XXX_NM_OK;
    sf8xxx_nm_init();
    ESP_ERROR_CHECK(gpio_pulldown_en(CONFIG_SF8XXX_NM_UART_RXD));
    ESP_ERROR_CHECK(gpio_pulldown_en(CONFIG_SF8XXX_NM_UART_TXD));

    uint16_t serial_number, device_id;
    err = sf8xxx_nm_get_serial_number(&serial_number);
    if (err != SF8XXX_NM_OK) {
        ESP_LOGE(TAG, "Device not connected");
        vTaskDelete(NULL);
    }

    sf8xxx_nm_get_device_id(&device_id);
    ESP_LOGI(TAG, "Serial number: %d", serial_number);
    ESP_LOGI(TAG, "Device id: %d", device_id);

    float frequency_hz, duration_ms, current_val, tec_temp;
    err = sf8xxx_nm_set_freq(0.0);
    if (err != SF8XXX_NM_OK) {
        ESP_LOGE(TAG, "Ошибка установки частоты.");
    }
    DELAY_BTW_CMDS;

    err = sf8xxx_nm_get_freq(&frequency_hz);
    if (err != SF8XXX_NM_OK) {
        ESP_LOGE(TAG, "Ошибка чтения частоты.");
    } else {
        ESP_LOGI(TAG, "Установленная частота испульса: %.1f Hz.", frequency_hz);
    }
    DELAY_BTW_CMDS;

    err = sf8xxx_nm_set_duration(6.9);
    if (err != SF8XXX_NM_OK) {
        ESP_LOGE(TAG, "Ошибка установки продолжительности импульса.");
    }
    DELAY_BTW_CMDS;

    err = sf8xxx_nm_get_duration(&duration_ms);
    if (err != SF8XXX_NM_OK) {
        ESP_LOGE(TAG, "Ошибка чтения продолжительности импульса.");
    } else {
        ESP_LOGI(TAG, "Установленная продолжительность испульса: %.1f ms.", duration_ms);
    }
    DELAY_BTW_CMDS;

    err = sf8xxx_nm_set_current(150.0);
    if (err != SF8XXX_NM_OK) {
        ESP_LOGE(TAG, "Ошибка установки тока на ЛД.");
    }
    DELAY_BTW_CMDS;

    err = sf8xxx_nm_get_current(&current_val);
    if (err != SF8XXX_NM_OK) {
        ESP_LOGE(TAG, "Ошибка чтения тока на ЛД.");
    } else {
        ESP_LOGI(TAG, "Установленный ток на ЛД: %.1f mA.", current_val);
    }
    DELAY_BTW_CMDS;

    err = sf8xxx_nm_set_tec_temp(17.0);
    if (err != SF8XXX_NM_OK) {
        ESP_LOGE(TAG, "Ошибка установки температуры на TEC.");
    }
    DELAY_BTW_CMDS;

    err = sf8xxx_nm_get_tec_temp(&tec_temp);
    if (err != SF8XXX_NM_OK) {
        ESP_LOGE(TAG, "Ошибка чтения температуры на TEC.");
    } else {
        ESP_LOGI(TAG, "Установленная температура на TEC: %.1f C.", tec_temp);
    }
    DELAY_BTW_CMDS;

    // Preparing TEC for launch
    sf8xxx_nm_tec_state_info_t tec_state;
    err = sf8xxx_nm_get_tec_state(&tec_state);
    if (err != SF8XXX_NM_OK) {
        ESP_LOGE(TAG, "Ошибка чтения состояния TEC.");
    } else {
        ESP_LOGI(TAG, "Начальное состояние TEC:");
        print_tec_state(tec_state);
    }
    DELAY_BTW_CMDS;
    
    err = sf8xxx_nm_set_tec_state(SF8XXX_NM_TEC_STATE_WRITE_INTERNAL_ENABLE);
    if (err != SF8XXX_NM_OK) {
        ESP_LOGE(TAG, "Ошибка установки состояния TEC.");
    }
    DELAY_BTW_CMDS;

    err = sf8xxx_nm_set_tec_state(SF8XXX_NM_TEC_STATE_WRITE_INTERNAL_TEMP_SET);
    if (err != SF8XXX_NM_OK) {
        ESP_LOGE(TAG, "Ошибка установки состояния TEC.");
    }
    DELAY_BTW_CMDS;

    err = sf8xxx_nm_set_tec_state(SF8XXX_NM_TEC_STATE_WRITE_START);
    if (err != SF8XXX_NM_OK) {
        ESP_LOGE(TAG, "Ошибка установки состояния TEC.");
    } else {
        ESP_LOGI(TAG, "TEC ЗАПУЩЕН!");
    }
    DELAY_BTW_CMDS;

    err = sf8xxx_nm_get_tec_state(&tec_state);
    if (err != SF8XXX_NM_OK) {
        ESP_LOGE(TAG, "Ошибка чтения состояния TEC.");
    } else {
        ESP_LOGI(TAG, "Начальное состояние TEC:");
        print_tec_state(tec_state);
    }
    DELAY_BTW_CMDS;

    // Preparing the LD for launch
    sf8xxx_nm_driver_state_info_t driver_state;
    err = sf8xxx_nm_get_driver_state(&driver_state);
    if (err != SF8XXX_NM_OK) {
        ESP_LOGE(TAG, "Ошибка чтения состояния драйвера.");
    } else {
        ESP_LOGI(TAG, "Начальное состояние драйвера:");
        print_driver_state(driver_state);
    }
    DELAY_BTW_CMDS;

    err = sf8xxx_nm_set_driver_state(SF8XXX_NM_DRIVER_STATE_WRITE_INTERNAL_CURRENT_SET);
    if (err != SF8XXX_NM_OK) {
        ESP_LOGE(TAG, "Ошибка установки состояния драйвера.");
    }
    DELAY_BTW_CMDS;

    err = sf8xxx_nm_set_driver_state(SF8XXX_NM_DRIVER_STATE_WRITE_INTERNAL_ENABLE);
    if (err != SF8XXX_NM_OK) {
        ESP_LOGE(TAG, "Ошибка установки состояния драйвера.");
    }
    DELAY_BTW_CMDS;

    err = sf8xxx_nm_set_driver_state(SF8XXX_NM_DRIVER_STATE_WRITE_DENY_INTERLOCK);
    if (err != SF8XXX_NM_OK) {
        ESP_LOGE(TAG, "Ошибка установки состояния драйвера.");
    }
    DELAY_BTW_CMDS;

    float tec_mes_val = -100;
    float ext_ntc_sens = -100;
    while (1) {
        err = sf8xxx_nm_set_driver_state(SF8XXX_NM_DRIVER_STATE_WRITE_START);
        if (err != SF8XXX_NM_OK) {
            ESP_LOGE(TAG, "Ошибка включения ЛД.");
        } else {
            ESP_LOGI(TAG, "ЛД вкл.");
        }
        DELAY_BTW_CMDS;
        
        vTaskDelay(pdMS_TO_TICKS(4000));

        err = sf8xxx_nm_get_tec_temp_measured(&tec_mes_val);
        if (err != SF8XXX_NM_OK) {
            ESP_LOGE(TAG, "Ошибка запроса замера температуры.");
        } else {
            ESP_LOGI(TAG, "Температура на TEC: %.1f", tec_mes_val);
        }
        DELAY_BTW_CMDS;

        err = sf8xxx_nm_get_external_ntc_temp_measured(&ext_ntc_sens);
        if (err != SF8XXX_NM_OK) {
            ESP_LOGE(TAG, "Ошибка запроса замера с NTC.");
        } else {
            ESP_LOGI(TAG, "Показания с NTC: %.1f", ext_ntc_sens);
        }
        DELAY_BTW_CMDS;

        err = sf8xxx_nm_get_tec_temp_measured(&tec_mes_val);

    }

    // never reach here
    vTaskDelete(NULL);
}

void app_main(void)
{
    xTaskCreate(ST8XXX_NM_task, "ST8XXX_NM_task", 3 * 1024, NULL, 5, NULL);
}
