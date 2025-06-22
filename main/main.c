#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "sdkconfig.h"
#include <string.h>
#include "esp_log.h"

#define BUF_SIZE 256

static const char *TAG = "ST8XXX_NM main";

static void send_command(const char *cmd) {
    size_t len = strlen(cmd);
    int tx_bytes = uart_write_bytes(CONFIG_SF8XXX_NM_UART_PORT_NUM, cmd, len);
    ESP_LOGI(TAG, "Sent %d bytes: %s", tx_bytes, cmd);
}

static int receive_response(char *buf, int buf_len, TickType_t timeout)
{
    int idx = 0;
    while (idx < buf_len - 1) {
        uint8_t ch;
        int len = uart_read_bytes(CONFIG_SF8XXX_NM_UART_PORT_NUM, &ch, 1, timeout);
        if (len > 0) {
            buf[idx++] = (char)ch;
            if (ch == '\r') {
                break;
            }
        } else {
            break;
        }
    }
    buf[idx] = '\0';
    return idx;
}

void ST8XXX_NM_task(void *pvParams) {
    uart_config_t uart_SF8XXX_NM_config = {
        .baud_rate = CONFIG_SF8XXX_NM_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(CONFIG_SF8XXX_NM_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CONFIG_SF8XXX_NM_UART_PORT_NUM, &uart_SF8XXX_NM_config));
    ESP_ERROR_CHECK(uart_set_pin(CONFIG_SF8XXX_NM_UART_PORT_NUM, CONFIG_SF8XXX_NM_UART_TXD, CONFIG_SF8XXX_NM_UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    char resp_buf[BUF_SIZE];

    while (1) {
        send_command("J0100\r");
        int len = receive_response(resp_buf, sizeof(resp_buf), pdMS_TO_TICKS(500));

        if (len > 0) {
            ESP_LOGI(TAG, "Received: %s", resp_buf);
            // Expected: K0100 XXXX\r (hex value)
            if (resp_buf[0] == 'K') {
                char *hex_str = strchr(resp_buf, ' ');
                if (hex_str) {
                    uint32_t code = strtoul(hex_str + 1, NULL, 16);
                    float frequency_hz = code * 0.1f;
                    ESP_LOGI(TAG, "Measured Frequency: %.1f Hz", frequency_hz);
                }
            }
        } else {
            ESP_LOGW(TAG, "No response or timeout");
        }

        vTaskDelay(pdMS_TO_TICKS(1500));
    }

    // never reach here
    vTaskDelete(NULL);
}

void app_main(void)
{
    xTaskCreate(ST8XXX_NM_task, "ST8XXX_NM_task", 3 * 1024, NULL, 5, NULL);
}
