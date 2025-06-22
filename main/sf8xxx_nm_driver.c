#include "sf8xxx_nm_driver.h"

#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"

#define RESPONSE_TIMEOUT_MS 1000

const static char *TAG = "SF8XXX_NM DRIVER";

sf8xxx_nm_err_t sf8xxx_nm_init(void) {
    uart_config_t uart_config = {
        .baud_rate = SF8XXX_NM_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(SF8XXX_NM_UART_PORT_NUM, SF8XXX_NM_RX_BUF_SIZE, SF8XXX_NM_TX_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(SF8XXX_NM_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(SF8XXX_NM_UART_PORT_NUM, SF8XXX_NM_UART_TXD, SF8XXX_NM_UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART initialized on port %d with baud rate %d", SF8XXX_NM_UART_PORT_NUM, SF8XXX_NM_UART_BAUDRATE);
    return ESP_OK;
}

void sf8xxx_nm_deinit(void) {
    uart_driver_delete(SF8XXX_NM_UART_PORT_NUM);
    ESP_LOGI(TAG, "UART deinitialized on port %d", SF8XXX_NM_UART_PORT_NUM);
}

int sf8xxx_nm_send_command(const char *command) {
    size_t len = strlen(command);
    int tx_bytes = uart_write_bytes(SF8XXX_NM_UART_PORT_NUM, command, len);
    ESP_LOGD(TAG, "Sent %d bytes: %s", tx_bytes, command);
    return tx_bytes;
}

int sf8xxx_nm_receive_response(char *buffer, int buffer_len) {
    int idx = 0;
    TickType_t timeout_ticks = pdMS_TO_TICKS(RESPONSE_TIMEOUT_MS);

    while (idx < buffer_len - 1) {
        uint8_t ch;
        int len = uart_read_bytes(SF8XXX_NM_UART_PORT_NUM, &ch, 1, timeout_ticks);
        if (len > 0) {
            buffer[idx++] = (char)ch;
            if (ch == '\r') {
                break; // End of command
            }
        } else {
            ESP_LOGW(TAG, "Timeout or no data received.");
            break;
        }
    }
    buffer[idx] = '\0';
    ESP_LOGD(TAG, "Received %d bytes: %s", idx, buffer);
    return idx;
}

sf8xxx_nm_err_t sf8xxx_nm_set_parameter(uint16_t param_num, uint16_t value) {
    char command[SF8XXX_NM_TX_BUF_SIZE];
    snprintf(command, SF8XXX_NM_TX_BUF_SIZE, "P%04X %04X\r", param_num, value);
    if (sf8xxx_nm_send_command(command) < 0) {
        return SF8XXX_ERROR_RESERVED;
    }
    return SF8XXX_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_parameter(uint16_t param_num, uint16_t *value) {
    char command[SF8XXX_NM_TX_BUF_SIZE];
    char response[SF8XXX_NM_RX_BUF_SIZE];
    int bytes_read;

    snprintf(command, SF8XXX_NM_TX_BUF_SIZE, "J%04X\r", param_num);
    if (sf8xxx_nm_send_command(command) < 0) {
        return SF8XXX_ERROR_RESERVED;
    }

    bytes_read = sf8xxx_nm_receive_response(response, SF8XXX_NM_RX_BUF_SIZE);
    if (bytes_read <= 0) {
        return SF8XXX_ERROR_RESERVED;
    }

    // Expected response format: K<param_hex> <value_hex><CR>
    uint16_t received_param_num;
    unsigned int received_value_u;

    if (sscanf(response, "K%4hX %4X\r", &received_param_num, &received_value_u) != 2) {
        return SF8XXX_ERROR_PARSE;
    }
    if (received_param_num != param_num) {
        return SF8XXX_ERROR_PARSE;
    }

    *value = (uint16_t)received_value_u;
    return SF8XXX_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_set_frequency(float frequency_hz) {
    uint16_t value = (uint16_t)(frequency_hz * 10.0f);
    return sf8xxx_nm_set_parameter(0x0100, value);
}

sf8xxx_nm_err_t sf8xxx_nm_get_frequency(float *frequency_hz) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_parameter(0x0100, &value);
    if (err == SF8XXX_OK) {
        *frequency_hz = (float)value / 10.0f;
    }
    return err;
}

sf8xxx_nm_err_t sf8xxx_nm_get_min_frequency(float *frequency_hz) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_parameter(0x0101, &value);
    if (err == SF8XXX_OK) {
        *frequency_hz = (float)value / 10.0f;
    }
    return err;
}

sf8xxx_nm_err_t sf8xxx_nm_get_max_frequency(float *frequency_hz) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_parameter(0x0102, &value);
    if (err == SF8XXX_OK) {
        *frequency_hz = (float)value / 10.0f;
    }
    return err;
}

sf8xxx_nm_err_t sf8xxx_nm_set_duration(uint16_t duration_ms) {
    uint16_t value = (uint16_t)(duration_ms * 10.0f);
    return sf8xxx_nm_set_parameter(0x0200, value);
}

sf8xxx_nm_err_t sf8xxx_nm_get_duration(uint16_t *duration_ms) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_parameter(0x0200, &value);
    if (err == SF8XXX_OK) {
        *duration_ms = (float)value / 10.0f;
    }
    return err;
}

sf8xxx_nm_err_t sf8xxx_nm_get_min_duration(uint16_t *duration_ms) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_parameter(0x0201, &value);
    if (err == SF8XXX_OK) {
        *duration_ms = (float)value / 10.0f;
    }
    return err;
}

sf8xxx_nm_err_t sf8xxx_nm_get_max_duration(uint16_t *duration_ms) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_parameter(0x0202, &value);
    if (err == SF8XXX_OK) {
        *duration_ms = (float)value / 10.0f;
    }
    return err;
}

sf8xxx_nm_err_t sf8xxx_nm_set_current(float current_ma) {
    uint16_t value = (uint16_t)(current_ma * 10.0f);
    return sf8xxx_nm_set_parameter(0x0300, value);
}

sf8xxx_nm_err_t sf8xxx_nm_get_current(float *current_ma) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_parameter(0x0300, &value);
    if (err == SF8XXX_OK) {
        *current_ma = (float)value / 10.0f;
    }
    return err;
}

sf8xxx_nm_err_t sf8xxx_nm_get_min_current(float *current_ma) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_parameter(0x0301, &value);
    if (err == SF8XXX_OK) {
        *current_ma = (float)value / 10.0f;
    }
    return err;
}

sf8xxx_nm_err_t sf8xxx_nm_get_max_current(float *current_ma) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_parameter(0x0302, &value);
    if (err == SF8XXX_OK) {
        *current_ma = (float)value / 10.0f;
    }
    return err;
}

sf8xxx_nm_err_t sf8xxx_nm_set_max_current(float current_ma) {
    uint16_t value = (uint16_t)(current_ma * 10.0f);
    return sf8xxx_nm_set_parameter(0x0300, value);
}

sf8xxx_nm_err_t sf8xxx_nm_get_max_limit_current(float *current_ma) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_parameter(0x0306, &value);
    if (err == SF8XXX_OK) {
        *current_ma = (float)value / 10.0f;
    }
    return err;
}

sf8xxx_nm_err_t sf8xxx_nm_get_measured_current(float *current_ma) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_parameter(0x0307, &value);
    if (err == SF8XXX_OK) {
        *current_ma = (float)value / 10.0f;
    }
    return err;
}

sf8xxx_nm_err_t sf8xxx_nm_get_current_protection_threshold(float *current_ma) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_parameter(0x0308, &value);
    if (err == SF8XXX_OK) {
        *current_ma = (float)value / 10.0f;
    }
    return err;
}
