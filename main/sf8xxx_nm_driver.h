#ifndef SFXXX_NM_DRIVER_H
#define SFXXX_NM_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

#define SF8XXX_NM_UART_PORT_NUM CONFIG_SF8XXX_NM_UART_PORT_NUM
#define SF8XXX_NM_UART_BAUDRATE CONFIG_SF8XXX_NM_UART_BAUDRATE
#define SF8XXX_NM_UART_TXD CONFIG_SF8XXX_NM_UART_TXD
#define SF8XXX_NM_UART_RXD CONFIG_SF8XXX_NM_UART_RXD
#define SF8XXX_NM_RX_BUF_SIZE 256
#define SF8XXX_NM_TX_BUF_SIZE 256

typedef enum {
    SF8XXX_E0000 = 0, // Internal buffer overflow, cannot find <CR>/<LF>, invalid command format 
    SF8XXX_E0001 = 1, // Unknown command or failed to interpret 
    SF8XXX_E0002 = 2, // CRC mismatch (for extended protocol) 
    SF8XXX_OK    = 3, // Operation successful
    SF8XXX_ERROR_TIMEOUT = 4, // Timeout during communication
    SF8XXX_ERROR_PARSE = 5, // Error parsing response
    SF8XXX_ERROR_INVALID_PARAM = 6, // Invalid parameter value
    SF8XXX_ERROR_RESERVED = 7,
} sf8xxx_nm_err_t;

sf8xxx_nm_err_t sf8xxx_nm_init(void);
void sf8xxx_nm_deinit(void);

int sf8xxx_nm_send_command(const char *command);
int sf8xxx_nm_receive_response(char *buffer, int buffer_len);

// Helper function to send P-type (set) commands
sf8xxx_nm_err_t sf8xxx_nm_set_parameter(uint16_t param_num, uint16_t value);
// Helper function to send J-type (get) commands and receive K-type responses
sf8xxx_nm_err_t sf8xxx_nm_get_parameter(uint16_t param_num, uint16_t *value);

// Datasheet p.20 table "Available parameters and its description"
// Frequency
sf8xxx_nm_err_t sf8xxx_nm_set_frequency(float frequency_hz);
sf8xxx_nm_err_t sf8xxx_nm_get_frequency(float *frequency_hz);
sf8xxx_nm_err_t sf8xxx_nm_get_min_frequency(float *frequency_hz);
sf8xxx_nm_err_t sf8xxx_nm_get_max_frequency(float *frequency_hz);

// // Duration
sf8xxx_nm_err_t sf8xxx_nm_set_duration(uint16_t duration_ms);
sf8xxx_nm_err_t sf8xxx_nm_get_duration(uint16_t *duration_ms);
sf8xxx_nm_err_t sf8xxx_nm_get_min_duration(uint16_t *duration_ms);
sf8xxx_nm_err_t sf8xxx_nm_get_max_duration(uint16_t *duration_ms);

// Current
sf8xxx_nm_err_t sf8xxx_nm_set_current(float current_ma);
sf8xxx_nm_err_t sf8xxx_nm_get_current(float *current_ma);
sf8xxx_nm_err_t sf8xxx_nm_get_min_current(float *current_ma);
sf8xxx_nm_err_t sf8xxx_nm_get_max_current(float *current_ma);
sf8xxx_nm_err_t sf8xxx_nm_set_max_current(float current_ma);
sf8xxx_nm_err_t sf8xxx_nm_get_max_limit_current(float *current_ma);
sf8xxx_nm_err_t sf8xxx_nm_get_measured_current(float *current_ma);
sf8xxx_nm_err_t sf8xxx_nm_get_current_protection_threshold(float *current_ma);

// Current set calibration
sf8xxx_nm_err_t sf8xxx_nm_set_current_calibration(float calibration_percent);
sf8xxx_nm_err_t sf8xxx_nm_get_current_calibration(float *calibration_percent);

// Voltage
sf8xxx_nm_err_t sf8xxx_nm_get_voltage(uint16_t *voltage_v);

// State of the driver
sf8xxx_nm_err_t sf8xxx_nm_set_driver_state(uint16_t state_mask);
sf8xxx_nm_err_t sf8xxx_nm_get_driver_state(uint16_t *state_mask);

// Serial number
sf8xxx_nm_err_t sf8xxx_nm_get_serial_number(uint16_t *serial_number);

// Lock status (bit mask)
sf8xxx_nm_err_t sf8xxx_nm_get_lock_state(uint16_t *state_mask);

// Save parameters
sf8xxx_nm_err_t sf8xxx_nm_save_parameters(void);

// Reset parameters
sf8xxx_nm_err_t sf8xxx_nm_reset_parameters(void);

// External NTC sensor temperature

// TEC temperature
sf8xxx_nm_err_t sf8xxx_nm_set_tec_temperature(float temperature_celsius);
sf8xxx_nm_err_t sf8xxx_nm_get_tec_temperature(float *temperature_celsius);

// TEC current
sf8xxx_nm_err_t sf8xxx_nm_get_tec_current(float *current);

// TEC voltage
sf8xxx_nm_err_t sf8xxx_nm_get_tec_voltage(float *voltage);

// State of the TEC
sf8xxx_nm_err_t sf8xxx_nm_set_tec_state(uint16_t state_mask);
sf8xxx_nm_err_t sf8xxx_nm_get_tec_state(uint16_t *state_mask);

// Current set callibration
sf8xxx_nm_err_t sf8xxx_nm_set_tec_current_calibration(float calibration_percent);
sf8xxx_nm_err_t sf8xxx_nm_get_tec_current_calibration(float *calibration_percent);

// Internal LD NTC sensor

// P coefficient
sf8xxx_nm_err_t sf8xxx_nm_set_p_coefficient(uint16_t p_coef);
sf8xxx_nm_err_t sf8xxx_nm_get_p_coefficient(uint16_t *p_coef);

// I coefficient
sf8xxx_nm_err_t sf8xxx_nm_set_i_coefficient(uint16_t i_coef);
sf8xxx_nm_err_t sf8xxx_nm_get_i_coefficient(uint16_t *i_coef);

// D coefficient
sf8xxx_nm_err_t sf8xxx_nm_set_d_coefficient(uint16_t d_coef);
sf8xxx_nm_err_t sf8xxx_nm_get_d_coefficient(uint16_t *d_coef);

#endif  // SFXXX_NM_DRIVER_H