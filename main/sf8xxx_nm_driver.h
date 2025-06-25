#ifndef SF8XXX_NM_DRIVER_H
#define SF8XXX_NM_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

#define SF8XXX_NM_UART_PORT_NUM CONFIG_SF8XXX_NM_UART_PORT_NUM
#define SF8XXX_NM_UART_BAUDRATE CONFIG_SF8XXX_NM_UART_BAUDRATE
#define SF8XXX_NM_UART_TXD CONFIG_SF8XXX_NM_UART_TXD
#define SF8XXX_NM_UART_RXD CONFIG_SF8XXX_NM_UART_RXD
#define SF8XXX_NM_RX_BUF_SIZE 256
#define SF8XXX_NM_TX_BUF_SIZE 256

// TODO:
// typedef enum {
// } sf8xxx_nm_param_code_t;

typedef enum {
    SF8XXX_NM_E0000 = 0, // Internal buffer overflow, cannot find <CR>/<LF>, invalid command format 
    SF8XXX_NM_E0001 = 1, // Unknown command or failed to interpret 
    SF8XXX_NM_E0002 = 2, // CRC mismatch (for extended protocol) 
    SF8XXX_NM_OK    = 3, // Operation successful
    SF8XXX_NM_ERROR_TIMEOUT = 4, // Timeout during communication
    SF8XXX_NM_ERROR_PARSE = 5, // Error parsing response
    SF8XXX_NM_ERROR_INVALID_PARAM = 6, // Invalid parameter value
    SF8XXX_NM_ERROR_RESERVED = 7,
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
// Frequency (0.1 Hz)
sf8xxx_nm_err_t sf8xxx_nm_set_frequency(float frequency_hz);
sf8xxx_nm_err_t sf8xxx_nm_get_frequency(float *frequency_hz);
sf8xxx_nm_err_t sf8xxx_nm_get_min_frequency(float *frequency_hz);
sf8xxx_nm_err_t sf8xxx_nm_get_max_frequency(float *frequency_hz);

// Duration (0.1 ms)
sf8xxx_nm_err_t sf8xxx_nm_set_duration(uint16_t duration_ms);
sf8xxx_nm_err_t sf8xxx_nm_get_duration(uint16_t *duration_ms);
sf8xxx_nm_err_t sf8xxx_nm_get_min_duration(uint16_t *duration_ms);
sf8xxx_nm_err_t sf8xxx_nm_get_max_duration(uint16_t *duration_ms);

// Current (0.1 mA)
sf8xxx_nm_err_t sf8xxx_nm_set_current(float current_ma);
sf8xxx_nm_err_t sf8xxx_nm_get_current(float *current_ma);
sf8xxx_nm_err_t sf8xxx_nm_get_min_current(float *current_ma);
sf8xxx_nm_err_t sf8xxx_nm_get_max_current(float *current_ma);
sf8xxx_nm_err_t sf8xxx_nm_set_max_current(float current_ma);
sf8xxx_nm_err_t sf8xxx_nm_get_max_limit_current(float *current_ma);
sf8xxx_nm_err_t sf8xxx_nm_get_measured_current(float *current_ma);
sf8xxx_nm_err_t sf8xxx_nm_get_current_protection_threshold(float *current_ma);

// Current set calibration (0.01%)
sf8xxx_nm_err_t sf8xxx_nm_set_current_calibration(float calibration_percent);
sf8xxx_nm_err_t sf8xxx_nm_get_current_calibration(float *calibration_percent);

// Voltage (0.1 V)
sf8xxx_nm_err_t sf8xxx_nm_get_measured_voltage(uint16_t *voltage);

// State of the driver
sf8xxx_nm_err_t sf8xxx_nm_set_driver_state(uint16_t driver_state_mask);
sf8xxx_nm_err_t sf8xxx_nm_get_driver_state(uint16_t *driver_state_mask);

// Serial number
sf8xxx_nm_err_t sf8xxx_nm_get_serial_number(uint16_t *serial_number);
sf8xxx_nm_err_t sf8xxx_nm_get_device_id(uint16_t *device_id);

// Lock status (bit mask)
sf8xxx_nm_err_t sf8xxx_nm_get_lock_status(uint16_t *lock_status_mask);

// Save parameters
sf8xxx_nm_err_t sf8xxx_nm_save_parameters(void);

// Reset parameters
sf8xxx_nm_err_t sf8xxx_nm_reset_parameters(void);

// External NTC sensor temperature (0.1°C)
sf8xxx_nm_err_t sf8xxx_nm_set_ext_ntc_lower_limit(float temperature_celsius);
sf8xxx_nm_err_t sf8xxx_nm_get_ext_ntc_lower_limit(float *temperature_celsius);
sf8xxx_nm_err_t sf8xxx_nm_set_ext_ntc_upper_limit(float temperature_celsius);
sf8xxx_nm_err_t sf8xxx_nm_get_ext_ntc_upper_limit(float *temperature_celsius);
sf8xxx_nm_err_t sf8xxx_nm_get_ext_ntc_measured_temp(float *temperature_celsius);
// TODO: code 0B0E

// TEC temperature (0.01°C)
sf8xxx_nm_err_t sf8xxx_nm_set_tec_temp(float temperature_celsius);
sf8xxx_nm_err_t sf8xxx_nm_get_tec_temp(float *temperature_celsius);
sf8xxx_nm_err_t sf8xxx_nm_set_tec_max_temp(float temperature_celsius);
sf8xxx_nm_err_t sf8xxx_nm_get_tec_min_temp(float *temperature_celsius);
sf8xxx_nm_err_t sf8xxx_nm_get_tec_max_limit_temp(float *temperature_celsius);
sf8xxx_nm_err_t sf8xxx_nm_get_tec_min_limit_temp(float *temperature_celsius);
sf8xxx_nm_err_t sf8xxx_nm_get_tec_measured_temp(float *temperature_celsius);

// TEC current (0.1 A)
sf8xxx_nm_err_t sf8xxx_nm_get_tec_measured_current(float *current);
sf8xxx_nm_err_t sf8xxx_nm_set_tec_current_limit(float current);
sf8xxx_nm_err_t sf8xxx_nm_get_tec_current_limit(float *current);

// TEC voltage (0.1 V)
sf8xxx_nm_err_t sf8xxx_nm_get_tec_measured_voltage(float *voltage);

// State of the TEC
sf8xxx_nm_err_t sf8xxx_nm_set_tec_state(uint16_t tec_state_mask);
sf8xxx_nm_err_t sf8xxx_nm_get_tec_state(uint16_t *tec_state_mask);

// Current set callibration (0.01%)
sf8xxx_nm_err_t sf8xxx_nm_set_tec_current_calibration(float calibration_percent);
sf8xxx_nm_err_t sf8xxx_nm_get_tec_current_calibration(float *calibration_percent);

// Internal LD NTC sensor
// TODO: code 0A1F

// P coefficient
sf8xxx_nm_err_t sf8xxx_nm_set_p_coefficient(uint16_t p_coef);
sf8xxx_nm_err_t sf8xxx_nm_get_p_coefficient(uint16_t *p_coef);

// I coefficient
sf8xxx_nm_err_t sf8xxx_nm_set_i_coefficient(uint16_t i_coef);
sf8xxx_nm_err_t sf8xxx_nm_get_i_coefficient(uint16_t *i_coef);

// D coefficient
sf8xxx_nm_err_t sf8xxx_nm_set_d_coefficient(uint16_t d_coef);
sf8xxx_nm_err_t sf8xxx_nm_get_d_coefficient(uint16_t *d_coef);

#endif  // SF8XXX_NM_DRIVER_H