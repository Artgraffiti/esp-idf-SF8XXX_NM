#ifndef SF8XXX_NM_H
#define SF8XXX_NM_H

#include "sf8xxx_nm_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

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
sf8xxx_nm_err_t sf8xxx_nm_get_freq(float *freq_val);
sf8xxx_nm_err_t sf8xxx_nm_get_freq_min(float *freq_min);
sf8xxx_nm_err_t sf8xxx_nm_get_freq_max(float *freq_max);
sf8xxx_nm_err_t sf8xxx_nm_set_freq(float freq_val);

// Duration (0.1 ms)
sf8xxx_nm_err_t sf8xxx_nm_get_duration(float *duration_val);
sf8xxx_nm_err_t sf8xxx_nm_get_duration_min(float *duration_min);
sf8xxx_nm_err_t sf8xxx_nm_get_duration_max(float *duration_max);
sf8xxx_nm_err_t sf8xxx_nm_set_duration(float duration_val);

// Current (0.1 mA)
sf8xxx_nm_err_t sf8xxx_nm_get_current(float *current_val);
sf8xxx_nm_err_t sf8xxx_nm_get_current_min(float *current_min);
sf8xxx_nm_err_t sf8xxx_nm_get_current_max(float *current_max);
sf8xxx_nm_err_t sf8xxx_nm_get_current_max_limit(float *current_max_limit);
sf8xxx_nm_err_t sf8xxx_nm_get_measured_current(float *measured_current_val);
sf8xxx_nm_err_t sf8xxx_nm_get_current_protection_threshold(float *threshold_val);
sf8xxx_nm_err_t sf8xxx_nm_set_current(float current_val);
sf8xxx_nm_err_t sf8xxx_nm_set_current_max(float current_max);

// Current set calibration (0.01%)
sf8xxx_nm_err_t sf8xxx_nm_get_calibration_current(float *calibration_val);
sf8xxx_nm_err_t sf8xxx_nm_set_calibration_current(float calibration_val);

// Voltage (0.1 V)
sf8xxx_nm_err_t sf8xxx_nm_get_measured_voltage(float *measured_voltage_val);

// State of the driver
sf8xxx_nm_err_t sf8xxx_nm_get_driver_state(sf8xxx_nm_driver_state_info_t *driver_state);
sf8xxx_nm_err_t sf8xxx_nm_set_driver_state(sf8xxx_nm_driver_state_w_flags_t flag);

// Serial number
sf8xxx_nm_err_t sf8xxx_nm_get_serial_number(uint16_t *serial_number);
sf8xxx_nm_err_t sf8xxx_nm_get_device_id(uint16_t *device_id);

// Lock status (bit mask)
sf8xxx_nm_err_t sf8xxx_nm_get_lock_status(sf8xxx_nm_lock_status_info_t *lock_status);

// Save parameters
sf8xxx_nm_err_t sf8xxx_nm_save_parameters(void);

// Reset parameters
sf8xxx_nm_err_t sf8xxx_nm_reset_parameters(void);

// External NTC sensor temperature (0.1°C)
sf8xxx_nm_err_t sf8xxx_nm_get_external_ntc_temp_lower_limit(float *temp_limit);
sf8xxx_nm_err_t sf8xxx_nm_get_external_ntc_temp_upper_limit(float *temp_limit);
sf8xxx_nm_err_t sf8xxx_nm_get_external_ntc_temp_measured(float *measured_val);
sf8xxx_nm_err_t sf8xxx_nm_get_b25_100_external_ntc(float *b25_100_val);
sf8xxx_nm_err_t sf8xxx_nm_set_external_ntc_sensor_temp_lower_limit(float temp_limit);
sf8xxx_nm_err_t sf8xxx_nm_set_external_ntc_sensor_temp_upper_limit(float temp_limit);
sf8xxx_nm_err_t sf8xxx_nm_set_external_ntc_sensor_b25_100(float *b25_100_val);

// TEC temperature (0.01°C)
sf8xxx_nm_err_t sf8xxx_nm_get_tec_temp(float *temp_val);
sf8xxx_nm_err_t sf8xxx_nm_get_tec_temp_max(float *temp_max);
sf8xxx_nm_err_t sf8xxx_nm_get_tec_temp_min(float *temp_min);
sf8xxx_nm_err_t sf8xxx_nm_get_tec_temp_max_limit(float *temp_limit);
sf8xxx_nm_err_t sf8xxx_nm_get_tec_temp_min_limit(float *temp_limit);
sf8xxx_nm_err_t sf8xxx_nm_get_tec_temp_measured(float *measured_val);
sf8xxx_nm_err_t sf8xxx_nm_set_tec_temp(float temp_val);
sf8xxx_nm_err_t sf8xxx_nm_set_tec_temp_max(float temp_max);
sf8xxx_nm_err_t sf8xxx_nm_set_tec_temp_min(float temp_min);

// TEC current (0.1 A)
sf8xxx_nm_err_t sf8xxx_nm_get_tec_measured_current(float *measured_val);
sf8xxx_nm_err_t sf8xxx_nm_get_tec_current_limit(float *current_limit);
sf8xxx_nm_err_t sf8xxx_nm_set_tec_current_limit(float current_limit);

// TEC voltage (0.1 V)
sf8xxx_nm_err_t sf8xxx_nm_get_tec_measured_voltage(float *measured_val);

// State of the TEC
sf8xxx_nm_err_t sf8xxx_nm_get_tec_state(sf8xxx_nm_tec_state_info_t *tec_state);
sf8xxx_nm_err_t sf8xxx_nm_set_tec_state(sf8xxx_nm_tec_state_w_flags_t flag);

// Current set callibration (0.01%)
sf8xxx_nm_err_t sf8xxx_nm_get_tec_calibration_current(float *calibration_val);
sf8xxx_nm_err_t sf8xxx_nm_set_tec_calibration_current(float calibration_val);

// Internal LD NTC sensor
sf8xxx_nm_err_t sf8xxx_nm_get_internal_ld_ntc_sensor_b25_100(uint16_t *b25_100_val);
sf8xxx_nm_err_t sf8xxx_nm_set_internal_ld_ntc_sensor_b25_100(uint16_t b25_100_val);

// P coefficient
sf8xxx_nm_err_t sf8xxx_nm_get_p_coefficient(uint16_t *p_coeff);
sf8xxx_nm_err_t sf8xxx_nm_set_p_coefficient(uint16_t p_coeff);

// I coefficient
sf8xxx_nm_err_t sf8xxx_nm_get_i_coefficient(uint16_t *i_coeff);
sf8xxx_nm_err_t sf8xxx_nm_set_i_coefficient(uint16_t i_coeff);

// D coefficient
sf8xxx_nm_err_t sf8xxx_nm_get_d_coefficient(uint16_t *d_coeff);
sf8xxx_nm_err_t sf8xxx_nm_set_d_coefficient(uint16_t d_coeff);

#ifdef __cplusplus
}
#endif

#endif  // SF8XXX_NM_H