#pragma once

#include <Arduino.h>

#define ESPHOME_LOG_LEVEL_NONE 0
#define ESPHOME_LOG_LEVEL_ERROR 1
#define ESPHOME_LOG_LEVEL_WARN 2
#define ESPHOME_LOG_LEVEL_INFO 3
#define ESPHOME_LOG_LEVEL_CONFIG 4
#define ESPHOME_LOG_LEVEL_DEBUG 5
#define ESPHOME_LOG_LEVEL_VERBOSE 6
#define ESPHOME_LOG_LEVEL_VERY_VERBOSE 7

#define RX_BUFFER_SIZE 256

typedef void (*logFuncPtr) (int level, const char* format, ...);

namespace ant_bms {

class AntBms {
 public:
  void loop();
  void dump_state();
  void update();

  void set_password(const String &password) { this->password_ = password; }
  void set_supports_new_commands(bool supports_new_commands) { this->supports_new_commands_ = supports_new_commands; }
  void set_logFunc(logFuncPtr logFunc) { this->logFunc_ = logFunc; }
  void set_stream(Stream *stream) { this->stream_ = stream; }

  void set_rx_timeout(uint16_t rx_timeout) { rx_timeout_ = rx_timeout; }
  void set_status_timeout(uint16_t status_timeout) { status_timeout_ = status_timeout; }
  void write_register(uint8_t address, uint16_t value);
  bool supports_new_commands() { return supports_new_commands_; }
  bool is_online() {  return (millis() - last_status_ts_) < status_timeout_; }

  bool online_status_binary_sensor_;

  float battery_strings_sensor_;
  float current_sensor_;
  float soc_sensor_;
  float total_battery_capacity_setting_sensor_;
  float capacity_remaining_sensor_;
  float battery_cycle_capacity_sensor_;
  float total_voltage_sensor_;
  float total_runtime_sensor_;
  float average_cell_voltage_sensor_;
  float power_sensor_;
  float min_cell_voltage_sensor_;
  float max_cell_voltage_sensor_;
  float min_voltage_cell_sensor_;
  float max_voltage_cell_sensor_;
  float delta_cell_voltage_sensor_;
  float charge_mosfet_status_code_sensor_;
  float discharge_mosfet_status_code_sensor_;
  float balancer_status_code_sensor_;

  bool charging_switch_;
  bool discharging_switch_;
  bool balancer_switch_;
  bool bluetooth_switch_;
  bool buzzer_switch_;

  const char *charge_mosfet_status_text_sensor_;
  const char *discharge_mosfet_status_text_sensor_;
  const char *balancer_status_text_sensor_;
  //const char *total_runtime_formatted_text_sensor_;

  struct Cell {
    float cell_voltage_sensor_;
  } cells_[32];

  struct Temperature {
    float temperature_sensor_;
  } temperatures_[6];

 protected:
  logFuncPtr logFunc_;
  Stream *stream_;

  bool supports_new_commands_;
  String password_;

  uint8_t rx_buffer_[RX_BUFFER_SIZE];
  size_t rx_buffer_index_{0};

  uint32_t last_status_ts_{0};
  uint16_t status_timeout_{50};

  uint32_t last_rx_ts_{0};
  uint16_t rx_timeout_{50};

  bool on_ant_bms_data(const uint8_t *data, size_t size);
  void on_status_data_(const uint8_t *data, size_t size);
  bool parse_ant_bms_byte_(const uint8_t *data, size_t size);
  void authenticate_();
  void authenticate_v2021_();
  void authenticate_v2021_variable_(const uint8_t *data, uint8_t data_len);
  void read_registers_();
  void send_(uint8_t function, uint8_t address, uint16_t value);
  void send_v2021_(uint8_t function, uint8_t address, uint16_t value);

  uint16_t chksum_(const uint8_t data[], const uint16_t len) {
    uint16_t checksum = 0;
    for (uint16_t i = 4; i < len; i++) {
      checksum = checksum + data[i];
    }
    return checksum;
  }

  uint16_t crc16_(const uint8_t *data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) {
      crc ^= *data++;
      for (uint8_t i = 0; i < 8; i++) {
        if ((crc & 0x01) != 0) {
          crc >>= 1;
          crc ^= 0xA001;
        } else {
          crc >>= 1;
        }
      }
    }
    return crc;
  }

  /*std::string format_total_runtime_(const uint32_t value) {
    int seconds = (int) value;
    int years = seconds / (24 * 3600 * 365);
    seconds = seconds % (24 * 3600 * 365);
    int days = seconds / (24 * 3600);
    seconds = seconds % (24 * 3600);
    int hours = seconds / 3600;
    return (years ? to_string(years) + "y " : "") + (days ? to_string(days) + "d " : "") +
           (hours ? to_string(hours) + "h" : "");
  }*/
};

}  // namespace ant_bms
