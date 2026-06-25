#ifndef ATI_AXIA80_M20_ETHERCAT_SENSOR__AXIA80_ETHERCAT_DRIVER_HPP_
#define ATI_AXIA80_M20_ETHERCAT_SENSOR__AXIA80_ETHERCAT_DRIVER_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"

#include "ati_axia80_m20_ethercat_sensor/axia80_ethercat_config.hpp"

namespace ati_axia80_m20_ethercat_sensor
{

struct Axia80Sample
{
  geometry_msgs::msg::WrenchStamped wrench;
  uint32_t status_code{0};
  uint32_t sample_counter{0};
  bool valid{false};
};

struct Axia80CalibrationInfo
{
  std::string ft_serial;
  std::string calibration_part_number;
  std::string calibration_time;
  double counts_per_force{0.0};
  double counts_per_torque{0.0};
  uint8_t active_calibration_slot{0};
};

struct Axia80DiagnosticReadings
{
  double supply_voltage_v{0.0};
  double gage_temperature_c{0.0};
  std::string status_message;
};

struct Axia80EtherCATState
{
  bool have_domain_state{false};
  unsigned int domain_working_counter{0};
  unsigned int domain_wc_state{0};

  bool have_master_state{false};
  unsigned int master_slaves_responding{0};
  unsigned int master_al_states{0};
  bool master_link_up{false};

  bool have_slave_state{false};
  bool slave_online{false};
  bool slave_operational{false};
  unsigned int slave_al_state{0};
};

struct Axia80DriverParameters
{
  unsigned int master_index{0};
  AxiaEtherCATConfig slave;

  // If true, counts-per-force/torque are read from 0x2021:0x37/0x38.
  bool read_calibration_sdo{true};
  double counts_per_force{1000000.0};
  double counts_per_torque{1000000.0};

  // ATI 0x7010:01 bits 4-7. 0 disables filtering; 1..8 select the
  // manual-defined cutoff table for the configured sample rate.
  uint8_t filter_selection{0};
  uint8_t calibration_slot{0};    // 0 or 1 for Axia80 factory calibrations.
  uint8_t sample_rate_code{1};    // 0=487Hz, 1=975Hz, 2=1990Hz, 3=3900Hz.

  bool set_bias_on_activate{false};
  bool clear_bias_on_activate{true};
};

class Axia80DriverInterface
{
public:
  virtual ~Axia80DriverInterface() = default;
  virtual void init() = 0;
  virtual void shutdown() = 0;
  virtual void set_bias() = 0;
  virtual void clear_bias() = 0;
  virtual bool is_active() const = 0;
  virtual Axia80Sample read_once() = 0;
  virtual Axia80DiagnosticReadings read_diagnostic_readings() const = 0;
  virtual Axia80EtherCATState state_snapshot() const = 0;
};

class Axia80EtherCATDriver : public Axia80DriverInterface
{
public:
  explicit Axia80EtherCATDriver(const Axia80DriverParameters & parameters);
  ~Axia80EtherCATDriver() override;

  Axia80EtherCATDriver(const Axia80EtherCATDriver &) = delete;
  Axia80EtherCATDriver & operator=(const Axia80EtherCATDriver &) = delete;

  void init() override;
  void shutdown() override;
  void set_bias() override;
  void clear_bias() override;
  bool is_active() const override;
  Axia80Sample read_once() override;
  Axia80DiagnosticReadings read_diagnostic_readings() const override;
  Axia80EtherCATState state_snapshot() const override;

private:
  void request_master_();
  void find_slave_();
  void configure_pdos_();
  void read_calibration_sdo_();
  void activate_master_();
  void write_control_(uint32_t control_1, uint32_t control_2);
  void cycle_once_();
  void pulse_control_bit_(uint32_t bit_mask);
  uint32_t control_word_() const;
  void check_ethercat_state_();
  Axia80CalibrationInfo read_calibration_info_sdo_();
  std::vector<uint8_t> upload_sdo_(uint16_t index, uint8_t subindex, size_t size) const;
  uint32_t upload_sdo_u32_(uint16_t index, uint8_t subindex) const;
  int16_t upload_sdo_i16_(uint16_t index, uint8_t subindex) const;
  uint16_t upload_sdo_u16_(uint16_t index, uint8_t subindex) const;
  std::string upload_sdo_string_(uint16_t index, uint8_t subindex, size_t size) const;

  Axia80DriverParameters parameters_;
  ec_master_t * master_{nullptr};
  ec_domain_t * domain_{nullptr};
  ec_slave_config_t * slave_config_{nullptr};
  ec_master_info_t master_info_{};
  ec_slave_info_t slave_info_{};
  uint8_t * domain_pd_{nullptr};
  AxiaPdoOffsets offsets_{};
  bool active_{false};
  ec_domain_state_t last_domain_state_{};
  ec_master_state_t last_master_state_{};
  ec_slave_config_state_t last_slave_state_{};
  bool have_domain_state_{false};
  bool have_master_state_{false};
  bool have_slave_state_{false};
  bool state_check_required_{true};
  std::chrono::steady_clock::time_point next_state_check_time_{};

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Logger logger_;
};

}  // namespace ati_axia80_m20_ethercat_sensor

#endif  // ATI_AXIA80_M20_ETHERCAT_SENSOR__AXIA80_ETHERCAT_DRIVER_HPP_
