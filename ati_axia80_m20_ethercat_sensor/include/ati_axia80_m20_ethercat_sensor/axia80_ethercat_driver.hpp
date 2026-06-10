#ifndef ATI_AXIA80_M20_ETHERCAT_SENSOR__AXIA80_ETHERCAT_DRIVER_HPP_
#define ATI_AXIA80_M20_ETHERCAT_SENSOR__AXIA80_ETHERCAT_DRIVER_HPP_

#include <cstdint>
#include <optional>
#include <string>

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

struct Axia80DriverParameters
{
  unsigned int master_index{0};
  AxiaEtherCATConfig slave;

  // If true, counts-per-force/torque are read from 0x2021:0x37/0x38.
  bool read_calibration_sdo{true};
  double counts_per_force{1000000.0};
  double counts_per_torque{1000000.0};

  uint8_t filter_selection{0};    // 0 = off, 1..8 = sensor low-pass filter choices.
  uint8_t calibration_slot{0};    // 0 or 1 for Axia80 factory calibrations.
  uint8_t sample_rate_code{0};    // 0=487Hz, 1=975Hz, 2=1990Hz, 3=3900Hz.

  bool set_bias_on_activate{false};
  bool clear_bias_on_activate{false};
};

class Axia80EtherCATDriver
{
public:
  explicit Axia80EtherCATDriver(const Axia80DriverParameters & parameters);
  ~Axia80EtherCATDriver();

  Axia80EtherCATDriver(const Axia80EtherCATDriver &) = delete;
  Axia80EtherCATDriver & operator=(const Axia80EtherCATDriver &) = delete;

  void init();
  void shutdown();
  void set_bias();
  void clear_bias();
  Axia80Sample read_once();

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
  int32_t upload_sdo_i32_(uint16_t index, uint8_t subindex) const;

  Axia80DriverParameters parameters_;
  ec_master_t * master_{nullptr};
  ec_domain_t * domain_{nullptr};
  ec_slave_config_t * slave_config_{nullptr};
  ec_master_info_t master_info_{};
  ec_slave_info_t slave_info_{};
  uint8_t * domain_pd_{nullptr};
  AxiaPdoOffsets offsets_{};
  bool active_{false};

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Logger logger_;
};

}  // namespace ati_axia80_m20_ethercat_sensor

#endif  // ATI_AXIA80_M20_ETHERCAT_SENSOR__AXIA80_ETHERCAT_DRIVER_HPP_
