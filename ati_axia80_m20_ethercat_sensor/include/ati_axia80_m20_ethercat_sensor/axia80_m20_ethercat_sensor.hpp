#ifndef ATI_AXIA80_M20_ETHERCAT_SENSOR__AXIA80_M20_ETHERCAT_SENSOR_HPP_
#define ATI_AXIA80_M20_ETHERCAT_SENSOR__AXIA80_M20_ETHERCAT_SENSOR_HPP_

#include <mutex>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "ati_axia80_m20_ethercat_sensor/axia80_ethercat_driver.hpp"

namespace ati_axia80_m20_ethercat_sensor
{

constexpr char LOGGER_NAME[] = "ATIAxia80M20EtherCATSensor";

class Axia80M20EtherCATSensor : public hardware_interface::SensorInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void parse_parameters_(const hardware_interface::HardwareInfo & hardware_info);
  void reset_state_();
  std::string sensor_name_() const;
  void create_bias_services_();
  bool driver_ready_() const;

  Axia80DriverParameters parameters_;
  std::unique_ptr<Axia80EtherCATDriver> driver_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_bias_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_bias_service_;
  mutable std::mutex driver_mutex_;

  geometry_msgs::msg::WrenchStamped measurement_;
  double timestamp_sec_{0.0};
  double timestamp_nanosec_{0.0};
  double status_code_{0.0};
  double sample_counter_{0.0};
};

}  // namespace ati_axia80_m20_ethercat_sensor

#endif  // ATI_AXIA80_M20_ETHERCAT_SENSOR__AXIA80_M20_ETHERCAT_SENSOR_HPP_
