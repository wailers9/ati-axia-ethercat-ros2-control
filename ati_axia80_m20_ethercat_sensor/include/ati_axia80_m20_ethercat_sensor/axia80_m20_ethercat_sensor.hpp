#ifndef ATI_AXIA80_M20_ETHERCAT_SENSOR__AXIA80_M20_ETHERCAT_SENSOR_HPP_
#define ATI_AXIA80_M20_ETHERCAT_SENSOR__AXIA80_M20_ETHERCAT_SENSOR_HPP_

#include <chrono>
#include <cstdint>
#include <future>
#include <mutex>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "ati_axia80_m20_ethercat_sensor/axia80_ethercat_driver.hpp"

namespace ati_axia80_m20_ethercat_sensor
{

constexpr char LOGGER_NAME[] = "ATIAxia80M20EtherCATSensor";

struct RuntimeDiagnosticSdoResult
{
  enum class Outcome
  {
    SUCCESS,
    SKIPPED,
    FAILED
  };

  Outcome outcome{Outcome::FAILED};
  Axia80DiagnosticReadings readings;
  std::string message;
  uint64_t elapsed_us{0};
};

struct SampleCounterDiagnostics
{
  uint8_t level{0};
  std::string message{"sample counter OK"};
  double elapsed_sec{1.0};
  double expected_sensor_rate_hz{975.0};
  double expected_read_rate_hz{975.0};
  double expected_repeats_per_sec{0.0};
  double actual_repeats_per_sec{0.0};
  double expected_skipped_samples_per_sec{0.0};
  double expected_jump_events_per_sec{0.0};
  double actual_skipped_samples_per_sec{0.0};
  double actual_jump_events_per_sec{0.0};
  uint64_t repeated_reads{0};
  uint64_t skipped_samples{0};
  uint64_t jump_events{0};
  uint32_t max_delta{0};
  uint32_t large_jump_threshold{10};
  uint32_t consecutive_repeats{0};
  uint32_t max_consecutive_repeats{0};
};

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
  void create_diagnostics_publisher_();
  void publish_diagnostics_();
  RuntimeDiagnosticSdoResult read_runtime_diagnostic_sdo_();
  void handle_status_code_(uint32_t status_code);
  void check_sample_counter_(uint32_t sample_counter);
  SampleCounterDiagnostics sample_counter_diagnostics_();
  bool driver_ready_() const;

  Axia80DriverParameters parameters_;
  std::unique_ptr<Axia80EtherCATDriver> driver_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_bias_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_bias_service_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  mutable std::mutex driver_mutex_;
  bool runtime_diagnostic_sdo_enabled_{true};
  std::chrono::milliseconds runtime_diagnostic_sdo_timeout_{5};
  std::future<RuntimeDiagnosticSdoResult> runtime_diagnostic_sdo_future_;
  std::chrono::steady_clock::time_point runtime_diagnostic_sdo_start_time_{};
  bool runtime_diagnostic_sdo_auto_paused_{false};
  std::string runtime_diagnostic_sdo_pause_reason_;
  uint32_t consecutive_runtime_sdo_lock_failures_{0};
  uint64_t runtime_sdo_last_elapsed_us_{0};
  uint64_t sdo_success_{0};
  uint64_t sdo_skipped_{0};
  uint64_t sdo_failed_{0};
  double expected_sensor_rate_hz_{975.0};
  double expected_read_rate_hz_{975.0};
  mutable std::mutex sample_counter_mutex_;
  std::chrono::steady_clock::time_point sample_counter_window_start_{};
  uint64_t repeated_reads_window_{0};
  uint64_t skipped_samples_window_{0};
  uint64_t jump_events_window_{0};
  uint32_t max_delta_window_{0};
  uint32_t consecutive_repeats_{0};
  uint32_t max_consecutive_repeats_window_{0};

  geometry_msgs::msg::WrenchStamped measurement_;
  Axia80EtherCATState ethercat_state_;
  uint32_t status_code_raw_{0};
  uint32_t sample_counter_raw_{0};
  std::optional<uint32_t> previous_status_code_;
  std::optional<uint32_t> previous_sample_counter_;
  double timestamp_sec_{0.0};
  double timestamp_nanosec_{0.0};
  double status_code_{0.0};
  double sample_counter_{0.0};
};

}  // namespace ati_axia80_m20_ethercat_sensor

#endif  // ATI_AXIA80_M20_ETHERCAT_SENSOR__AXIA80_M20_ETHERCAT_SENSOR_HPP_
