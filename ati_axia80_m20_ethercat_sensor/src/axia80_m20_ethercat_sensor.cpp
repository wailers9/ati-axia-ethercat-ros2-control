#include "ati_axia80_m20_ethercat_sensor/axia80_m20_ethercat_sensor.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "pluginlib/class_list_macros.hpp"

namespace ati_axia80_m20_ethercat_sensor
{
namespace
{
using Parameters = std::unordered_map<std::string, std::string>;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using namespace std::chrono_literals;

struct StatusBitDescription
{
  uint8_t bit;
  const char * name;
  bool error;
};

constexpr StatusBitDescription STATUS_BITS[] = {
  {0, "internal temperature out of range", true},
  {1, "supply voltage out of range", true},
  {2, "broken gage", true},
  {3, "busy", true},
  {5, "hardware or stack error", true},
  {26, "gage out-of-range warning", true},
  {27, "gage out of range", true},
  {28, "simulated error", true},
  {29, "calibration checksum error", true},
  {30, "sensing range exceeded", true},
  {31, "error summary", true},
};

const std::string & required_param(const Parameters & params, const std::string & key)
{
  const auto it = params.find(key);
  if (it == params.end()) {
    throw std::runtime_error("missing required hardware parameter '" + key + "'");
  }
  return it->second;
}

std::string optional_param(
  const Parameters & params, const std::string & key, const std::string & default_value)
{
  const auto it = params.find(key);
  return it == params.end() ? default_value : it->second;
}

bool parse_bool(const std::string & value)
{
  return value == "true" || value == "True" || value == "1" || value == "yes" || value == "Yes";
}

uint8_t parse_u8(const std::string & value, const std::string & key, uint8_t max_value)
{
  const auto parsed = std::stoul(value);
  if (parsed > max_value) {
    throw std::runtime_error("'" + key + "' must be <= " + std::to_string(max_value));
  }
  return static_cast<uint8_t>(parsed);
}

std::string hex_u32(uint32_t value)
{
  std::ostringstream stream;
  stream << "0x" << std::hex << std::setw(8) << std::setfill('0') << value;
  return stream.str();
}

std::string describe_status_bits(uint32_t status_code)
{
  std::ostringstream stream;
  bool first = true;
  for (const auto & bit : STATUS_BITS) {
    if ((status_code & (uint32_t{1} << bit.bit)) == 0) {
      continue;
    }
    if (!first) {
      stream << "; ";
    }
    stream << "bit " << static_cast<unsigned int>(bit.bit) << " " << bit.name;
    first = false;
  }
  if (first) {
    return "no active status bits";
  }
  return stream.str();
}

KeyValue diagnostic_value(const std::string & key, const std::string & value)
{
  KeyValue key_value;
  key_value.key = key;
  key_value.value = value;
  return key_value;
}
}  // namespace

hardware_interface::CallbackReturn Axia80M20EtherCATSensor::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  const auto ret = hardware_interface::SensorInterface::on_init(params);
  if (ret != hardware_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  try {
    parse_parameters_(params.hardware_info);
    reset_state_();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Parameter parsing failed: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Axia80M20EtherCATSensor::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    std::lock_guard<std::mutex> lock(driver_mutex_);
    driver_ = std::make_unique<Axia80EtherCATDriver>(parameters_);
    create_bias_services_();
    create_diagnostics_publisher_();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Driver configuration failed: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Axia80M20EtherCATSensor::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!driver_) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Driver is not configured");
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    std::lock_guard<std::mutex> lock(driver_mutex_);
    driver_->init();
    if (parameters_.clear_bias_on_activate) {
      driver_->clear_bias();
    }
    if (parameters_.set_bias_on_activate) {
      driver_->set_bias();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "EtherCAT activation failed: %s", e.what());
    if (driver_) {
      driver_->shutdown();
    }
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "ATI Axia80-M20 EtherCAT sensor activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Axia80M20EtherCATSensor::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::lock_guard<std::mutex> lock(driver_mutex_);
  if (driver_) {
    driver_->shutdown();
  }
  reset_state_();
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "ATI Axia80-M20 EtherCAT sensor deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
Axia80M20EtherCATSensor::export_state_interfaces()
{
  const auto name = sensor_name_();
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(name, "timestamp.sec", &timestamp_sec_);
  state_interfaces.emplace_back(name, "timestamp.nanosec", &timestamp_nanosec_);
  state_interfaces.emplace_back(name, "force.x", &measurement_.wrench.force.x);
  state_interfaces.emplace_back(name, "force.y", &measurement_.wrench.force.y);
  state_interfaces.emplace_back(name, "force.z", &measurement_.wrench.force.z);
  state_interfaces.emplace_back(name, "torque.x", &measurement_.wrench.torque.x);
  state_interfaces.emplace_back(name, "torque.y", &measurement_.wrench.torque.y);
  state_interfaces.emplace_back(name, "torque.z", &measurement_.wrench.torque.z);
  state_interfaces.emplace_back(name, "status_code", &status_code_);
  state_interfaces.emplace_back(name, "sample_counter", &sample_counter_);
  return state_interfaces;
}

hardware_interface::return_type Axia80M20EtherCATSensor::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!driver_) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Driver is not configured");
    return hardware_interface::return_type::ERROR;
  }

  try {
    std::lock_guard<std::mutex> lock(driver_mutex_);
    const auto sample = driver_->read_once();
    if (!sample.valid) {
      return hardware_interface::return_type::OK;
    }

    measurement_ = sample.wrench;
    timestamp_sec_ = static_cast<double>(measurement_.header.stamp.sec);
    timestamp_nanosec_ = static_cast<double>(measurement_.header.stamp.nanosec);
    status_code_raw_ = sample.status_code;
    sample_counter_raw_ = sample.sample_counter;
    status_code_ = static_cast<double>(status_code_raw_);
    sample_counter_ = static_cast<double>(sample_counter_raw_);
    handle_status_code_(status_code_raw_);
    check_sample_counter_(sample_counter_raw_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "EtherCAT read failed: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

void Axia80M20EtherCATSensor::parse_parameters_(
  const hardware_interface::HardwareInfo & hardware_info)
{
  if (hardware_info.sensors.empty()) {
    throw std::runtime_error("hardware description must contain one <sensor> entry");
  }

  const auto & params = hardware_info.sensors[0].parameters;
  parameters_.master_index = static_cast<unsigned int>(
    std::stoul(optional_param(params, "master_index", "0")));
  parameters_.slave.alias = static_cast<uint16_t>(
    std::stoul(optional_param(params, "slave_alias", "0")));
  parameters_.slave.position = static_cast<uint16_t>(
    std::stoul(required_param(params, "slave_position")));
  parameters_.slave.vendor_id = static_cast<uint32_t>(
    std::stoul(optional_param(params, "vendor_id", std::to_string(ATI_VENDOR_ID)), nullptr, 0));
  parameters_.slave.product_code = static_cast<uint32_t>(
    std::stoul(optional_param(params, "product_code", std::to_string(AXIA_PRODUCT_CODE)), nullptr, 0));
  parameters_.slave.revision = static_cast<uint32_t>(
    std::stoul(optional_param(params, "revision", std::to_string(AXIA_REVISION)), nullptr, 0));

  parameters_.read_calibration_sdo =
    parse_bool(optional_param(params, "read_calibration_sdo", "true"));
  parameters_.counts_per_force = std::stod(optional_param(params, "counts_per_force", "1000000"));
  parameters_.counts_per_torque = std::stod(optional_param(params, "counts_per_torque", "1000000"));
  if (!std::isfinite(parameters_.counts_per_force) || parameters_.counts_per_force <= 0.0) {
    throw std::runtime_error("counts_per_force must be a finite positive value");
  }
  if (!std::isfinite(parameters_.counts_per_torque) || parameters_.counts_per_torque <= 0.0) {
    throw std::runtime_error("counts_per_torque must be a finite positive value");
  }

  parameters_.filter_selection =
    parse_u8(optional_param(params, "filter_selection", "0"), "filter_selection", 8);
  parameters_.calibration_slot =
    parse_u8(optional_param(params, "calibration_slot", "0"), "calibration_slot", 1);
  parameters_.sample_rate_code =
    parse_u8(optional_param(params, "sample_rate_code", "0"), "sample_rate_code", 3);
  parameters_.set_bias_on_activate =
    parse_bool(optional_param(params, "set_bias_on_activate", "false"));
  parameters_.clear_bias_on_activate =
    parse_bool(optional_param(params, "clear_bias_on_activate", "true"));
}

void Axia80M20EtherCATSensor::reset_state_()
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  timestamp_sec_ = nan;
  timestamp_nanosec_ = nan;
  measurement_.wrench.force.x = nan;
  measurement_.wrench.force.y = nan;
  measurement_.wrench.force.z = nan;
  measurement_.wrench.torque.x = nan;
  measurement_.wrench.torque.y = nan;
  measurement_.wrench.torque.z = nan;
  status_code_raw_ = 0;
  sample_counter_raw_ = 0;
  previous_status_code_.reset();
  previous_sample_counter_.reset();
  status_code_ = nan;
  sample_counter_ = nan;
}

std::string Axia80M20EtherCATSensor::sensor_name_() const
{
  if (!info_.sensors.empty() && !info_.sensors[0].name.empty()) {
    return info_.sensors[0].name;
  }
  return "ati_axia80_m20";
}

void Axia80M20EtherCATSensor::create_bias_services_()
{
  const auto node = get_node();
  if (!node) {
    throw std::runtime_error("hardware node is not available for bias services");
  }

  const auto base_service_name = "/" + sensor_name_();

  set_bias_service_ = node->create_service<std_srvs::srv::Trigger>(
    base_service_name + "/set_bias",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      std::lock_guard<std::mutex> lock(driver_mutex_);
      if (!driver_ready_()) {
        response->success = false;
        response->message = "ATI Axia80-M20 EtherCAT driver is not active";
        return;
      }

      try {
        driver_->set_bias();
        response->success = true;
        response->message = "ATI Axia80-M20 bias set";
      } catch (const std::exception & e) {
        response->success = false;
        response->message = std::string("Failed to set bias: ") + e.what();
      }
    });

  clear_bias_service_ = node->create_service<std_srvs::srv::Trigger>(
    base_service_name + "/clear_bias",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      std::lock_guard<std::mutex> lock(driver_mutex_);
      if (!driver_ready_()) {
        response->success = false;
        response->message = "ATI Axia80-M20 EtherCAT driver is not active";
        return;
      }

      try {
        driver_->clear_bias();
        response->success = true;
        response->message = "ATI Axia80-M20 bias cleared";
      } catch (const std::exception & e) {
        response->success = false;
        response->message = std::string("Failed to clear bias: ") + e.what();
      }
    });
}

void Axia80M20EtherCATSensor::create_diagnostics_publisher_()
{
  const auto node = get_node();
  if (!node) {
    throw std::runtime_error("hardware node is not available for diagnostics");
  }

  diagnostics_publisher_ =
    node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  diagnostics_timer_ = node->create_wall_timer(1s, [this]() { publish_diagnostics_(); });
}

void Axia80M20EtherCATSensor::publish_diagnostics_()
{
  if (!diagnostics_publisher_) {
    return;
  }

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();

  DiagnosticStatus status;
  status.name = sensor_name_() + ": ATI Axia80-M20";
  status.hardware_id = sensor_name_();
  status.level = DiagnosticStatus::STALE;
  status.message = "driver is not active";
  status.values.push_back(diagnostic_value("status_code", hex_u32(status_code_raw_)));
  status.values.push_back(diagnostic_value("status_bits", describe_status_bits(status_code_raw_)));
  status.values.push_back(diagnostic_value("sample_counter", std::to_string(sample_counter_raw_)));

  std::unique_lock<std::mutex> lock(driver_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    status.level = DiagnosticStatus::STALE;
    status.message = "diagnostic SDO skipped because EtherCAT driver is busy";
    array.status.push_back(status);
    diagnostics_publisher_->publish(array);
    return;
  }

  if (!driver_ready_()) {
    array.status.push_back(status);
    diagnostics_publisher_->publish(array);
    return;
  }

  try {
    const auto readings = driver_->read_diagnostic_readings();
    const bool status_error = (status_code_raw_ & (uint32_t{1} << 31)) != 0;
    const bool diagnostic_error =
      !readings.status_message.empty() && readings.status_message != "No status code errors";

    status.level = (status_error || diagnostic_error) ?
      DiagnosticStatus::ERROR : DiagnosticStatus::OK;
    status.message = readings.status_message.empty() ? "No status code errors" :
      readings.status_message;
    status.values.push_back(
      diagnostic_value("supply_voltage_v", std::to_string(readings.supply_voltage_v)));
    status.values.push_back(
      diagnostic_value("gage_temperature_c", std::to_string(readings.gage_temperature_c)));
    status.values.push_back(
      diagnostic_value("diagnostic_status_message", status.message));
  } catch (const std::exception & e) {
    status.level = DiagnosticStatus::WARN;
    status.message = std::string("failed to read 0x2080 diagnostic SDO: ") + e.what();
  }

  array.status.push_back(status);
  diagnostics_publisher_->publish(array);
}

void Axia80M20EtherCATSensor::handle_status_code_(uint32_t status_code)
{
  if (previous_status_code_ && *previous_status_code_ == status_code) {
    return;
  }

  const auto description = describe_status_bits(status_code);
  if (status_code == 0) {
    RCLCPP_INFO(
      rclcpp::get_logger(LOGGER_NAME),
      "Axia status_code changed to %s: %s",
      hex_u32(status_code).c_str(),
      description.c_str());
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger(LOGGER_NAME),
      "Axia status_code changed to %s: %s",
      hex_u32(status_code).c_str(),
      description.c_str());
  }
  previous_status_code_ = status_code;
}

void Axia80M20EtherCATSensor::check_sample_counter_(uint32_t sample_counter)
{
  if (!previous_sample_counter_) {
    previous_sample_counter_ = sample_counter;
    return;
  }

  const uint32_t expected = *previous_sample_counter_ + 1u;
  if (sample_counter == *previous_sample_counter_) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger(LOGGER_NAME),
      *get_node()->get_clock(),
      1000,
      "Axia sample_counter repeated at %u",
      sample_counter);
  } else if (sample_counter != expected) {
    RCLCPP_WARN(
      rclcpp::get_logger(LOGGER_NAME),
      "Axia sample_counter discontinuity: previous=%u expected=%u actual=%u",
      *previous_sample_counter_,
      expected,
      sample_counter);
  }

  previous_sample_counter_ = sample_counter;
}

bool Axia80M20EtherCATSensor::driver_ready_() const
{
  return driver_ && driver_->is_active();
}

}  // namespace ati_axia80_m20_ethercat_sensor

PLUGINLIB_EXPORT_CLASS(
  ati_axia80_m20_ethercat_sensor::Axia80M20EtherCATSensor,
  hardware_interface::SensorInterface)
