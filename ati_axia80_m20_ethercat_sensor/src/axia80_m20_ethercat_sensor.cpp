#include "ati_axia80_m20_ethercat_sensor/axia80_m20_ethercat_sensor.hpp"

#include <chrono>
#include <limits>
#include <stdexcept>
#include <string>

#include "pluginlib/class_list_macros.hpp"

namespace ati_axia80_m20_ethercat_sensor
{
namespace
{
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using namespace std::chrono_literals;

std::string bool_text(bool value)
{
  return value ? "true" : "false";
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
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger(LOGGER_NAME), *get_node()->get_clock(), 5000,
      "Driver is not configured");
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
    const auto ethercat_state = driver_->state_snapshot();
    {
      std::lock_guard<std::mutex> snapshot_lock(realtime_snapshot_mutex_);
      realtime_snapshot_ = {true, status_code_raw_, sample_counter_raw_, ethercat_state};
    }
    handle_status_code_(status_code_raw_);
    check_sample_counter_(sample_counter_raw_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger(LOGGER_NAME), *get_node()->get_clock(), 5000,
      "EtherCAT read failed: %s", e.what());
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

  const auto parsed = parse_hardware_parameters(hardware_info.sensors[0].parameters);
  parameters_ = parsed.driver;
  runtime_diagnostic_sdo_enabled_ = parsed.runtime_diagnostic_sdo_enabled;
  runtime_diagnostic_sdo_timeout_ = parsed.runtime_diagnostic_sdo_timeout;
  expected_sensor_rate_hz_ = parsed.expected_sensor_rate_hz;
  expected_read_rate_hz_ = parsed.expected_read_rate_hz;
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
  {
    std::lock_guard<std::mutex> lock(realtime_snapshot_mutex_);
    realtime_snapshot_ = {};
  }
  {
    std::lock_guard<std::mutex> lock(sample_counter_mutex_);
    sample_counter_window_start_ = std::chrono::steady_clock::now();
    sample_counter_state_ = {};
  }
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
      const auto result = execute_bias(driver_.get(), true);
      response->success = result.success;
      response->message = result.message;
    });

  clear_bias_service_ = node->create_service<std_srvs::srv::Trigger>(
    base_service_name + "/clear_bias",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      std::lock_guard<std::mutex> lock(driver_mutex_);
      const auto result = execute_bias(driver_.get(), false);
      response->success = result.success;
      response->message = result.message;
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

  RealtimeDiagnosticSnapshot snapshot;
  {
    std::lock_guard<std::mutex> lock(realtime_snapshot_mutex_);
    snapshot = realtime_snapshot_;
  }
  const auto sample_diagnostics = sample_counter_diagnostics_();

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();

  DiagnosticStatus communication;
  communication.name = sensor_name_() + ": Axia80 EtherCAT communication";
  communication.hardware_id = sensor_name_();
  const auto & ethercat = snapshot.ethercat;
  const bool communication_ok =
    snapshot.driver_active && ethercat.have_domain_state && ethercat.have_master_state &&
    ethercat.have_slave_state && ethercat.master_link_up &&
    ethercat.master_slaves_responding > 0 && ethercat.slave_online &&
    ethercat.slave_operational;
  const auto initial_levels = make_independent_diagnostic_levels(
    snapshot.driver_active, communication_ok, sample_diagnostics.level,
    DiagnosticStatus::STALE, snapshot.status_code);
  communication.level = initial_levels.communication;
  communication.message = snapshot.driver_active ?
    (communication_ok ? "EtherCAT communication OK" : "EtherCAT communication fault") :
    "driver is not active";
  communication.values.push_back(
    diagnostic_value("ethercat_domain_state_seen", bool_text(ethercat.have_domain_state)));
  communication.values.push_back(diagnostic_value(
      "ethercat_domain_working_counter", std::to_string(ethercat.domain_working_counter)));
  communication.values.push_back(
    diagnostic_value("ethercat_domain_wc_state", std::to_string(ethercat.domain_wc_state)));
  communication.values.push_back(
    diagnostic_value("ethercat_master_state_seen", bool_text(ethercat.have_master_state)));
  communication.values.push_back(diagnostic_value(
      "ethercat_master_slaves_responding", std::to_string(ethercat.master_slaves_responding)));
  communication.values.push_back(
    diagnostic_value("ethercat_master_al_states", std::to_string(ethercat.master_al_states)));
  communication.values.push_back(
    diagnostic_value("ethercat_master_link_up", bool_text(ethercat.master_link_up)));
  communication.values.push_back(
    diagnostic_value("ethercat_slave_state_seen", bool_text(ethercat.have_slave_state)));
  communication.values.push_back(
    diagnostic_value("ethercat_slave_online", bool_text(ethercat.slave_online)));
  communication.values.push_back(
    diagnostic_value("ethercat_slave_operational", bool_text(ethercat.slave_operational)));
  communication.values.push_back(
    diagnostic_value("ethercat_slave_al_state", std::to_string(ethercat.slave_al_state)));

  DiagnosticStatus sample_counter;
  sample_counter.name = sensor_name_() + ": Axia80 sample counter";
  sample_counter.hardware_id = sensor_name_();
  sample_counter.level = initial_levels.sample_counter;
  sample_counter.message = snapshot.driver_active ?
    sample_diagnostics.message : "driver is not active";
  sample_counter.values.push_back(
    diagnostic_value("sample_counter", std::to_string(snapshot.sample_counter)));
  sample_counter.values.push_back(diagnostic_value(
      "expected_sensor_rate_hz", std::to_string(sample_diagnostics.expected_sensor_rate_hz)));
  sample_counter.values.push_back(diagnostic_value(
      "expected_read_rate_hz", std::to_string(sample_diagnostics.expected_read_rate_hz)));
  sample_counter.values.push_back(diagnostic_value(
      "sample_counter_window_sec", std::to_string(sample_diagnostics.elapsed_sec)));
  sample_counter.values.push_back(diagnostic_value(
      "expected_repeats_per_sec", std::to_string(sample_diagnostics.expected_repeats_per_sec)));
  sample_counter.values.push_back(diagnostic_value(
      "actual_repeats_per_sec", std::to_string(sample_diagnostics.actual_repeats_per_sec)));
  sample_counter.values.push_back(
    diagnostic_value("repeat_rate", std::to_string(sample_diagnostics.actual_repeats_per_sec)));
  sample_counter.values.push_back(diagnostic_value(
      "expected_skipped_samples_per_sec",
      std::to_string(sample_diagnostics.expected_skipped_samples_per_sec)));
  sample_counter.values.push_back(diagnostic_value(
      "expected_jump_events_per_sec",
      std::to_string(sample_diagnostics.expected_jump_events_per_sec)));
  sample_counter.values.push_back(diagnostic_value(
      "actual_skipped_samples_per_sec",
      std::to_string(sample_diagnostics.actual_skipped_samples_per_sec)));
  sample_counter.values.push_back(diagnostic_value(
      "actual_jump_events_per_sec",
      std::to_string(sample_diagnostics.actual_jump_events_per_sec)));
  sample_counter.values.push_back(
    diagnostic_value("repeated_reads", std::to_string(sample_diagnostics.repeated_reads)));
  sample_counter.values.push_back(
    diagnostic_value("skipped_samples", std::to_string(sample_diagnostics.skipped_samples)));
  sample_counter.values.push_back(
    diagnostic_value("jump_events", std::to_string(sample_diagnostics.jump_events)));
  sample_counter.values.push_back(
    diagnostic_value("max_delta", std::to_string(sample_diagnostics.max_delta)));
  sample_counter.values.push_back(diagnostic_value(
      "large_jump_threshold", std::to_string(sample_diagnostics.large_jump_threshold)));
  sample_counter.values.push_back(diagnostic_value(
      "consecutive_repeats", std::to_string(sample_diagnostics.consecutive_repeats)));
  sample_counter.values.push_back(diagnostic_value(
      "max_consecutive_repeats", std::to_string(sample_diagnostics.max_consecutive_repeats)));
  sample_counter.values.push_back(
    diagnostic_value("sample_counter_status", sample_diagnostics.message));

  DiagnosticStatus sensor_status;
  sensor_status.name = sensor_name_() + ": Axia80 sensor status code";
  sensor_status.hardware_id = sensor_name_();
  sensor_status.level = initial_levels.sensor_status;
  sensor_status.message = snapshot.driver_active ?
    describe_status_bits(snapshot.status_code) : "driver is not active";
  sensor_status.values.push_back(
    diagnostic_value("status_code", hex_u32(snapshot.status_code)));
  sensor_status.values.push_back(
    diagnostic_value("status_bits", describe_status_bits(snapshot.status_code)));

  DiagnosticStatus runtime_sdo;
  runtime_sdo.name = sensor_name_() + ": Axia80 runtime SDO diagnostics";
  runtime_sdo.hardware_id = sensor_name_();
  runtime_sdo.level = initial_levels.runtime_sdo;
  runtime_sdo.message = snapshot.driver_active ?
    "runtime diagnostic SDO has no result yet" : "driver is not active";

  std::optional<RuntimeDiagnosticSdoResult> result;
  if (runtime_diagnostic_sdo_future_.valid()) {
    const auto future_status = runtime_diagnostic_sdo_future_.wait_for(0ms);
    if (future_status != std::future_status::ready) {
      ++sdo_skipped_;
      const auto elapsed = std::chrono::steady_clock::now() - runtime_diagnostic_sdo_start_time_;
      if (!runtime_diagnostic_sdo_auto_paused_ && elapsed > runtime_diagnostic_sdo_timeout_) {
        runtime_diagnostic_sdo_auto_paused_ = true;
        runtime_diagnostic_sdo_pause_reason_ =
          "runtime diagnostic SDO still running after " +
          std::to_string(runtime_diagnostic_sdo_timeout_.count()) + " ms";
        RCLCPP_WARN(
          rclcpp::get_logger(LOGGER_NAME),
          "Runtime diagnostic SDO paused: %s",
          runtime_diagnostic_sdo_pause_reason_.c_str());
      }
      runtime_sdo.level = DiagnosticStatus::STALE;
      runtime_sdo.message = runtime_diagnostic_sdo_auto_paused_ ?
        "runtime diagnostic SDO auto-paused: " + runtime_diagnostic_sdo_pause_reason_ :
        "runtime diagnostic SDO still running";
    } else {
      try {
        result = runtime_diagnostic_sdo_future_.get();
      } catch (const std::exception & e) {
        result = RuntimeDiagnosticSdoResult{
          RuntimeDiagnosticSdoResult::Outcome::FAILED,
          {},
          std::string("runtime diagnostic SDO worker failed: ") + e.what(),
          0};
      }
    }
  }

  if (result) {
    runtime_sdo_last_elapsed_us_ = result->elapsed_us;
    if (result->elapsed_us > 1000) {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger(LOGGER_NAME),
        *get_node()->get_clock(),
        1000,
        "Runtime diagnostic SDO took %llu us",
        static_cast<unsigned long long>(result->elapsed_us));
    }
    if (result->elapsed_us >
      static_cast<uint64_t>(runtime_diagnostic_sdo_timeout_.count()) * 1000u)
    {
      runtime_diagnostic_sdo_auto_paused_ = true;
      runtime_diagnostic_sdo_pause_reason_ =
        "runtime diagnostic SDO exceeded " +
        std::to_string(runtime_diagnostic_sdo_timeout_.count()) + " ms";
      RCLCPP_WARN(
        rclcpp::get_logger(LOGGER_NAME),
        "Runtime diagnostic SDO paused: %s",
        runtime_diagnostic_sdo_pause_reason_.c_str());
    }

    if (result->outcome == RuntimeDiagnosticSdoResult::Outcome::SUCCESS) {
      ++sdo_success_;
      consecutive_runtime_sdo_lock_failures_ = 0;

      const auto & readings = result->readings;
      const bool diagnostic_error =
        !readings.status_message.empty() && readings.status_message != "No status code errors";

      runtime_sdo.level = diagnostic_error ?
        DiagnosticStatus::ERROR : DiagnosticStatus::OK;
      runtime_sdo.message = readings.status_message.empty() ? "No status code errors" :
        readings.status_message;
      runtime_sdo.values.push_back(
        diagnostic_value("supply_voltage_v", std::to_string(readings.supply_voltage_v)));
      runtime_sdo.values.push_back(
        diagnostic_value("gage_temperature_c", std::to_string(readings.gage_temperature_c)));
      runtime_sdo.values.push_back(
        diagnostic_value("diagnostic_status_message", runtime_sdo.message));
    } else if (result->outcome == RuntimeDiagnosticSdoResult::Outcome::SKIPPED) {
      ++sdo_skipped_;
      if (result->message == "diagnostic SDO skipped because EtherCAT driver is busy") {
        ++consecutive_runtime_sdo_lock_failures_;
        if (consecutive_runtime_sdo_lock_failures_ >= 5) {
          runtime_diagnostic_sdo_auto_paused_ = true;
          runtime_diagnostic_sdo_pause_reason_ =
            "runtime diagnostic SDO failed to acquire driver lock 5 consecutive times";
          RCLCPP_WARN(
            rclcpp::get_logger(LOGGER_NAME),
            "Runtime diagnostic SDO paused: %s",
            runtime_diagnostic_sdo_pause_reason_.c_str());
        }
      } else {
        consecutive_runtime_sdo_lock_failures_ = 0;
      }
      runtime_sdo.level = DiagnosticStatus::STALE;
      runtime_sdo.message = result->message;
    } else {
      ++sdo_failed_;
      consecutive_runtime_sdo_lock_failures_ = 0;
      runtime_sdo.level = DiagnosticStatus::WARN;
      runtime_sdo.message = result->message;
    }
  }

  if (!runtime_diagnostic_sdo_enabled_) {
    ++sdo_skipped_;
    runtime_sdo.level = DiagnosticStatus::STALE;
    runtime_sdo.message = "runtime diagnostic SDO disabled";
  } else if (runtime_diagnostic_sdo_auto_paused_) {
    ++sdo_skipped_;
    runtime_sdo.level = DiagnosticStatus::WARN;
    runtime_sdo.message = "runtime diagnostic SDO auto-paused: " +
      runtime_diagnostic_sdo_pause_reason_;
  } else if (!runtime_diagnostic_sdo_future_.valid()) {
    runtime_diagnostic_sdo_future_ =
      std::async(std::launch::async, [this]() { return read_runtime_diagnostic_sdo_(); });
    runtime_diagnostic_sdo_start_time_ = std::chrono::steady_clock::now();
    if (!result) {
      runtime_sdo.level = DiagnosticStatus::STALE;
      runtime_sdo.message = "runtime diagnostic SDO started";
    }
  }

  runtime_sdo.values.push_back(diagnostic_value("sdo_success", std::to_string(sdo_success_)));
  runtime_sdo.values.push_back(diagnostic_value("sdo_skipped", std::to_string(sdo_skipped_)));
  runtime_sdo.values.push_back(diagnostic_value("sdo_failed", std::to_string(sdo_failed_)));
  runtime_sdo.values.push_back(diagnostic_value(
      "runtime_sdo_last_elapsed_us", std::to_string(runtime_sdo_last_elapsed_us_)));
  runtime_sdo.values.push_back(diagnostic_value(
      "runtime_sdo_consecutive_lock_failures",
      std::to_string(consecutive_runtime_sdo_lock_failures_)));
  runtime_sdo.values.push_back(diagnostic_value(
      "runtime_sdo_auto_paused", bool_text(runtime_diagnostic_sdo_auto_paused_)));
  runtime_sdo.values.push_back(
    diagnostic_value("runtime_sdo_pause_reason", runtime_diagnostic_sdo_pause_reason_));

  array.status = {communication, sample_counter, runtime_sdo, sensor_status};
  diagnostics_publisher_->publish(array);
}

RuntimeDiagnosticSdoResult Axia80M20EtherCATSensor::read_runtime_diagnostic_sdo_()
{
  const auto start = std::chrono::steady_clock::now();
  const auto elapsed_us = [&]() {
    return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - start).count());
  };

  std::unique_lock<std::mutex> lock(driver_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    return {
      RuntimeDiagnosticSdoResult::Outcome::SKIPPED,
      {},
      "diagnostic SDO skipped because EtherCAT driver is busy",
      elapsed_us()};
  }

  if (!driver_ready_()) {
    return {
      RuntimeDiagnosticSdoResult::Outcome::SKIPPED,
      {},
      "driver is not active",
      elapsed_us()};
  }

  try {
    const auto readings = driver_->read_diagnostic_readings();
    return {
      RuntimeDiagnosticSdoResult::Outcome::SUCCESS,
      readings,
      {},
      elapsed_us()};
  } catch (const std::exception & e) {
    return {
      RuntimeDiagnosticSdoResult::Outcome::FAILED,
      {},
      std::string("failed to read 0x2080 diagnostic SDO: ") + e.what(),
      elapsed_us()};
  }
}

void Axia80M20EtherCATSensor::handle_status_code_(uint32_t status_code)
{
  if (previous_status_code_ && *previous_status_code_ == status_code) {
    return;
  }

  const auto description = describe_status_bits(status_code);
  if (status_code == 0) {
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger(LOGGER_NAME), *get_node()->get_clock(), 1000,
      "Axia status_code changed to %s: %s",
      hex_u32(status_code).c_str(),
      description.c_str());
  } else {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger(LOGGER_NAME), *get_node()->get_clock(), 1000,
      "Axia status_code changed to %s: %s",
      hex_u32(status_code).c_str(),
      description.c_str());
  }
  previous_status_code_ = status_code;
}

void Axia80M20EtherCATSensor::check_sample_counter_(uint32_t sample_counter)
{
  std::lock_guard<std::mutex> lock(sample_counter_mutex_);
  observe_sample_counter(sample_counter_state_, sample_counter);
}

SampleCounterDiagnostics Axia80M20EtherCATSensor::sample_counter_diagnostics_()
{
  std::lock_guard<std::mutex> lock(sample_counter_mutex_);
  const auto now = std::chrono::steady_clock::now();
  const double elapsed_sec =
    std::chrono::duration<double>(now - sample_counter_window_start_).count();
  sample_counter_window_start_ = now;
  return take_sample_counter_diagnostics(
    sample_counter_state_, elapsed_sec, expected_sensor_rate_hz_, expected_read_rate_hz_);
}

bool Axia80M20EtherCATSensor::driver_ready_() const
{
  return driver_ && driver_->is_active();
}

}  // namespace ati_axia80_m20_ethercat_sensor

PLUGINLIB_EXPORT_CLASS(
  ati_axia80_m20_ethercat_sensor::Axia80M20EtherCATSensor,
  hardware_interface::SensorInterface)
