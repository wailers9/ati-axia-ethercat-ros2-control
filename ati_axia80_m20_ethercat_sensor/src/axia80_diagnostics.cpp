#include "ati_axia80_m20_ethercat_sensor/axia80_diagnostics.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "ati_axia80_m20_ethercat_sensor/axia80_ethercat_config.hpp"

namespace ati_axia80_m20_ethercat_sensor
{
namespace
{
using diagnostic_msgs::msg::DiagnosticStatus;

struct StatusBitDescription
{
  uint8_t bit;
  const char * name;
};

constexpr StatusBitDescription STATUS_BITS[] = {
  {0, "internal temperature out of range"},
  {1, "supply voltage out of range"},
  {2, "broken gage"},
  {3, "busy"},
  {5, "hardware or stack error"},
  {26, "gage out-of-range warning"},
  {27, "gage out of range"},
  {28, "simulated error"},
  {29, "calibration checksum error"},
  {30, "sensing range exceeded"},
  {31, "error summary"},
};

const std::string & required_param(const HardwareParameters & params, const std::string & key)
{
  const auto it = params.find(key);
  if (it == params.end() || it->second.empty()) {
    throw std::runtime_error("missing required hardware parameter '" + key + "'");
  }
  return it->second;
}

std::string optional_param(
  const HardwareParameters & params, const std::string & key, const std::string & default_value)
{
  const auto it = params.find(key);
  return it == params.end() ? default_value : it->second;
}

uint8_t parse_u8(const std::string & value, const std::string & key, uint8_t max_value)
{
  size_t consumed = 0;
  const auto parsed = std::stoul(value, &consumed, 0);
  if (consumed != value.size() || parsed > max_value) {
    throw std::runtime_error("'" + key + "' must be an integer in [0, " +
      std::to_string(max_value) + "]");
  }
  return static_cast<uint8_t>(parsed);
}

double parse_positive_double(const std::string & value, const std::string & key)
{
  size_t consumed = 0;
  const double parsed = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(parsed) || parsed <= 0.0) {
    throw std::runtime_error("'" + key + "' must be a finite positive value");
  }
  return parsed;
}

uint32_t parse_u32(const std::string & value, const std::string & key)
{
  size_t consumed = 0;
  const auto parsed = std::stoull(value, &consumed, 0);
  if (consumed != value.size() || parsed > UINT32_MAX) {
    throw std::runtime_error("'" + key + "' must be a valid uint32 value");
  }
  return static_cast<uint32_t>(parsed);
}

uint16_t parse_u16(const std::string & value, const std::string & key)
{
  const auto parsed = parse_u32(value, key);
  if (parsed > UINT16_MAX) {
    throw std::runtime_error("'" + key + "' must be a valid uint16 value");
  }
  return static_cast<uint16_t>(parsed);
}

double sample_rate_hz(uint8_t sample_rate_code)
{
  constexpr double SAMPLE_RATES_HZ[] = {487.0, 975.0, 1990.0, 3900.0};
  return SAMPLE_RATES_HZ[sample_rate_code];
}
}  // namespace

bool parse_bool_parameter(const std::string & value, const std::string & key)
{
  if (value == "true" || value == "True" || value == "1" ||
    value == "yes" || value == "Yes")
  {
    return true;
  }
  if (value == "false" || value == "False" || value == "0" ||
    value == "no" || value == "No")
  {
    return false;
  }
  throw std::runtime_error("'" + key + "' must be a boolean value");
}

ParsedHardwareParameters parse_hardware_parameters(const HardwareParameters & params)
{
  ParsedHardwareParameters result;
  result.driver.master_index = parse_u32(optional_param(params, "master_index", "0"), "master_index");
  result.driver.slave.alias = parse_u16(optional_param(params, "slave_alias", "0"), "slave_alias");
  result.driver.slave.position = parse_u16(required_param(params, "slave_position"), "slave_position");
  result.driver.slave.vendor_id = parse_u32(
    optional_param(params, "vendor_id", std::to_string(ATI_VENDOR_ID)), "vendor_id");
  result.driver.slave.product_code = parse_u32(
    optional_param(params, "product_code", std::to_string(AXIA_PRODUCT_CODE)), "product_code");
  result.driver.slave.revision = parse_u32(
    optional_param(params, "revision", std::to_string(AXIA_REVISION)), "revision");

  result.driver.read_calibration_sdo = parse_bool_parameter(
    optional_param(params, "read_calibration_sdo", "true"), "read_calibration_sdo");
  result.runtime_diagnostic_sdo_enabled = parse_bool_parameter(
    optional_param(params, "runtime_diagnostic_sdo", "true"), "runtime_diagnostic_sdo");
  result.runtime_diagnostic_sdo_timeout = std::chrono::milliseconds(
    parse_u32(
      optional_param(params, "runtime_diagnostic_sdo_timeout_ms", "5"),
      "runtime_diagnostic_sdo_timeout_ms"));
  if (result.runtime_diagnostic_sdo_timeout.count() == 0) {
    throw std::runtime_error("'runtime_diagnostic_sdo_timeout_ms' must be > 0");
  }

  result.driver.counts_per_force = parse_positive_double(
    optional_param(params, "counts_per_force", "1000000"), "counts_per_force");
  result.driver.counts_per_torque = parse_positive_double(
    optional_param(params, "counts_per_torque", "1000000"), "counts_per_torque");
  result.driver.filter_selection = parse_u8(
    optional_param(params, "filter_selection", "0"), "filter_selection", 8);
  result.driver.calibration_slot = parse_u8(
    optional_param(params, "calibration_slot", "0"), "calibration_slot", 1);
  result.driver.sample_rate_code = parse_u8(
    optional_param(params, "sample_rate_code", "1"), "sample_rate_code", 3);

  result.expected_sensor_rate_hz = parse_positive_double(
    optional_param(
      params, "expected_sensor_rate_hz",
      std::to_string(sample_rate_hz(result.driver.sample_rate_code))),
    "expected_sensor_rate_hz");
  result.expected_read_rate_hz = parse_positive_double(
    optional_param(
      params, "expected_read_rate_hz", std::to_string(result.expected_sensor_rate_hz)),
    "expected_read_rate_hz");
  result.driver.set_bias_on_activate = parse_bool_parameter(
    optional_param(params, "set_bias_on_activate", "false"), "set_bias_on_activate");
  result.driver.clear_bias_on_activate = parse_bool_parameter(
    optional_param(params, "clear_bias_on_activate", "false"), "clear_bias_on_activate");
  return result;
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
  return first ? "no active status bits" : stream.str();
}

uint8_t status_code_level(uint32_t status_code)
{
  if (status_code == 0) {
    return DiagnosticStatus::OK;
  }
  if ((status_code & (uint32_t{1} << 26)) != 0 &&
    (status_code & ~(uint32_t{1} << 26)) == 0)
  {
    return DiagnosticStatus::WARN;
  }
  return DiagnosticStatus::ERROR;
}

IndependentDiagnosticLevels make_independent_diagnostic_levels(
  bool driver_active, bool communication_ok, uint8_t sample_counter_level,
  uint8_t runtime_sdo_level, uint32_t status_code)
{
  if (!driver_active) {
    return {
      DiagnosticStatus::STALE, DiagnosticStatus::STALE,
      DiagnosticStatus::STALE, DiagnosticStatus::STALE};
  }
  return {
    communication_ok ? DiagnosticStatus::OK : DiagnosticStatus::ERROR,
    sample_counter_level,
    runtime_sdo_level,
    status_code_level(status_code)};
}

void observe_sample_counter(SampleCounterState & state, uint32_t sample_counter)
{
  if (!state.previous) {
    state.previous = sample_counter;
    return;
  }

  const uint32_t delta = sample_counter - *state.previous;
  if (delta == 0) {
    ++state.repeated_reads;
    ++state.consecutive_repeats;
    state.max_consecutive_repeats =
      std::max(state.max_consecutive_repeats, state.consecutive_repeats);
  } else {
    state.consecutive_repeats = 0;
    state.max_delta = std::max(state.max_delta, delta);
    if (delta > 1) {
      ++state.jump_events;
      state.skipped_samples += static_cast<uint64_t>(delta) - 1u;
    }
  }
  state.previous = sample_counter;
}

SampleCounterDiagnostics take_sample_counter_diagnostics(
  SampleCounterState & state, double elapsed_sec, double expected_sensor_rate_hz,
  double expected_read_rate_hz)
{
  SampleCounterDiagnostics result;
  result.elapsed_sec = std::max(0.001, elapsed_sec);
  result.expected_sensor_rate_hz = expected_sensor_rate_hz;
  result.expected_read_rate_hz = expected_read_rate_hz;
  result.expected_repeats_per_sec =
    std::max(0.0, expected_read_rate_hz - expected_sensor_rate_hz);
  result.expected_skipped_samples_per_sec =
    std::max(0.0, expected_sensor_rate_hz - expected_read_rate_hz);
  result.expected_jump_events_per_sec =
    std::min(expected_read_rate_hz, result.expected_skipped_samples_per_sec);
  result.large_jump_threshold = static_cast<uint32_t>(
    std::max(10.0, std::ceil(expected_sensor_rate_hz / expected_read_rate_hz * 5.0)));

  result.repeated_reads = state.repeated_reads;
  result.skipped_samples = state.skipped_samples;
  result.jump_events = state.jump_events;
  result.max_delta = state.max_delta;
  result.consecutive_repeats = state.consecutive_repeats;
  result.max_consecutive_repeats =
    std::max(state.consecutive_repeats, state.max_consecutive_repeats);

  state.repeated_reads = 0;
  state.skipped_samples = 0;
  state.jump_events = 0;
  state.max_delta = 0;
  state.max_consecutive_repeats = state.consecutive_repeats;

  result.actual_repeats_per_sec =
    static_cast<double>(result.repeated_reads) / result.elapsed_sec;
  result.actual_skipped_samples_per_sec =
    static_cast<double>(result.skipped_samples) / result.elapsed_sec;
  result.actual_jump_events_per_sec =
    static_cast<double>(result.jump_events) / result.elapsed_sec;

  const bool error =
    result.max_consecutive_repeats >= 50 ||
    result.actual_repeats_per_sec > result.expected_repeats_per_sec * 2.0 + 100.0 ||
    result.actual_skipped_samples_per_sec >
    result.expected_skipped_samples_per_sec * 2.0 + 50.0 ||
    result.max_delta > static_cast<uint64_t>(result.large_jump_threshold) * 5u;
  const bool warn =
    result.max_consecutive_repeats >= 10 ||
    result.actual_repeats_per_sec > result.expected_repeats_per_sec * 2.0 + 60.0 ||
    result.actual_skipped_samples_per_sec >
    result.expected_skipped_samples_per_sec * 2.0 + 30.0 ||
    result.max_delta > result.large_jump_threshold;

  result.level = error ? DiagnosticStatus::ERROR :
    (warn ? DiagnosticStatus::WARN : DiagnosticStatus::OK);
  std::ostringstream message;
  message << "sample counter " <<
    (error ? "ERROR" : (warn ? "WARN" : "OK")) <<
    ": repeats=" << std::fixed << std::setprecision(1) <<
    result.actual_repeats_per_sec << "/s, skipped=" <<
    result.actual_skipped_samples_per_sec << "/s, jump_events=" <<
    result.actual_jump_events_per_sec << "/s, max_delta=" << result.max_delta <<
    ", consecutive_repeats=" << result.consecutive_repeats;
  result.message = message.str();
  return result;
}

BiasResult execute_bias(Axia80DriverInterface * driver, bool set_bias)
{
  if (!driver || !driver->is_active()) {
    return {false, "ATI Axia80-M20 EtherCAT driver is not active"};
  }
  try {
    if (set_bias) {
      driver->set_bias();
      return {true, "ATI Axia80-M20 bias set"};
    }
    driver->clear_bias();
    return {true, "ATI Axia80-M20 bias cleared"};
  } catch (const std::exception & e) {
    return {
      false, std::string(set_bias ? "Failed to set bias: " : "Failed to clear bias: ") + e.what()};
  }
}

}  // namespace ati_axia80_m20_ethercat_sensor
