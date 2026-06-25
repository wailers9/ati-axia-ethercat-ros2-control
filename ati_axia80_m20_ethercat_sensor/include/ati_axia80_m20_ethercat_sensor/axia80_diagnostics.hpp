#ifndef ATI_AXIA80_M20_ETHERCAT_SENSOR__AXIA80_DIAGNOSTICS_HPP_
#define ATI_AXIA80_M20_ETHERCAT_SENSOR__AXIA80_DIAGNOSTICS_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>

#include "ati_axia80_m20_ethercat_sensor/axia80_ethercat_driver.hpp"

namespace ati_axia80_m20_ethercat_sensor
{

using HardwareParameters = std::unordered_map<std::string, std::string>;

struct ParsedHardwareParameters
{
  Axia80DriverParameters driver;
  bool runtime_diagnostic_sdo_enabled{true};
  std::chrono::milliseconds runtime_diagnostic_sdo_timeout{5};
  double expected_sensor_rate_hz{975.0};
  double expected_read_rate_hz{975.0};
};

struct SampleCounterState
{
  std::optional<uint32_t> previous;
  uint64_t repeated_reads{0};
  uint64_t skipped_samples{0};
  uint64_t jump_events{0};
  uint32_t max_delta{0};
  uint32_t consecutive_repeats{0};
  uint32_t max_consecutive_repeats{0};
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

struct BiasResult
{
  bool success{false};
  std::string message;
};

struct IndependentDiagnosticLevels
{
  uint8_t communication{0};
  uint8_t sample_counter{0};
  uint8_t runtime_sdo{0};
  uint8_t sensor_status{0};
};

bool parse_bool_parameter(const std::string & value, const std::string & key);
ParsedHardwareParameters parse_hardware_parameters(const HardwareParameters & params);
std::string hex_u32(uint32_t value);
std::string describe_status_bits(uint32_t status_code);
uint8_t status_code_level(uint32_t status_code);
IndependentDiagnosticLevels make_independent_diagnostic_levels(
  bool driver_active, bool communication_ok, uint8_t sample_counter_level,
  uint8_t runtime_sdo_level, uint32_t status_code);
void observe_sample_counter(SampleCounterState & state, uint32_t sample_counter);
SampleCounterDiagnostics take_sample_counter_diagnostics(
  SampleCounterState & state, double elapsed_sec, double expected_sensor_rate_hz,
  double expected_read_rate_hz);
BiasResult execute_bias(Axia80DriverInterface * driver, bool set_bias);

}  // namespace ati_axia80_m20_ethercat_sensor

#endif  // ATI_AXIA80_M20_ETHERCAT_SENSOR__AXIA80_DIAGNOSTICS_HPP_
