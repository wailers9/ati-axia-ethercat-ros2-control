#include <gtest/gtest.h>

#include <stdexcept>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "ati_axia80_m20_ethercat_sensor/axia80_diagnostics.hpp"

namespace ati_axia80_m20_ethercat_sensor
{
namespace
{
using diagnostic_msgs::msg::DiagnosticStatus;

class MockDriver : public Axia80DriverInterface
{
public:
  void init() override {active = true;}
  void shutdown() override {active = false;}
  void set_bias() override {++set_bias_calls;}
  void clear_bias() override {++clear_bias_calls;}
  bool is_active() const override {return active;}
  Axia80Sample read_once() override {return Axia80Sample();}
  Axia80DiagnosticReadings read_diagnostic_readings() const override {return {};}
  Axia80EtherCATState state_snapshot() const override {return {};}

  bool active{false};
  int set_bias_calls{0};
  int clear_bias_calls{0};
};

TEST(ParameterParsing, RequiresSlavePosition)
{
  EXPECT_THROW(parse_hardware_parameters({}), std::runtime_error);
}

TEST(ParameterParsing, RejectsOutOfRangeSampleRate)
{
  EXPECT_THROW(
    parse_hardware_parameters({
      {"slave_position", "0"},
      {"sample_rate_code", "4"},
    }),
    std::runtime_error);
}

TEST(ParameterParsing, RejectsInvalidForceScale)
{
  for (const auto * value : {"0", "-1", "nan", "not-a-number"}) {
    EXPECT_THROW(
      parse_hardware_parameters({
        {"slave_position", "0"},
        {"counts_per_force", value},
      }),
      std::exception);
  }
}

TEST(ParameterParsing, ParsesValidHardwareConfigurationWithoutHardware)
{
  const auto parsed = parse_hardware_parameters({
    {"slave_position", "2"},
    {"sample_rate_code", "2"},
    {"runtime_diagnostic_sdo", "false"},
  });
  EXPECT_EQ(parsed.driver.slave.position, 2u);
  EXPECT_EQ(parsed.driver.sample_rate_code, 2u);
  EXPECT_DOUBLE_EQ(parsed.expected_sensor_rate_hz, 1990.0);
  EXPECT_FALSE(parsed.runtime_diagnostic_sdo_enabled);
  EXPECT_FALSE(parsed.driver.clear_bias_on_activate);
  EXPECT_FALSE(parsed.driver.set_bias_on_activate);
}

TEST(SampleCounter, CountsRepeatsJumpsAndSkippedSamples)
{
  SampleCounterState state;
  for (const uint32_t value : {100u, 100u, 101u, 103u}) {
    observe_sample_counter(state, value);
  }
  const auto result = take_sample_counter_diagnostics(state, 1.0, 975.0, 975.0);
  EXPECT_EQ(result.repeated_reads, 1u);
  EXPECT_EQ(result.jump_events, 1u);
  EXPECT_EQ(result.skipped_samples, 1u);
  EXPECT_EQ(result.max_delta, 2u);
}

TEST(SampleCounter, HandlesUint32Rollover)
{
  SampleCounterState state;
  observe_sample_counter(state, UINT32_MAX);
  observe_sample_counter(state, 0u);
  const auto result = take_sample_counter_diagnostics(state, 1.0, 975.0, 975.0);
  EXPECT_EQ(result.jump_events, 0u);
  EXPECT_EQ(result.skipped_samples, 0u);
  EXPECT_EQ(result.max_delta, 1u);
}

TEST(DiagnosticLevels, RuntimeSdoStaleDoesNotMaskSampleCounterError)
{
  const auto levels = make_independent_diagnostic_levels(
    true, true, DiagnosticStatus::ERROR, DiagnosticStatus::STALE, 0u);
  EXPECT_EQ(levels.runtime_sdo, DiagnosticStatus::STALE);
  EXPECT_EQ(levels.sample_counter, DiagnosticStatus::ERROR);
}

TEST(BiasService, RejectsBiasWhenDriverIsInactive)
{
  MockDriver driver;
  const auto result = execute_bias(&driver, true);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(driver.set_bias_calls, 0);
}

TEST(BiasService, CallsMockDriverWhenActive)
{
  MockDriver driver;
  driver.active = true;
  EXPECT_TRUE(execute_bias(&driver, true).success);
  EXPECT_TRUE(execute_bias(&driver, false).success);
  EXPECT_EQ(driver.set_bias_calls, 1);
  EXPECT_EQ(driver.clear_bias_calls, 1);
}

TEST(PureFunctions, ParsesBooleanHexAndStatusBits)
{
  EXPECT_TRUE(parse_bool_parameter("yes", "flag"));
  EXPECT_FALSE(parse_bool_parameter("False", "flag"));
  EXPECT_THROW(parse_bool_parameter("enabled", "flag"), std::runtime_error);
  EXPECT_EQ(hex_u32(0x1au), "0x0000001a");
  EXPECT_EQ(describe_status_bits(0u), "no active status bits");
  EXPECT_NE(describe_status_bits(uint32_t{1} << 31).find("error summary"), std::string::npos);
  EXPECT_EQ(status_code_level(uint32_t{1} << 26), DiagnosticStatus::WARN);
  EXPECT_EQ(status_code_level(uint32_t{1} << 31), DiagnosticStatus::ERROR);
}

}  // namespace
}  // namespace ati_axia80_m20_ethercat_sensor
