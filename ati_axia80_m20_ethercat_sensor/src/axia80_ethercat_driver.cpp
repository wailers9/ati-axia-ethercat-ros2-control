#include "ati_axia80_m20_ethercat_sensor/axia80_ethercat_driver.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/mman.h>

#include "rclcpp/rclcpp.hpp"

namespace ati_axia80_m20_ethercat_sensor
{
namespace
{
using namespace std::chrono_literals;

constexpr size_t MAX_SAFE_STACK = 8 * 1024;
constexpr auto STARTUP_SDO_WARN_THRESHOLD = 500ms;
constexpr uint32_t CONTROL_SET_BIAS = 1u << 0;
constexpr uint32_t CONTROL_CLEAR_BIAS = 1u << 2;

void prefault_stack()
{
  unsigned char dummy[MAX_SAFE_STACK];
  std::memset(dummy, 0, MAX_SAFE_STACK);
}

uint32_t le_u32(const uint8_t * data)
{
  return static_cast<uint32_t>(data[0]) |
    (static_cast<uint32_t>(data[1]) << 8) |
    (static_cast<uint32_t>(data[2]) << 16) |
    (static_cast<uint32_t>(data[3]) << 24);
}

uint16_t le_u16(const uint8_t * data)
{
  return static_cast<uint16_t>(data[0]) |
    static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8);
}

std::string hex_object(uint16_t index, uint8_t subindex)
{
  std::ostringstream stream;
  stream << "0x" << std::hex << index << ":0x" << static_cast<unsigned int>(subindex);
  return stream.str();
}

std::string clean_sdo_string(const std::vector<uint8_t> & data)
{
  const auto end = std::find(data.begin(), data.end(), uint8_t{0});
  return std::string(data.begin(), end);
}

void validate_scale(const std::string & name, double value)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::runtime_error(name + " must be a finite positive value");
  }
}
}  // namespace

Axia80EtherCATDriver::Axia80EtherCATDriver(const Axia80DriverParameters & parameters)
: parameters_(parameters), logger_(rclcpp::get_logger("Axia80EtherCATDriver"))
{
  validate_scale("counts_per_force", parameters_.counts_per_force);
  validate_scale("counts_per_torque", parameters_.counts_per_torque);
}

Axia80EtherCATDriver::~Axia80EtherCATDriver()
{
  shutdown();
}

void Axia80EtherCATDriver::init()
{
  // Keep EtherCAT cyclic memory resident; this mirrors the gripper reference driver.
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    RCLCPP_WARN(logger_, "mlockall failed; continuing without locked memory");
  }
  prefault_stack();

  request_master_();
  find_slave_();
  configure_pdos_();
  activate_master_();
  write_control_(control_word_(), 0);
  cycle_once_();

  if (parameters_.read_calibration_sdo) {
    const auto start = std::chrono::steady_clock::now();
    read_calibration_sdo_();
    const auto elapsed = std::chrono::steady_clock::now() - start;
    if (elapsed > STARTUP_SDO_WARN_THRESHOLD) {
      const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
      RCLCPP_WARN(
        logger_,
        "Startup calibration SDO took %lld ms",
        static_cast<long long>(elapsed_ms));
    }
  }
}

void Axia80EtherCATDriver::shutdown()
{
  if (master_) {
    if (active_) {
      write_control_(0, 0);
      cycle_once_();
      ecrt_master_deactivate(master_);
      active_ = false;
    }
    ecrt_release_master(master_);
    master_ = nullptr;
  }
  domain_ = nullptr;
  slave_config_ = nullptr;
  domain_pd_ = nullptr;
  state_check_required_ = true;
}

void Axia80EtherCATDriver::set_bias()
{
  pulse_control_bit_(CONTROL_SET_BIAS);
}

void Axia80EtherCATDriver::clear_bias()
{
  pulse_control_bit_(CONTROL_CLEAR_BIAS);
}

bool Axia80EtherCATDriver::is_active() const
{
  return active_ && domain_pd_ != nullptr;
}

Axia80Sample Axia80EtherCATDriver::read_once()
{
  if (!is_active()) {
    throw std::runtime_error("EtherCAT driver is not active");
  }
  validate_scale("counts_per_force", parameters_.counts_per_force);
  validate_scale("counts_per_torque", parameters_.counts_per_torque);

  ecrt_master_receive(master_);
  ecrt_domain_process(domain_);
  const auto now = std::chrono::steady_clock::now();
  if (state_check_required_ || now >= next_state_check_time_) {
    check_ethercat_state_();
    state_check_required_ = false;
    next_state_check_time_ = now + 1s;
  }

  Axia80Sample sample;
  sample.wrench.header.stamp = steady_clock_.now();
  sample.wrench.wrench.force.x =
    static_cast<double>(EC_READ_S32(domain_pd_ + offsets_.wrench_counts[0])) /
    parameters_.counts_per_force;
  sample.wrench.wrench.force.y =
    static_cast<double>(EC_READ_S32(domain_pd_ + offsets_.wrench_counts[1])) /
    parameters_.counts_per_force;
  sample.wrench.wrench.force.z =
    static_cast<double>(EC_READ_S32(domain_pd_ + offsets_.wrench_counts[2])) /
    parameters_.counts_per_force;
  sample.wrench.wrench.torque.x =
    static_cast<double>(EC_READ_S32(domain_pd_ + offsets_.wrench_counts[3])) /
    parameters_.counts_per_torque;
  sample.wrench.wrench.torque.y =
    static_cast<double>(EC_READ_S32(domain_pd_ + offsets_.wrench_counts[4])) /
    parameters_.counts_per_torque;
  sample.wrench.wrench.torque.z =
    static_cast<double>(EC_READ_S32(domain_pd_ + offsets_.wrench_counts[5])) /
    parameters_.counts_per_torque;
  sample.status_code = EC_READ_U32(domain_pd_ + offsets_.status_code);
  sample.sample_counter = EC_READ_U32(domain_pd_ + offsets_.sample_counter);
  sample.valid = true;

  write_control_(control_word_(), 0);
  ecrt_master_sync_reference_clock(master_);
  ecrt_master_sync_slave_clocks(master_);
  ecrt_domain_queue(domain_);
  ecrt_master_send(master_);

  return sample;
}

Axia80DiagnosticReadings Axia80EtherCATDriver::read_diagnostic_readings() const
{
  if (!master_) {
    throw std::runtime_error("cannot read Axia diagnostics before EtherCAT master is requested");
  }

  Axia80DiagnosticReadings readings;
  readings.supply_voltage_v = static_cast<double>(
    upload_sdo_u16_(OBJ_DIAGNOSTIC_READINGS, SUB_DIAGNOSTIC_SUPPLY_VOLTAGE)) / 10.0;
  readings.gage_temperature_c = static_cast<double>(
    upload_sdo_i16_(OBJ_DIAGNOSTIC_READINGS, SUB_DIAGNOSTIC_GAGE_TEMPERATURE)) / 10.0;
  readings.status_message = upload_sdo_string_(
    OBJ_DIAGNOSTIC_READINGS, SUB_DIAGNOSTIC_STATUS_MESSAGE, 40);
  return readings;
}

void Axia80EtherCATDriver::request_master_()
{
  master_ = ecrt_request_master(parameters_.master_index);
  if (!master_) {
    throw std::runtime_error("failed to request EtherCAT master " +
      std::to_string(parameters_.master_index));
  }
  if (ecrt_master(master_, &master_info_) != 0) {
    throw std::runtime_error("failed to read EtherCAT master info");
  }
}

void Axia80EtherCATDriver::find_slave_()
{
  bool found = false;
  for (unsigned int i = 0; i < master_info_.slave_count; ++i) {
    ec_slave_info_t info;
    if (ecrt_master_get_slave(master_, i, &info) != 0) {
      continue;
    }

    const bool product_matches =
      info.vendor_id == parameters_.slave.vendor_id &&
      info.product_code == parameters_.slave.product_code;
    const bool alias_matches =
      parameters_.slave.alias == 0 || info.alias == parameters_.slave.alias;
    const bool position_matches =
      parameters_.slave.position == i || parameters_.slave.position == info.position;

    if (product_matches && alias_matches && position_matches) {
      slave_info_ = info;
      parameters_.slave.alias = info.alias;
      parameters_.slave.position = info.position;
      found = true;
      break;
    }
  }

  if (!found) {
    throw std::runtime_error("ATI Axia EtherCAT slave was not found");
  }
}

void Axia80EtherCATDriver::configure_pdos_()
{
  domain_ = ecrt_master_create_domain(master_);
  if (!domain_) {
    throw std::runtime_error("failed to create EtherCAT domain");
  }

  slave_config_ = ecrt_master_slave_config(
    master_,
    parameters_.slave.alias,
    parameters_.slave.position,
    parameters_.slave.vendor_id,
    parameters_.slave.product_code);
  if (!slave_config_) {
    throw std::runtime_error("failed to create Axia slave configuration");
  }

  if (ecrt_slave_config_pdos(slave_config_, EC_END, axia_syncs()) != 0) {
    throw std::runtime_error("failed to configure Axia PDOs");
  }

  auto regs = make_domain_regs(parameters_.slave, offsets_);
  if (ecrt_domain_reg_pdo_entry_list(domain_, regs.data()) != 0) {
    throw std::runtime_error("failed to register Axia PDO entries");
  }
}

void Axia80EtherCATDriver::read_calibration_sdo_()
{
  const auto calibration = read_calibration_info_sdo_();
  parameters_.counts_per_force = calibration.counts_per_force;
  parameters_.counts_per_torque = calibration.counts_per_torque;

  validate_scale("counts_per_force read from Axia SDO 0x2021:0x37", parameters_.counts_per_force);
  validate_scale("counts_per_torque read from Axia SDO 0x2021:0x38", parameters_.counts_per_torque);

  RCLCPP_INFO(
    logger_,
    "Axia calibration: ft_serial='%s' part_number='%s' calibration_time='%s' "
    "counts_per_force=%.3f counts_per_torque=%.3f active_calibration_slot=%u",
    calibration.ft_serial.c_str(),
    calibration.calibration_part_number.c_str(),
    calibration.calibration_time.c_str(),
    parameters_.counts_per_force,
    parameters_.counts_per_torque,
    static_cast<unsigned int>(calibration.active_calibration_slot));
}

void Axia80EtherCATDriver::activate_master_()
{
  if (ecrt_master_activate(master_) < 0) {
    throw std::runtime_error("failed to activate EtherCAT master");
  }
  domain_pd_ = ecrt_domain_data(domain_);
  if (!domain_pd_) {
    throw std::runtime_error("failed to get EtherCAT domain process data pointer");
  }
  state_check_required_ = true;
  active_ = true;
}

void Axia80EtherCATDriver::write_control_(uint32_t control_1, uint32_t control_2)
{
  EC_WRITE_U32(domain_pd_ + offsets_.control_1, control_1);
  EC_WRITE_U32(domain_pd_ + offsets_.control_2, control_2);
}

void Axia80EtherCATDriver::cycle_once_()
{
  ecrt_master_receive(master_);
  ecrt_domain_process(domain_);
  ecrt_domain_queue(domain_);
  ecrt_master_send(master_);
}

void Axia80EtherCATDriver::pulse_control_bit_(uint32_t bit_mask)
{
  if (!is_active()) {
    throw std::runtime_error("cannot pulse control bit before EtherCAT activation");
  }

  write_control_(control_word_() | bit_mask, 0);
  cycle_once_();
  write_control_(control_word_(), 0);
  cycle_once_();
}

uint32_t Axia80EtherCATDriver::control_word_() const
{
  uint32_t word = 0;
  word |= (static_cast<uint32_t>(parameters_.filter_selection & 0x0F) << 4);
  word |= (static_cast<uint32_t>(parameters_.calibration_slot & 0x0F) << 8);
  word |= (static_cast<uint32_t>(parameters_.sample_rate_code & 0x0F) << 12);
  return word;
}

void Axia80EtherCATDriver::check_ethercat_state_()
{
  ec_domain_state_t domain_state{};
  ecrt_domain_state(domain_, &domain_state);
  if (!have_domain_state_ ||
    domain_state.working_counter != last_domain_state_.working_counter ||
    domain_state.wc_state != last_domain_state_.wc_state)
  {
    RCLCPP_INFO(
      logger_,
      "EtherCAT domain state: working_counter=%u wc_state=%u",
      domain_state.working_counter,
      static_cast<unsigned int>(domain_state.wc_state));
    last_domain_state_ = domain_state;
    have_domain_state_ = true;
  }

  ec_master_state_t master_state{};
  ecrt_master_state(master_, &master_state);
  if (!have_master_state_ ||
    master_state.slaves_responding != last_master_state_.slaves_responding ||
    master_state.al_states != last_master_state_.al_states ||
    master_state.link_up != last_master_state_.link_up)
  {
    RCLCPP_INFO(
      logger_,
      "EtherCAT master state: slaves_responding=%u al_states=0x%02x link_up=%u",
      master_state.slaves_responding,
      master_state.al_states,
      static_cast<unsigned int>(master_state.link_up));
    last_master_state_ = master_state;
    have_master_state_ = true;
  }

  ec_slave_config_state_t slave_state{};
  ecrt_slave_config_state(slave_config_, &slave_state);
  if (!have_slave_state_ ||
    slave_state.online != last_slave_state_.online ||
    slave_state.operational != last_slave_state_.operational ||
    slave_state.al_state != last_slave_state_.al_state)
  {
    RCLCPP_INFO(
      logger_,
      "Axia slave state: online=%u operational=%u al_state=0x%02x",
      static_cast<unsigned int>(slave_state.online),
      static_cast<unsigned int>(slave_state.operational),
      slave_state.al_state);
    last_slave_state_ = slave_state;
    have_slave_state_ = true;
  }
}

Axia80CalibrationInfo Axia80EtherCATDriver::read_calibration_info_sdo_()
{
  Axia80CalibrationInfo calibration;
  calibration.ft_serial = upload_sdo_string_(OBJ_CALIBRATION, 0x01, 8);
  calibration.calibration_part_number = upload_sdo_string_(OBJ_CALIBRATION, 0x02, 30);
  calibration.calibration_time = upload_sdo_string_(OBJ_CALIBRATION, 0x04, 30);
  calibration.counts_per_force = static_cast<double>(
    upload_sdo_u32_(OBJ_CALIBRATION, SUB_COUNTS_PER_FORCE));
  calibration.counts_per_torque = static_cast<double>(
    upload_sdo_u32_(OBJ_CALIBRATION, SUB_COUNTS_PER_TORQUE));
  calibration.active_calibration_slot = parameters_.calibration_slot;
  return calibration;
}

std::vector<uint8_t> Axia80EtherCATDriver::upload_sdo_(
  uint16_t index, uint8_t subindex, size_t size) const
{
  std::vector<uint8_t> data(size);
  size_t result_size = 0;
  uint32_t abort_code = 0;
  const int ret = ecrt_master_sdo_upload(
    master_, slave_info_.position, index, subindex, data.data(), data.size(), &result_size, &abort_code);

  if (ret != 0 || result_size > data.size()) {
    throw std::runtime_error(
      "failed SDO upload " + hex_object(index, subindex) +
      " abort_code=" + std::to_string(abort_code));
  }
  data.resize(result_size);
  return data;
}

uint32_t Axia80EtherCATDriver::upload_sdo_u32_(uint16_t index, uint8_t subindex) const
{
  const auto data = upload_sdo_(index, subindex, 4);
  if (data.size() != 4) {
    throw std::runtime_error("unexpected SDO size for " + hex_object(index, subindex));
  }
  return le_u32(data.data());
}

int16_t Axia80EtherCATDriver::upload_sdo_i16_(uint16_t index, uint8_t subindex) const
{
  const auto data = upload_sdo_(index, subindex, 2);
  if (data.size() != 2) {
    throw std::runtime_error("unexpected SDO size for " + hex_object(index, subindex));
  }
  return static_cast<int16_t>(le_u16(data.data()));
}

uint16_t Axia80EtherCATDriver::upload_sdo_u16_(uint16_t index, uint8_t subindex) const
{
  const auto data = upload_sdo_(index, subindex, 2);
  if (data.size() != 2) {
    throw std::runtime_error("unexpected SDO size for " + hex_object(index, subindex));
  }
  return le_u16(data.data());
}

std::string Axia80EtherCATDriver::upload_sdo_string_(
  uint16_t index, uint8_t subindex, size_t size) const
{
  return clean_sdo_string(upload_sdo_(index, subindex, size));
}

}  // namespace ati_axia80_m20_ethercat_sensor
