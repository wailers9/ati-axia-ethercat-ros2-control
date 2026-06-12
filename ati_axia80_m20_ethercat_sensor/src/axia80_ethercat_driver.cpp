#include "ati_axia80_m20_ethercat_sensor/axia80_ethercat_driver.hpp"

#include <array>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <sys/mman.h>

#include "rclcpp/rclcpp.hpp"

namespace ati_axia80_m20_ethercat_sensor
{
namespace
{
constexpr size_t MAX_SAFE_STACK = 8 * 1024;
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
  if (parameters_.read_calibration_sdo) {
    read_calibration_sdo_();
  }
  activate_master_();

  write_control_(control_word_(), 0);
  cycle_once_();
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
  parameters_.counts_per_force = static_cast<double>(
    upload_sdo_u32_(OBJ_CALIBRATION, SUB_COUNTS_PER_FORCE));
  parameters_.counts_per_torque = static_cast<double>(
    upload_sdo_u32_(OBJ_CALIBRATION, SUB_COUNTS_PER_TORQUE));

  validate_scale("counts_per_force read from Axia SDO 0x2021:0x37", parameters_.counts_per_force);
  validate_scale("counts_per_torque read from Axia SDO 0x2021:0x38", parameters_.counts_per_torque);

  RCLCPP_INFO(
    logger_,
    "Axia calibration: counts_per_force=%.3f counts_per_torque=%.3f",
    parameters_.counts_per_force,
    parameters_.counts_per_torque);
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

uint32_t Axia80EtherCATDriver::upload_sdo_u32_(uint16_t index, uint8_t subindex) const
{
  std::array<uint8_t, 4> data{};
  size_t result_size = 0;
  uint32_t abort_code = 0;
  const int ret = ecrt_master_sdo_upload(
    master_, slave_info_.position, index, subindex, data.data(), data.size(), &result_size, &abort_code);

  if (ret != 0 || result_size != data.size()) {
    throw std::runtime_error(
      "failed SDO upload 0x" + std::to_string(index) + ":" + std::to_string(subindex) +
      " abort_code=" + std::to_string(abort_code));
  }
  return le_u32(data.data());
}

}  // namespace ati_axia80_m20_ethercat_sensor
