#ifndef ATI_AXIA80_M20_ETHERCAT_SENSOR__AXIA80_ETHERCAT_CONFIG_HPP_
#define ATI_AXIA80_M20_ETHERCAT_SENSOR__AXIA80_ETHERCAT_CONFIG_HPP_

#include <array>
#include <cstdint>

#include <ecrt.h>

namespace ati_axia80_m20_ethercat_sensor
{

// Values come from "200807 - ATI Axia EtherCAT FT.xml".
constexpr uint32_t ATI_VENDOR_ID = 0x00000732;
constexpr uint32_t AXIA_PRODUCT_CODE = 0x26483053;
constexpr uint32_t AXIA_REVISION = 0x00000001;

constexpr uint16_t PDO_RX_CONTROL = 0x1601;
constexpr uint16_t PDO_TX_READING = 0x1A00;

constexpr uint16_t OBJ_READING_DATA = 0x6000;
constexpr uint16_t OBJ_STATUS_CODE = 0x6010;
constexpr uint16_t OBJ_SAMPLE_COUNTER = 0x6020;
constexpr uint16_t OBJ_CONTROL_CODES = 0x7010;
constexpr uint16_t OBJ_CALIBRATION = 0x2021;
constexpr uint16_t OBJ_DIAGNOSTIC_READINGS = 0x2080;

constexpr uint8_t SUB_CONTROL_1 = 0x01;
constexpr uint8_t SUB_CONTROL_2 = 0x02;
constexpr uint8_t SUB_COUNTS_PER_FORCE = 0x37;
constexpr uint8_t SUB_COUNTS_PER_TORQUE = 0x38;
constexpr uint8_t SUB_DIAGNOSTIC_SUPPLY_VOLTAGE = 0x01;
constexpr uint8_t SUB_DIAGNOSTIC_GAGE_TEMPERATURE = 0x02;
constexpr uint8_t SUB_DIAGNOSTIC_STATUS_MESSAGE = 0x03;

struct AxiaPdoOffsets
{
  std::array<unsigned int, 6> wrench_counts{};
  unsigned int status_code{};
  unsigned int sample_counter{};
  unsigned int control_1{};
  unsigned int control_2{};
};

struct AxiaEtherCATConfig
{
  uint32_t vendor_id{ATI_VENDOR_ID};
  uint32_t product_code{AXIA_PRODUCT_CODE};
  uint32_t revision{AXIA_REVISION};
  uint16_t alias{0};
  uint16_t position{0};
};

ec_pdo_entry_info_t * axia_pdo_entries();
ec_pdo_info_t * axia_pdos();
ec_sync_info_t * axia_syncs();

std::array<ec_pdo_entry_reg_t, 11> make_domain_regs(
  const AxiaEtherCATConfig & config, AxiaPdoOffsets & offsets);

}  // namespace ati_axia80_m20_ethercat_sensor

#endif  // ATI_AXIA80_M20_ETHERCAT_SENSOR__AXIA80_ETHERCAT_CONFIG_HPP_
