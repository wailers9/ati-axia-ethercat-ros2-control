#include "ati_axia80_m20_ethercat_sensor/axia80_ethercat_config.hpp"

namespace ati_axia80_m20_ethercat_sensor
{

ec_pdo_entry_info_t * axia_pdo_entries()
{
  static ec_pdo_entry_info_t entries[] = {
    {OBJ_CONTROL_CODES, SUB_CONTROL_1, 32},
    {OBJ_CONTROL_CODES, SUB_CONTROL_2, 32},
    {OBJ_READING_DATA, 0x01, 32},
    {OBJ_READING_DATA, 0x02, 32},
    {OBJ_READING_DATA, 0x03, 32},
    {OBJ_READING_DATA, 0x04, 32},
    {OBJ_READING_DATA, 0x05, 32},
    {OBJ_READING_DATA, 0x06, 32},
    {OBJ_STATUS_CODE, 0x00, 32},
    {OBJ_SAMPLE_COUNTER, 0x00, 32},
  };
  return entries;
}

ec_pdo_info_t * axia_pdos()
{
  static ec_pdo_info_t pdos[] = {
    {PDO_RX_CONTROL, 2, axia_pdo_entries() + 0},
    {PDO_TX_READING, 8, axia_pdo_entries() + 2},
  };
  return pdos;
}

ec_sync_info_t * axia_syncs()
{
  static ec_sync_info_t syncs[] = {
    {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, nullptr, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, axia_pdos() + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, axia_pdos() + 1, EC_WD_DISABLE},
    {0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DEFAULT},
  };
  return syncs;
}

std::array<ec_pdo_entry_reg_t, 11> make_domain_regs(
  const AxiaEtherCATConfig & config, AxiaPdoOffsets & offsets)
{
  return {{
    {config.alias, config.position, config.vendor_id, config.product_code,
      OBJ_CONTROL_CODES, SUB_CONTROL_1, &offsets.control_1, nullptr},
    {config.alias, config.position, config.vendor_id, config.product_code,
      OBJ_CONTROL_CODES, SUB_CONTROL_2, &offsets.control_2, nullptr},
    {config.alias, config.position, config.vendor_id, config.product_code,
      OBJ_READING_DATA, 0x01, &offsets.wrench_counts[0], nullptr},
    {config.alias, config.position, config.vendor_id, config.product_code,
      OBJ_READING_DATA, 0x02, &offsets.wrench_counts[1], nullptr},
    {config.alias, config.position, config.vendor_id, config.product_code,
      OBJ_READING_DATA, 0x03, &offsets.wrench_counts[2], nullptr},
    {config.alias, config.position, config.vendor_id, config.product_code,
      OBJ_READING_DATA, 0x04, &offsets.wrench_counts[3], nullptr},
    {config.alias, config.position, config.vendor_id, config.product_code,
      OBJ_READING_DATA, 0x05, &offsets.wrench_counts[4], nullptr},
    {config.alias, config.position, config.vendor_id, config.product_code,
      OBJ_READING_DATA, 0x06, &offsets.wrench_counts[5], nullptr},
    {config.alias, config.position, config.vendor_id, config.product_code,
      OBJ_STATUS_CODE, 0x00, &offsets.status_code, nullptr},
    {config.alias, config.position, config.vendor_id, config.product_code,
      OBJ_SAMPLE_COUNTER, 0x00, &offsets.sample_counter, nullptr},
    {}
  }};
}

}  // namespace ati_axia80_m20_ethercat_sensor
