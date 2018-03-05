#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "cci/cci.h"
           
static const char* TAG = "CCIDriver";

/**
 * Initialse the CCI (Camera Control Interface).
 * 
 * The CCI is an I2C-based communication link between the Lepton camera and the host that permits
 * adjustment of parameters and extraction of state information.
 */
void cci_init()
{
  // Set up the I2C peripheral
  ESP_LOGI(TAG, "Initialising CCI I2C peripheral")
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = CCI_SDA_PIN;
  conf.scl_io_num = CCI_SCL_PIN;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = CCI_I2C_FREQ;
  i2c_param_config(I2C_NUM_0, &conf);
  i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

/**
 * Write a CCI register.
 */
int cci_write_register(uint16_t reg, uint16_t value)
{
  // Create the link context
  ESP_LOGD(TAG, "preparing register write (reg %02x value %02x)...", reg, value);
  i2c_cmd_handle_t command = i2c_cmd_link_create();

  // Write the message and stop the master
  i2c_master_start(command);
  i2c_master_write_byte(command, CCI_ADDRESS << 1, 1);
  i2c_master_write_byte(command, reg >> 8 & 0xff, 1);
  i2c_master_write_byte(command, reg & 0xff, 1);
  i2c_master_write_byte(command, value >> 8 & 0xff, 1);
  i2c_master_write_byte(command, value & 0xff, 1);
  i2c_master_stop(command);

  // Send the queued command
  ESP_LOGD(TAG, "performing register write...");
  if (i2c_master_cmd_begin(I2C_NUM_0, command, 1000 / portTICK_RATE_MS) != ESP_OK) {
    ESP_LOGE(TAG, "failed to write CCI register write command to bus: %02x with value %02x", reg, value);
    return -1;
  }

  // Delete the link context
  i2c_cmd_link_delete(command);

  return 1;
}

/**
 * Read a CCI register.
 */
uint16_t cci_read_register(uint16_t reg)
{
  // Response
  uint8_t resp_h, resp_l = 0;

  // Create the link context
  ESP_LOGD(TAG, "preparing register read (reg %02x)...", reg);
  i2c_cmd_handle_t command = i2c_cmd_link_create();

  // Write the message, read the response and stop the master
  i2c_master_start(command);
  i2c_master_write_byte(command, CCI_ADDRESS << 1, 1);
  i2c_master_write_byte(command, reg >> 8 & 0xff, 1);
  i2c_master_write_byte(command, reg & 0xff, 1);
  i2c_master_stop(command);

  // Send the queued command
  ESP_LOGD(TAG, "performing register read setup...");
  if (i2c_master_cmd_begin(I2C_NUM_0, command, 1000 / portTICK_RATE_MS) != ESP_OK) {
    ESP_LOGD(TAG, "failed to setup read of register: %02x", reg);
    return -1;
  }

  // Delete the link context
  i2c_cmd_link_delete(command);

  // Now perform a read
  command = i2c_cmd_link_create();
  i2c_master_start(command);
  i2c_master_write_byte(command, (CCI_ADDRESS << 1) | 0x01, 1);
  i2c_master_read_byte(command, &resp_h, 0);
  i2c_master_read_byte(command, &resp_l, 1);
  i2c_master_stop(command);

  // Send the queued command
  ESP_LOGD(TAG, "performing register read...");
  if (i2c_master_cmd_begin(I2C_NUM_0, command, 1000 / portTICK_RATE_MS) != ESP_OK) {
    ESP_LOGD(TAG, "failed to read register: %02x", reg);
    return -1;
  }

  // Return the response data
  return resp_h << 8 | resp_l;
}

/**
 * Wait for the busy flag to be deasserted.
 * If the busy flag is asserted, we'll sleep for 10ms and try again upto a maximum amount of time.
 * 
 * If we finish without the busy flag being deasserted, return false; otherwise return true.
 */
bool wait_for_busy_deassert(int timeout)
{
  ESP_LOGD(TAG, "waiting for busy deassert...")
  while (cci_read_register(CCI_REG_STATUS) & 0x01) ;
  return true;
}

/**
 * Get the result of the last command.
 */
int8_t get_command_result()
{
  ESP_LOGD(TAG, "getting command result state...");
  return (cci_read_register(CCI_REG_STATUS) >> 8) & 0xff;
}

/**
 * Request that a flat field correction occur immediately.
 */
void cci_run_ffc()
{
  ESP_LOGI(TAG, "requesting flat-field correction...");
  wait_for_busy_deassert(CCI_BUSY_WAIT_TIMEOUT);
  cci_write_register(CCI_REG_COMMAND, CCI_CMD_SYS_RUN_FFC);
  wait_for_busy_deassert(CCI_BUSY_WAIT_TIMEOUT);
  ESP_LOGI(TAG, "FFC request completed");
}

/**
 * Get the FFC state of the camera.
 */
cci_ffc_state_t cci_get_ffc_state()
{
  ESP_LOGI(TAG, "requesting FFC state...");
  wait_for_busy_deassert(CCI_BUSY_WAIT_TIMEOUT);
  cci_write_register(CCI_REG_DATA_LENGTH, 2);
  cci_write_register(CCI_REG_COMMAND, CCI_CMD_SYS_GET_FFC_STATE);
  wait_for_busy_deassert(CCI_BUSY_WAIT_TIMEOUT);

  // Get the command result
  int8_t command_result = get_command_result();
  if (command_result != 0) {
    ESP_LOGE(TAG, "command failed - code: %d", command_result);
    return CCI_FFC_ERROR;
  }

  // Read the FFC state as a 32-bit value
  return cci_read_register(CCI_REG_DATA_0) |
    cci_read_register(CCI_REG_DATA_0 + CCI_WORD_LENGTH) << 16;
}