#include <stdio.h>
#include "esp_log.h"
#include "xtensa/core-macros.h"
#include "freertos/FreeRTOS.h"
#include "freeRTOS/portmacro.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "cci/cci.h"
#include "driver/gpio.h"
           
static const char* TAG = "CCIDriver";

/**
 * Initialse the CCI (Camera Control Interface).
 * 
 * The CCI is an I2C-like communication link between the Lepton camera and the host that permits
 * adjustment of parameters and extraction of state information.
 */
void cci_init()
{
  // Set up the I2C peripheral
  ESP_LOGI(TAG, "setting up GPIO pins");
  gpio_set_level(CCI_SDA_PIN, 1);
  gpio_set_level(CCI_SCL_PIN, 1);
  gpio_set_direction(CCI_SCL_PIN, GPIO_MODE_INPUT_OUTPUT_OD);
  gpio_set_direction(CCI_SDA_PIN, GPIO_MODE_INPUT_OUTPUT_OD);
  gpio_set_pull_mode(CCI_SCL_PIN, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(CCI_SDA_PIN, GPIO_PULLUP_ONLY);
  gpio_set_drive_capability(CCI_SDA_PIN, GPIO_DRIVE_CAP_1);
}

/**
 * Insert a wait.
 */
void wait(uint8_t multiple)
{
  uint32_t c_current = 0, c_start = XTHAL_GET_CCOUNT();
  do {
    c_current = XTHAL_GET_CCOUNT();
  } while (c_current - c_start < I2C_WAIT_CYCLES);
}

void scl_hi()
{
  gpio_set_level(CCI_SCL_PIN, 1);
  wait(1);
}

void scl_lo()
{
  gpio_set_level(CCI_SCL_PIN, 0);
  wait(1);
}

void sda_hi()
{
  gpio_set_level(CCI_SDA_PIN, 1);
  wait(1);
}

void sda_lo()
{
  gpio_set_level(CCI_SDA_PIN, 0);
  wait(1);
}

bool scl_read()
{
  return gpio_get_level(CCI_SCL_PIN) != 0;
}

bool sda_read()
{
  return gpio_get_level(CCI_SDA_PIN) != 0;
}

/**
 * Send a start bit sequence.
 */
void i2c_start()
{
  sda_hi();
  scl_hi();
  sda_lo();
  scl_lo();
  wait(1);
}

/**
 * Send a stop bit sequence.
 */
void i2c_stop()
{
  sda_lo();
  scl_hi();
  sda_hi();
  wait(1);
}

uint8_t i2c_rx(bool send_ack)
{
  uint8_t data = 0;
  sda_hi();

  for (uint x = 0; x < 8; x ++) {

    // Shift into `data`
    data <<= 1;

    scl_hi();
    wait(1);
    if (sda_read()) {
      data |= 1;
    }
    scl_lo();

    // Should we acknowledge?
    if (send_ack) {
      sda_lo();
    } else {
      sda_hi();
    }
  }

  scl_hi();
  scl_lo();
  sda_hi();
  return data;
}

bool i2c_tx(uint8_t data)
{
  // Shift out `data`
  for (uint x = 8; x; x--) {
    if (data & 0x80) {
      sda_hi();
    } else {
      sda_lo();
    }
    scl_hi();
    data <<= 1;
    scl_lo();
  }

  // Read the acknowledgement (if present)
  sda_hi();
  scl_hi();
  bool ret = sda_read() ? false : true;
  scl_lo();

  return ret;
}

/**
 * Write a CCI register.
 */
int cci_write_register(uint16_t reg, uint16_t value)
{
  portMUX_TYPE mut = portMUX_INITIALIZER_UNLOCKED;
  taskENTER_CRITICAL(&mut);

  ESP_LOGD(TAG, "performing register write (reg %02x value %02x)...", reg, value);
  
  i2c_start();
  bool write_ok = i2c_tx(CCI_ADDRESS << 1);
  write_ok &= i2c_tx(reg >> 8 & 0xff);
  write_ok &= i2c_tx(reg & 0xff);
  write_ok &= i2c_tx(value >> 8 & 0xff);
  write_ok &= i2c_tx(value & 0xff);

  if (!write_ok) {
    taskEXIT_CRITICAL(&mut);
    ESP_LOGW(TAG, "register write %02x (value %02x) was not properly ack'd", reg, value);
    return -1;
  }

  i2c_stop();
  taskEXIT_CRITICAL(&mut);
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

  // Write the register address we'd like to read
  // ESP_LOGD(TAG, "performing register read setup...");
  // i2c_start();
  // bool setup_ok = i2c_tx(CCI_ADDRESS << 1);
  // setup_ok &= i2c_tx(reg >> 8 & 0xff);
  // setup_ok &= i2c_tx(reg & 0xff);

  // // Check the acks
  // if (!setup_ok) {
  //   ESP_LOGW(TAG, "setup portion of register read %02x was not properly ack'd", reg);
  //   return -1;
  // }

  // Now perform a read
  i2c_start();

  // CCI address with the R/W bit SET (R)
  ESP_LOGD(TAG, "performing register read...");
  bool read_ok = i2c_tx((CCI_ADDRESS << 1) | 0x01);
  resp_h = i2c_rx(true);
  resp_l = i2c_rx(false);
  i2c_stop();

  // Send the queued command
  if (!read_ok) {
    ESP_LOGW(TAG, "read portion of register read %02x was not properly ack'd", reg);
    return -1;
  }

  ESP_LOGD(TAG, "read complete - h: %02x, l: %02x",  resp_h, resp_l);

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
  while (cci_read_register(CCI_REG_STATUS) & 0x01) {
    vTaskDelay(CCI_BUSY_WAIT_AMOUNT / portTICK_RATE_MS);
    timeout -= (timeout < CCI_BUSY_WAIT_AMOUNT ? timeout - CCI_BUSY_WAIT_AMOUNT : CCI_BUSY_WAIT_AMOUNT);
    if (timeout == 0) {
      ESP_LOGW(TAG, "timed out waiting for busy deassert...")
      return false;
    }
  }
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

/**
 * Get the camera's FPA temperature in kelvin.
 */
uint16_t cci_get_fpa_temp_k()
{
  ESP_LOGI(TAG, "requesting FPA temperature...");
  wait_for_busy_deassert(CCI_BUSY_WAIT_TIMEOUT);
  cci_write_register(CCI_REG_DATA_LENGTH, 1);
  cci_write_register(CCI_REG_COMMAND, CCI_CMD_SYS_GET_FPA_TEMP_K);
  wait_for_busy_deassert(CCI_BUSY_WAIT_TIMEOUT);

  // Get the command result
  int8_t command_result = get_command_result();
  if (command_result != 0) {
    ESP_LOGE(TAG, "command failed - code: %d", command_result);
    return 0;
  }

  return cci_read_register(CCI_REG_DATA_0);
}

/**
 * Get the camera's uptime.
 */
uint32_t cci_get_uptime()
{
  ESP_LOGI(TAG, "requesting camera uptime...");
  wait_for_busy_deassert(CCI_BUSY_WAIT_TIMEOUT);
  cci_write_register(CCI_REG_DATA_LENGTH, 2);
  cci_write_register(CCI_REG_COMMAND, CCI_CMD_SYS_GET_UPTIME);
  wait_for_busy_deassert(CCI_BUSY_WAIT_TIMEOUT);

  uint16_t ls_word = cci_read_register(CCI_REG_DATA_0);
  uint16_t ms_word = cci_read_register(CCI_REG_DATA_0 + CCI_WORD_LENGTH);
  return ms_word << 16 | ls_word;
}