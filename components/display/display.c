#include <string.h>
#include <stdlib.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "display/display.h"
#include "display/ili9341_commands.h"
#include "vospi/vospi.h"

static const char* TAG = "Display";

// The SPI handle for the display
static spi_device_handle_t spi;

// The initialisation commands used to prime the display for use
static const ili_init_cmd_t ili_init_cmds_drom[] = {

  // Software reset
  {ILI9341_SWRESET, {0}, 0},

  // Display OFF
  {ILI9341_DISPOFF, {0}, 0},

  // Unsure...
  {0xCF, {0x00, 0xC1, 0x30}, 3},
  {0xED, {0x64, 0x03, 0x12, 0x81}, 4},
  {0xE8, {0x85, 0x00, 0x78}, 3},
  {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
  {0xF7, {0x20}, 1},
  {0xEA, {0x00, 0x00}, 2},

  // Power Control
  {ILI9341_PWCTR1, {0x23}, 1},
  {ILI9341_PWCTR2, {0x10}, 1},

  // VCM
  {ILI9341_VMCTR1, {0x3e, 0x28}, 2},
  {ILI9341_VMCTR2, {0x86}, 1},

  // Memory access direction
  {ILI9341_MADCTL, {
    0 |
    (0 << 7) |  // MY
    (0 << 6) |  // MX
    (1 << 5) |  // MV
    (1 << 4) |  // ML
    (1 << 3) |  // BGR
    (1 << 2)    // MH
  }, 1},

  // Pixel format
  {ILI9341_PIXFMT, {0x55}, 1},

  // Frame control
  {ILI9341_FRMCTR1, {0x00, 0x18}, 2},

  // Gamma (off, curve 1)
  {0xF2, {0x00}, 1},
  {ILI9341_GAMMASET, {0x01}, 1},

  // Positive and negative gamma correction
  {ILI9341_GMCTRP1, {0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00}, 15},
  {ILI9341_GMCTRN1, {0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F}, 15},

  // Framerate control (119Hz)
  {0xB1, {0x00, 0x10}, 2},

  // Dummy command that indicates the end of our list
  {0, {0}, 0xff}
};

/**
 * Send a command to the display.
 */
void ili_command(const uint8_t command)
{
  esp_err_t ret;
  spi_transaction_t *transaction = heap_caps_malloc(sizeof(spi_transaction_t), MALLOC_CAP_DMA);
  assert(transaction);
  spi_transaction_user_data_t user_data;
  memset(transaction, 0, sizeof(spi_transaction_t));
  memset(&user_data, 0, sizeof(user_data));
  transaction->length = 8;
  transaction->tx_buffer = &command;
  transaction->user = &user_data;
  user_data.dc = false;
  user_data.dc_io_num = PIN_NUM_DISPLAY_DC;
  ret = spi_device_transmit(spi, transaction);
  assert(ret == ESP_OK);
  free(transaction);
}

/**
 * Send some data to the display.
 */
void ili_data(void* data, int len)
{
  // Do we need to send anything?
  if (len == 0) {
    return;
  }

  esp_err_t ret;
  spi_transaction_t *transaction = heap_caps_malloc(sizeof(spi_transaction_t), MALLOC_CAP_DMA);
  assert(transaction);
  spi_transaction_user_data_t user_data;
  memset(transaction, 0, sizeof(spi_transaction_t));
  memset(&user_data, 0, sizeof(user_data));
  transaction->length = len * 8;
  transaction->tx_buffer = data;
  transaction->user = &user_data;
  user_data.dc = true;
  user_data.dc_io_num = PIN_NUM_DISPLAY_DC;
  ret = spi_device_transmit(spi, transaction);
  assert(ret == ESP_OK);
  free(transaction);
}

/**
 * The callback used to set the state of the DC line before a SPI transaction.
 */
void ili_spi_pre_transfer_callback(spi_transaction_t* t)
{
  gpio_set_level(
    ((spi_transaction_user_data_t*)t->user)->dc_io_num,
    ((spi_transaction_user_data_t*)t->user)->dc
  );
}

/**
 * Set up the SPI hardware for the display.
 */
void display_spi_init()
{
  esp_err_t ret;

  // Set up the SPI hardware
  ESP_LOGI(TAG, "initialising SPI hardware");

  spi_bus_config_t bus_cfg = {
      .miso_io_num = VSPI_PIN_NUM_MISO,
      .mosi_io_num = VSPI_PIN_NUM_MOSI,
      .sclk_io_num = VSPI_PIN_NUM_CLK,
      .max_transfer_sz = 64000,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1
  };

  spi_device_interface_config_t devcfg = {
    .command_bits = 0,
    .address_bits = 0,
    .clock_speed_hz = DISPLAY_SPI_SPEED_HZ,
    .mode = 0,
    .spics_io_num = VSPI_PIN_NUM_CS,
    .queue_size = 64,
    .flags = SPI_DEVICE_HALFDUPLEX,
    .pre_cb = ili_spi_pre_transfer_callback,
    .cs_ena_pretrans = 10
  };

  ret = spi_bus_initialize(VSPI_HOST, &bus_cfg, 2);
  assert(ret == ESP_OK);
  ret = spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
  assert(ret == ESP_OK);
}

/**
 * Initialise the display.
 */
void display_init()
{
  // Perform SPI initialisation
  display_spi_init();

  // Send initialisation commands to the display
  ESP_LOGI(TAG, "allocating space for initialisation commands");
  ili_init_cmd_t *ili_init_cmds = heap_caps_malloc(sizeof(ili_init_cmds_drom), MALLOC_CAP_DMA);
  if (!ili_init_cmds) {
    ESP_LOGE(TAG, "no memory available for initialisation commands");
    return;
  }

  ESP_LOGI(TAG, "copying initialisation commands into place");
  memcpy(ili_init_cmds, ili_init_cmds_drom, sizeof(ili_init_cmds_drom));

  //Initialize non-SPI GPIOs
  ESP_LOGI(TAG, "setting up GPIO pins");
  gpio_set_direction(PIN_NUM_DISPLAY_DC, GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_NUM_DISPLAY_RESET, GPIO_MODE_OUTPUT);

  //Reset the display
  ESP_LOGI(TAG, "resetting the display");
  gpio_set_level(PIN_NUM_DISPLAY_RESET, 0);
  vTaskDelay(100 / portTICK_RATE_MS);
  gpio_set_level(PIN_NUM_DISPLAY_RESET, 1);
  vTaskDelay(100 / portTICK_RATE_MS);
  ESP_LOGI(TAG, "display reset done");

  //Send all the commands
  ESP_LOGI(TAG, "sending initialisation commands");
  int cmd = 0;
  while (ili_init_cmds[cmd].data_len != 0xff) {
    ESP_LOGI(TAG, "sending command @ index %d (CMD: %d)", cmd, ili_init_cmds[cmd].command);
    ili_command(ili_init_cmds[cmd].command);
    ili_data(ili_init_cmds[cmd].data, ili_init_cmds[cmd].data_len & 0x1F);
    if (ili_init_cmds[cmd].data_len & 0x80) {
      vTaskDelay(100 / portTICK_RATE_MS);
    }
    cmd ++;
  }

  ESP_LOGI(TAG, "freeing initialisation commands");
  free(ili_init_cmds);

  // Power on!
  ESP_LOGI(TAG, "turning the display on");
  ili_command(ILI9341_DISPON);

  // Exit sleep
  ESP_LOGI(TAG, "waking up the display");
  ili_command(ILI9341_SLPOUT);

  // We'll always be writing full-widths of the display, so set this up now
  ESP_LOGI(TAG, "setting up display memory layout");
  ili_command(ILI9341_CASET);
  uint8_t caset_d[4] = {
    0,
    0,
    319 >> 8,     // -
    319 & 0xff    // End position is 319
  };
  ili_data(caset_d, 4);

  // Ready for action!
}

/**
 * Write out a single segment to the display at a defined position.
 */
void display_write_segment(uint8_t seg_position, display_segment_t* segment)
{
  // Jump to the page where this segment should start
  ili_command(ILI9341_PASET);
  uint8_t paset_d[4] = {
    seg_position * 60 >> 8,
    seg_position * 60 & 0xff,
    (seg_position + 1) * 60 >> 8,
    (seg_position + 1) * 60 & 0xff
  };
  ili_data(paset_d, 4);

  // Draw the pixel values for the segment to the display
  ili_command(ILI9341_RAMWR);
  ili_data(segment, VOSPI_PACKETS_PER_SEGMENT_NORMAL * VOSPI_PACKET_SYMBOLS * 4);
}
