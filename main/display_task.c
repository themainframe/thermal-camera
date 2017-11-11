#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include <string.h>
#include "ili9341_cmds.h"

#define VSPI_PIN_NUM_MISO 19
#define VSPI_PIN_NUM_MOSI 23
#define VSPI_PIN_NUM_CLK  18
#define VSPI_PIN_NUM_CS   5
#define PIN_NUM_DC        2
#define PIN_NUM_RESET     33

static const char* TAG = "DisplayTask";

typedef struct ili_device_t {
  spi_device_handle_t spi;
  int dc_io_num;
  int reset_io_num;
  int spics_io_num;
} ili_device_t;

typedef struct {
  uint8_t command;
  uint8_t data[16];
  uint8_t data_len;
} ili_init_cmd_t;

typedef struct {
  bool dc;
  uint8_t dc_io_num;
} spi_transaction_user_data_t;

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
    (0 << 3) |  // BGR
    (1 << 2)    // MH
  }, 1},

  // Pixel format
  {ILI9341_PIXFMT, {0x55}, 1},

  // Frame control
  {ILI9341_FRMCTR1, {0x00, 0x18}, 2},

  // Display function
  {ILI9341_DFUNCTR, {0x08, 0x82, 0x27}, 3},

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

void ili_command(ili_device_t* ili_handle, const uint8_t command)
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
  user_data.dc_io_num = ili_handle->dc_io_num;
  ret = spi_device_transmit(ili_handle->spi, transaction);
  assert(ret == ESP_OK);
  free(transaction);
}

void ili_data(ili_device_t* ili_handle, const uint8_t* data, int len)
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
  user_data.dc_io_num = ili_handle->dc_io_num;
  ret = spi_device_transmit(ili_handle->spi, transaction);
  assert(ret == ESP_OK);
  free(transaction);
}

void ili_spi_pre_transfer_callback(spi_transaction_t* t)
{
  gpio_set_level(
    ((spi_transaction_user_data_t*)t->user)->dc_io_num,
    ((spi_transaction_user_data_t*)t->user)->dc
  );
}

//Initialize the display
esp_err_t ili_init(ili_device_t* device)
{
  ESP_LOGI(TAG, "allocating space for initialisation commands");
  ili_init_cmd_t *ili_init_cmds = heap_caps_malloc(sizeof(ili_init_cmds_drom), MALLOC_CAP_DMA);
  if (!ili_init_cmds) {
    return ESP_ERR_NO_MEM;
  }

  ESP_LOGI(TAG, "copying initialisation comamnds into place");
  memcpy(ili_init_cmds, ili_init_cmds_drom, sizeof(ili_init_cmds_drom));

  int cmd = 0;

  //Initialize non-SPI GPIOs
  ESP_LOGI(TAG, "setting up GPIO pins");
  gpio_set_direction(device->dc_io_num, GPIO_MODE_OUTPUT);
  gpio_set_direction(device->reset_io_num, GPIO_MODE_OUTPUT);

  //Reset the display
  ESP_LOGI(TAG, "resetting display");
  gpio_set_level(device->reset_io_num, 0);
  vTaskDelay(100 / portTICK_RATE_MS);
  gpio_set_level(device->reset_io_num, 1);
  vTaskDelay(100 / portTICK_RATE_MS);
  ESP_LOGI(TAG, "reset done");

  //Send all the commands
  ESP_LOGI(TAG, "sending initialisation commands");
  while (ili_init_cmds[cmd].data_len != 0xff) {
    ESP_LOGI(TAG, "sending command @ index %d (CMD: %d)", cmd, ili_init_cmds[cmd].command);
    ili_command(device, ili_init_cmds[cmd].command);
    ili_data(device, ili_init_cmds[cmd].data, ili_init_cmds[cmd].data_len & 0x1F);
    if (ili_init_cmds[cmd].data_len & 0x80) {
      vTaskDelay(100 / portTICK_RATE_MS);
    }
    cmd ++;
  }

  ESP_LOGI(TAG, "freeing initialisation commands");
  free(ili_init_cmds);

  return ESP_OK;
}

esp_err_t ili_bus_add_device(spi_host_device_t host, ili_device_t* device)
{
  esp_err_t ret;
  spi_device_handle_t spi;

  spi_device_interface_config_t devcfg = {
    .command_bits = 0,
    .address_bits = 0,
    .clock_speed_hz = 32000000,
    .mode = 0,
    .spics_io_num = device->spics_io_num,
    .queue_size = 64,
    .flags = SPI_DEVICE_HALFDUPLEX,
    .pre_cb = ili_spi_pre_transfer_callback,
    .cs_ena_pretrans = 10
  };

  //Attach the LCD to the SPI bus
  ret = spi_bus_add_device(host, &devcfg, &spi);
  if (ret != ESP_OK) {
    return ret;
  }

  device->spi = spi;
  return ret;
}


void display_task()
{
  ESP_LOGI(TAG, "start DisplayTask...");

  spi_bus_config_t bus_cfg = {
      .miso_io_num = VSPI_PIN_NUM_MISO,
      .mosi_io_num = VSPI_PIN_NUM_MOSI,
      .sclk_io_num = VSPI_PIN_NUM_CLK,
      .max_transfer_sz = 50000,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1
  };

  ESP_LOGI(TAG, "initialise SPI bus");
  spi_bus_initialize(HSPI_HOST, &bus_cfg, 1);

  ESP_LOGI(TAG, "initialise ILI device");
  ili_device_t* device = malloc(sizeof(ili_device_t));
  device->dc_io_num = PIN_NUM_DC;
  device->spics_io_num = VSPI_PIN_NUM_CS;
  device->reset_io_num = PIN_NUM_RESET;

  ili_bus_add_device(HSPI_HOST, device);

  ESP_LOGI(TAG, "initialise display");
  ili_init(device);

  // Exit sleep
  ili_command(device, ILI9341_SLPOUT);

  vTaskDelay(1000 / portTICK_RATE_MS);
  ESP_LOGI(TAG, "turning the display on");
  ili_command(device, ILI9341_DISPON);


  vTaskDelay(1000 / portTICK_RATE_MS);
  ESP_LOGI(TAG, "drawing some shit...");

  // Column address set
  ili_command(device, ILI9341_CASET);
  uint8_t caset_d[4] = {
    0,
    0,
    320 >> 8,
    320 & 0xff
  };
  ili_data(device, caset_d, 4);

  // Page address set
  ili_command(device, ILI9341_PASET);
  uint8_t paset_d[4] = {
    0,
    0,
    240 >> 8,
    240 & 0xff
  };
  ili_data(device, paset_d, 4);

  uint8_t pix_d[240 * 2];
  uint16_t col = 0xf800;

  ili_command(device, ILI9341_RAMWR);
  for (uint16_t x = 0; x < 320; x ++) {

    if (x > 160) {
      col = 0x001f;
    }

    for (uint16_t y = 0; y < 240 * 2; y += 2) {
      pix_d[y] = col >> 8;
      pix_d[y + 1] = col & 0xff;
    }

    ili_data(device, pix_d, 240 * 2);
  }



  while (true) ;
}
