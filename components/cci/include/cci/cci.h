#ifndef CCI_H
#define CCI_H

// The speed of the I2C bus used for the CCI
#define CCI_I2C_FREQ 100000

// Data & clock pins for the CCI
#define CCI_SDA_PIN 17
#define CCI_SCL_PIN 16

// CCI constants
#define CCI_WORD_LENGTH 0x02
#define CCI_ADDRESS 0x2A

// CCI register locations
#define CCI_REG_STATUS 0x0002
#define CCI_REG_COMMAND 0x0004
#define CCI_REG_DATA_LENGTH 0x0006
#define CCI_REG_DATA_0 0x0008

// Commands
#define CCI_CMD_SYS_RUN_FFC 0x0242
#define CCI_CMD_SYS_GET_FFC_STATE 0x024C
#define CCI_CMD_SYS_GET_UPTIME 0x020C
#define CCI_CMD_SYS_GET_TELEMETRY_ENABLE_STATE 0x0218
#define CCI_CMD_SYS_SET_TELEMETRY_ENABLE_STATE 0x0219
#define CCI_CMD_SYS_GET_TELEMETRY_LOCATION 0x021C
#define CCI_CMD_SYS_SET_TELEMETRY_LOCATION 0x021D
#define CCI_CMD_SYS_GET_FPA_TEMP_K 0x0214
#define CCI_CMD_RAD_GET_RADIOMETRY_ENABLE_STATE 0x0E10
#define CCI_CMD_RAD_SET_RADIOMETRY_ENABLE_STATE 0x0E11
#define CCI_CMD_RAD_GET_RADIOMETRY_TLINEAR_ENABLE_STATE 0x0EC0
#define CCI_CMD_RAD_SET_RADIOMETRY_TLINEAR_ENABLE_STATE 0x0EC1
#define CCI_CMD_AGC_GET_AGC_ENABLE_STATE 0x0100
#define CCI_CMD_AGC_SET_AGC_ENABLE_STATE 0x0101

// Wait timeout parameters for the busy flag in the status register
#define CCI_BUSY_WAIT_AMOUNT 10
#define CCI_BUSY_WAIT_TIMEOUT 1000

// Cycle count delay for software I2C interface
#define I2C_WAIT_CYCLES 1600

// Structures
typedef enum {
  CCI_FFC_ERROR = -1,
  CCI_FFC_NEVER_COMMANDED = 0,
  CCI_FFC_IMMINENT,
  CCI_FFC_IN_PROGRESS,
  CCI_FFC_FINISHED
} cci_ffc_state_t;

// Procedures
void cci_init();
void cci_run_ffc();
cci_ffc_state_t cci_get_ffc_state();
uint16_t cci_get_fpa_temp_k();
uint32_t cci_get_uptime();
uint16_t cci_read_register(uint16_t reg);

#endif
