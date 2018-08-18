/*
 * flash-state.h
 *
 * Created on: August 16, 2017
 *     Author: Brian Blum
 *
 */

#include <stdint.h>

#ifndef FLASH_STATE_H_
#define FLASH_STATE_H_

// Default start address for cyclical write to Flash (page 123 and 124)
#ifndef FLASH_STATE_PG1_ADDR
#define FLASH_STATE_PG1_ADDR 0x3D800
#endif
#ifndef FLASH_STATE_PG2_ADDR
#define FLASH_STATE_PG2_ADDR 0x3E000
#endif

#define FLASH_WRITE_SIZE_MIN    4
#define FLASH_READ_HEADER_SIZE  4
#define FLASH_STATE_SIZE_MAX    64

extern uint8_t read_buf[FLASH_STATE_SIZE_MAX];
// state ID's (ensure no duplicates)
enum
{
  FLASH_STATE_PG_HEADER =       0x01,
  FLASH_STATE_WI_SENSOR_LIST =  0x02,
  FLASH_STATE_OTA_IMG_VERSION = 0x03,
  FLASH_STATE_OTA_IMG_SIZE =    0x04
};

void flashstate_init();
void flashstate_write(uint8_t stateId, uint8_t *data, uint8_t size);
// Read state from Flash
// Return size of state information
uint8_t flashstate_read(uint8_t stateId, uint8_t **data);

#endif /* FLASH_STATE_H_ */
