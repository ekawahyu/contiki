/*
 * flash-logging.h
 *
 * Created on: June 12, 2017
 *     Author: Brian Blum
 *
 */

#include <stdint.h>

#ifndef FLASH_LOGGING_H_
#define FLASH_LOGGING_H_

// Default start address for cyclical write to Flash (Bank 3)
#define FLASH_LOGGING_START 0x18000
// Default end address for cyclical write to Flash (end Bank 7)
#define FLASH_LOGGING_END   0x38000

/* FLASH LOGGING Globals */
extern uint32_t flash_logging_addr;

// logging component ID's
enum
{
  /* 0x00-0x1F  core */
  
  /* 0x20-0x3F  cpu */
  FLASH_LOGGING_CMP_ID = 0x20,
  
  /* 0x40-0x5F  dev */
  
  /* 0x60-0x7F  platform */
  
  /* 0x80-0x9F  examples */
  FLASH_LOG_CMP_ID = 0x80,
  /* 0xFF invalid */
  NOT_VALID = 0xFF
};

void flashlogging_init(void);
void flashlogging_write8(uint8_t componentId, uint8_t eventId, uint16_t timestamp, uint8_t *data);


#endif /* FLASH_LOGGING_H_ */
