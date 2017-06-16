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
// Default end address for cyclical write to Flash (beginning last Page Bank 8)
#define FLASH_LOGGING_END   0x3F800

/* FLASH LOGGING Globals */
enum
{
  LOGGING_RES  = 0x00,    // reserved
  LOGGING_INIT = 0x01,    // initialization
  LOGGING_REF  = 0x02,    // reference timestamp
};

// logging component ID's
enum
{
  /* 0x00-0x1F  core */
  
  /* 0x20-0x3F  cpu */
  FLASH_LOGGING_CMP_ID = 0x20,
  
  /* 0x40-0x5F  dev */
  
  /* 0x60-0x7F  platform */
  
  /* 0x80-0x9F  examples */
  RIME_SUB_CMP_ID = 0x80,
  RIME_RHT_CMP_ID = 0x81,
  RIME_OC_CMP_ID = 0x82,
  RIME_SW_CMP_ID = 0x83,
  RIME_ROUTER_CMP_ID = 0x83,
  
  /* 0xFF invalid */
  NOT_VALID = 0xFF
};

void flashlogging_init();
void flashlogging_write4(uint8_t componentId, uint8_t eventId, uint8_t *data);
void flashlogging_write_fullclock(uint8_t componentId, uint8_t eventId);

#endif /* FLASH_LOGGING_H_ */
