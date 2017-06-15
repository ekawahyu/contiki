/**
 * \file
 *         Flash Logger for managing logging of debug data. 
 *
 * \author
 *         Original: Brian Blum <brian.blum@conectric.com>
 *
 */

#include "flash-logging.h"
#include "dev/flash.h"
#include "clock.h"

uint32_t flash_logging_addr = FLASH_LOGGING_START;
// scratch pad for writes setup for start seq
uint8_t flash_logging_scratch[8] = { 0x00, 0x00, 0x00, 0x00, 0xAA, 0xBB, 0xCC, 0xDD };

#define FLASH_WRITE_SIZE_MIN 8

void flashlogging_init()
{
  uint8_t read_buf[FLASH_WRITE_SIZE_MIN];  // read by minimum write size
  flash_logging_addr = FLASH_LOGGING_START - FLASH_WRITE_SIZE_MIN;
  clock_time_t global_time = clock_seconds();
  
  // loop until you find the first unused memory (no writes begin with 0xFF)
  do {
    flash_logging_addr += FLASH_WRITE_SIZE_MIN;
    flash_read(FLASH_PAGE(flash_logging_addr), FLASH_PAGE_OFFSET(flash_logging_addr), read_buf, FLASH_WRITE_SIZE_MIN);
  } while (read_buf[0] != 0xFF && flash_logging_addr <= FLASH_LOGGING_END);
    
  // catch exceptional case where device reset with write area completely full
  flash_logging_addr = (flash_logging_addr < FLASH_LOGGING_END) ? flash_logging_addr : FLASH_LOGGING_START; 

  // place "boot" sequence in flash at start of write block
  memcpy(&flash_logging_scratch[4], &global_time, 4);
  flashlogging_write4(FLASH_LOGGING_CMP_ID, LOGGING_INIT, &flash_logging_scratch[4]);
}

void flashlogging_write4(uint8_t componentId, uint8_t eventId, uint8_t *data)
{
  clock_time_t global_time = clock_seconds();
    
  // if we arrived on a page boundary, erase the page before writing
  if(FLASH_PAGE_OFFSET(flash_logging_addr) == 0)
    flash_page_erase(FLASH_PAGE(flash_logging_addr));
  
  // put data for write into scratch buffer
  flash_logging_scratch[0] = componentId;
  flash_logging_scratch[1] = eventId;
  flash_logging_scratch[2] = (uint8_t)(global_time >> 8 & 0x000000FF);
  flash_logging_scratch[3] = (uint8_t)(global_time & 0x000000FF);
  
  if(data)
    memcpy(&flash_logging_scratch[4], data, 4);
           
  flash_write_DMA(flash_logging_scratch, 8, FLASH_WORD_ADDR(flash_logging_addr));
  
  flash_logging_addr += 8;
}