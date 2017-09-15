/**
 * \file
 *         OTA for firmware upgrade management 
 *
 * \author
 *         Original: Brian Blum <brian.blum@conectric.com>
 *
 */

#include "ota.h"
#include "dev/flash.h"

//uint32_t flash_ota_addr = FLASH_OTA_START;

//#define FLASH_WRITE_SIZE_MIN 8

/*---------------------------------------------------------------------------*/
void ota_clear()
{
  for(uint32_t pg = OTA_FLASH_START; pg < OTA_FLASH_END; pg += FLASH_PAGE_SIZE)
    flash_page_erase(FLASH_PAGE(pg));
}

/*---------------------------------------------------------------------------*/
void ota_flashwrite(uint16_t addr, uint8_t size, uint8_t *data)
{
    flash_write_DMA(data, size, FLASH_WORD_ADDR((uint32_t)(OTA_FLASH_START + addr)));
}

/*---------------------------------------------------------------------------*/
void ota_copyandreset()
{
  
}

