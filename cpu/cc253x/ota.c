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

uint8_t img_segment_map[OTA_SEGMAP_BYTES];

void ota_init()
{
  for(int x=0; x < OTA_SEGMAP_BYTES; x++)
    img_segment_map[x] = 0x00;
}

// update segment map for address
void ota_update_map(uint16_t addr)
{
  uint8_t bit = (addr >> OTA_UPDATE_MAP_SHIFT);
  uint8_t index_byte = bit / 8;
  uint8_t index_bit = bit % 8;
  img_segment_map[index_byte] |= (1 << index_bit);
}

// restore the segment map on reset (if <partial> new image is in OTA Flash)
void ota_restore_map()
{
  uint32_t data;
  for(uint32_t seg = OTA_FLASH_START; seg < OTA_FLASH_END; seg += OTA_SEGMENT_MAX)
  {
    flash_read(FLASH_PAGE(seg), FLASH_PAGE_OFFSET(seg), (uint8_t *)&data, sizeof(data));
    if(data != 0xFFFFFFFF)
      ota_update_map((uint16_t)(seg - OTA_FLASH_START));
  }  
}

/*---------------------------------------------------------------------------*/
void ota_clear()
{
  for(uint32_t pg = OTA_FLASH_START; pg < OTA_FLASH_END; pg += FLASH_PAGE_SIZE)
    flash_page_erase(FLASH_PAGE(pg));
  ota_init();
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

