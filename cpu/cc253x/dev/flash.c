/**
 * \file
 *         Driver for cc2530 Flash Read/Write. 
 *
 * \author
 *         Original: Brian Blum <brian.blum@conectric.com>
 *         Port: Zach Shelby <zach@sensinode.com>
 *         Further Modifications:
 *               George Oikonomou <oikonomou@users.sourceforge.net>
 *
 */

#include "flash.h"
#include "contiki.h"
#include "cc253x.h"

// Flash Controller Map
typedef struct {
     uint8_t SRCADDRH; //Byte 0
     uint8_t SRCADDRL; //Byte 1
     uint8_t DESTADDRH; //Byte 2
     uint8_t DESTADDRL; //Byte 3
     uint8_t LENH:5; //Byte 4 - Bit 4:0
     uint8_t VLEN:3; //Byte 4 - Bit 7:5
     uint8_t LENL; //Byte 5
     uint8_t TRIG:5; //Byte 6 - Bit 4:0
     uint8_t TMODE:2; //Byte 6 - Bit 6:5
     uint8_t WORDSIZE:1; //Byte 6 - Bit 7
     uint8_t PRIORITY:2; //Byte 7 - Bit 1:0
     uint8_t M8:1; //Byte 7 - Bit 2
     uint8_t IRQMASK:1; //Byte 7 - Bit 3
     uint8_t DESTINC:2; //Byte 7 - Bit 5:4
     uint8_t SRCINC:2; //Byte 7 - Bit 7:6
} DMA_DESC;

/**************************************************************************************************
 * @fn          flash_write_DMA
 *
 * @brief       This function writes 'cnt' bytes to the internal flash.
 *
 * input parameters
 *
 * @param       data - Valid buffer space at least as big as 'cnt' X 4.
 * @param       length - Number of 4-byte blocks to write.
 * @param       flashwordaddr - Valid flash write address: actual addr / 4 and quad-aligned.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */

void flash_write_DMA(uint8_t *data, uint16_t length, uint16_t flashwordadr)
{
       DMA_DESC dmaConfig0;
       dmaConfig0.SRCADDRH  = ((uint16_t)data >> 8) & 0x00FF;
       dmaConfig0.SRCADDRL  = (uint16_t)data & 0x00FF;
       dmaConfig0.DESTADDRH = (((uint16_t)&FWDATA) >> 8) & 0x00FF;
       dmaConfig0.DESTADDRL = ((uint16_t)&FWDATA) & 0x00FF;
       dmaConfig0.VLEN      = 0;
       dmaConfig0.LENH      = (length>>8) & 0x00FF;
       dmaConfig0.LENL      = length & 0x00FF;
       dmaConfig0.WORDSIZE  = 0;
       dmaConfig0.TMODE     = 0;
       dmaConfig0.TRIG      = 18;
       dmaConfig0.SRCINC    = 1;
       dmaConfig0.DESTINC   = 0;
       dmaConfig0.IRQMASK   = 0;
       dmaConfig0.M8        = 0;
       dmaConfig0.PRIORITY  = 2;
       while (FCTL & 0x80);
       FADDRH =(flashwordadr >> 8) & 0x00FF;
       FADDRL = (flashwordadr) & 0x00FF;
 
       DMA0CFGH = (((uint16_t)&dmaConfig0) >> 8) & 0x00FF;
       DMA0CFGL = ((uint16_t)&dmaConfig0) & 0x00FF;
       
       DMAARM |= 0x01;
       FCTL |= 0x02;
       while (!(DMAIRQ & 0x01));
       DMAIRQ = 0xFE;
       while (FCTL & (0x80));
}

/**************************************************************************************************
 * @fn          flash_read
 *
 * @brief       This function reads 'size' bytes from the internal flash.
 *
 * input parameters
 *
 * @param       pg - A valid flash page number.
 * @param       offset - A valid offset into the page.
 * @param       buf - A valid buffer space at least as big as the 'cnt' parameter.
 * @param       size - A valid number of bytes to read.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void flash_read(uint8_t pg, uint16_t offset, uint8_t *buf, uint16_t size)
{
  // Calculate the offset into the containing flash bank as it gets mapped into XDATA.
  uint8_t *pData = (uint8_t *)(offset + FLASH_PAGE_MAP) + ((pg % FLASH_BANK_PAGES) * FLASH_PAGE_SIZE);
  
  // Save to restore.
  uint8_t memctr = MEMCTR;  

  // Calculate the flash bank from the flash page.
  pg /= FLASH_BANK_PAGES;  

  // Calculate and map the containing flash bank into XDATA.
  MEMCTR = (MEMCTR & 0xF8) | pg;

  while (size--)
  {
    *buf++ = *pData++;
  }

  MEMCTR = memctr;
}

/**************************************************************************************************
 * @fn          flash_page_erase
 *
 * @brief       This function erases the specified page of the internal flash.
 *
 * input parameters
 *
 * @param       pg - A valid flash page number to erase.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */

void flash_page_erase(uint8_t pg)
{
 uint8_t test;
 EA = 0; // disable interrupt
 while(FCTL & 0x80);
 FADDRH = pg << 1;
 FADDRL = 0x00;
 FCTL |= 0x01;

 test = FCTL;
 while(FCTL & 0x80);
 EA = 1; // enable interrupt
 test=0;
}

