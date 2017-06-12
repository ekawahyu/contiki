/**
 * \file
 *         example flash log write and read
 * \author
 *         Brian Blum <brian.blum@conectric.com>
 */

#include "contiki.h"
#include <stdio.h> /* For printf() */
extern volatile uint8_t deep_sleep_requested;

/* FLASH Globals */
/*---------------*/
// CODE banks get mapped into the XDATA range 8000-FFFF.
#define FLASH_PAGE_MAP         0x8000
#define FLASH_WORD_ADDR(addr) (addr >> 2)
#define FLASH_PAGE_SIZE 2048
#define FLASH_BANK_PAGES 16
#define ADDR_PAGE(addr) (addr / FLASH_PAGE_SIZE)
#define ADDR_OFFSET(addr) (addr % FLASH_PAGE_SIZE)


// Log Test Globals

#define Log_Addr 0x18000

uint8_t WriteData[8]=
{
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

uint8_t ReadData[8]=
{
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

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

void Flash_WriteDMA(uint8_t *data, uint16_t length, uint16_t flashwordadr)
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
 * @fn          Flash_Read
 *
 * @brief       This function reads 'cnt' bytes from the internal flash.
 *
 * input parameters
 *
 * @param       pg - A valid flash page number.
 * @param       offset - A valid offset into the page.
 * @param       buf - A valid buffer space at least as big as the 'cnt' parameter.
 * @param       cnt - A valid number of bytes to read.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void Flash_Read(uint8_t pg, uint16_t offset, uint8_t *buf, uint16_t size)
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

void Flash_PageErase(uint8_t byPage)
{
 uint8_t test;
 EA = 0; // disable interrupt
 while(FCTL & 0x80);
 FADDRH = byPage << 1;
 FADDRL = 0x00;
 FCTL |= 0x01;

 test = FCTL;
 while(FCTL & 0x80);
 EA = 1; // enable interrupt
 test=0;
}
/*---------------*/

/*---------------------------------------------------------------------------*/
PROCESS(flash_log_process, "Flash Log process");
AUTOSTART_PROCESSES(&flash_log_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(flash_log_process, ev, data)
{

  PROCESS_BEGIN();

  printf("erasing flash\n");
  
  Flash_PageErase(ADDR_PAGE(Log_Addr));
  
  printf("Writing to Flash!\n");
  
  WriteData[0]=0x01;
  WriteData[1]=0x02;
  WriteData[2]=0x03;
  WriteData[3]=0x04;
  WriteData[4]=0x05;
  WriteData[5]=0x06;
  WriteData[6]=0x07;
  WriteData[7]=0x08;

  Flash_WriteDMA(WriteData,8, FLASH_WORD_ADDR(Log_Addr));

  Flash_Read(ADDR_PAGE(Log_Addr), ADDR_OFFSET(Log_Addr), ReadData, 8);
  
  volatile uint8_t test3 = ReadData[2];

  Flash_PageErase(ADDR_PAGE(Log_Addr));

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void invoke_process_before_sleep(void)
{
  deep_sleep_requested = 0;
}
/*---------------------------------------------------------------------------*/
