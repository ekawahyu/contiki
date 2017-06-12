/**
 * \file
 *         example flash log write and read
 * \author
 *         Brian Blum <brian.blum@conectric.com>
 */

#include "contiki.h"
#include <stdio.h> /* For printf() */
#include "flash.h"
extern volatile uint8_t deep_sleep_requested;

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
