/**
 * \file
 *         example flash log write and read
 * \author
 *         Brian Blum <brian.blum@conectric.com>
 */

#include "contiki.h"
#include <stdio.h> /* For printf() */
#include "flash.h"
#include "flash-logging.h"

/* Flash Logging */
enum
{
  LOG_EVENT1 = 0x01,    // event description
  LOG_EVENT2 = 0x02,    // event description
  LOG_EVENT3 = 0x03     // event description
};
/*---------------*/

extern volatile uint8_t deep_sleep_requested;

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

  flashlogging_init();
  
  WriteData[0]=0x01;
  WriteData[1]=0x02;
  WriteData[2]=0x03;
  WriteData[3]=0x04;
  WriteData[4]=0x05;
  WriteData[5]=0x06;
  WriteData[6]=0x07;
  WriteData[7]=0x08;

  uint16_t timestamp = 0x0001;
  while (timestamp < 0x00FF)
  {
    flashlogging_write8(FLASH_LOG_CMP_ID, LOG_EVENT2, timestamp, WriteData);
    
    timestamp += 0x11;
    for(int ct=0; ct < 8; ct++)
      WriteData[ct] += 0x10;
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void invoke_process_before_sleep(void)
{
  deep_sleep_requested = 0;
}
/*---------------------------------------------------------------------------*/
