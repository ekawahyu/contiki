/**
 * \file
 *         example flash log write and read
 * \author
 *         Brian Blum <brian.blum@conectric.com>
 */

#include "contiki.h"
#include "flash-logging.h"

#include <stdio.h>

extern volatile uint16_t deep_sleep_requested;

/*---------------------------------------------------------------------------*/
PROCESS(flash_log_process, "Flash Log process");
AUTOSTART_PROCESSES(&flash_log_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(flash_log_process, ev, data)
{
  static struct etimer et;
  
  PROCESS_BEGIN();

  flashlogging_init();
  
  while (1)
  {
    etimer_set(&et, 1 * CLOCK_SECOND * 60);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    
    flashlogging_write_fullclock(FLASH_LOGGING_CMP_ID, LOGGING_REF);
  }

  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
void
invoke_process_before_sleep(void)
{
  deep_sleep_requested = 0;
}
/*---------------------------------------------------------------------------*/
