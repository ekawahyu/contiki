/**
 * \file
 *         Basic hello world example
 * \author
 *         Zach Shelby <zach@sensinode.com>
 */

#include "contiki.h"
#include <stdio.h> /* For printf() */
extern volatile uint8_t deep_sleep_requested;
/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "Hello world process");
AUTOSTART_PROCESSES(&hello_world_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hello_world_process, ev, data)
{

  PROCESS_BEGIN();

  printf("Hello World!\n");

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void invoke_process_before_sleep(void)
{
  deep_sleep_requested = 1;
}
/*---------------------------------------------------------------------------*/
