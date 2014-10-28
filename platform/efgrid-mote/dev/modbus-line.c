/*
 * modbus-line.c
 *
 * Created on: Sep 2, 2014
 *     Author: Ekawahyu Susilo
 *
 * Copyright (c) 2014, Chongqing Aisenke Electronic Technology Co., Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the copyright holder.
 *
 */

#include "dev/modbus-line.h"
#include "dev/leds.h"

#define BUFSIZE 128

static int pos;
static uint8_t modbus_rx_data[BUFSIZE];

PROCESS(modbus_line_process, "MODBUS driver");

process_event_t modbus_line_event_message;

/*---------------------------------------------------------------------------*/
int
modbus_line_input_byte(unsigned char c)
{
  modbus_rx_data[pos++] = c;

  if(pos >= BUFSIZE) {
    pos = 0;
  }

  /* Wake up consumer process */
  process_poll(&modbus_line_process);
  return 1;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(modbus_line_process, ev, data)
{
  PROCESS_BEGIN();

  modbus_line_event_message = process_alloc_event();

  while(1) {

    PROCESS_YIELD();

    process_post(PROCESS_BROADCAST, modbus_line_event_message, modbus_rx_data);

    /* Wait until all processes have handled the modbus line event */
    if(PROCESS_ERR_OK ==
      process_post(PROCESS_CURRENT(), PROCESS_EVENT_CONTINUE, NULL)) {
      PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
modbus_line_init(void)
{
  process_start(&modbus_line_process, NULL);
}
