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

#include "lib/ringbuf.h"

#define RING_BUF_SIZE 32
#define BUFSIZE 260

static struct ringbuf modbus_rxbuf;
static uint8_t ring_buffer[RING_BUF_SIZE];

PROCESS(modbus_line_process, "MODBUS driver");

process_event_t modbus_line_event_message;

/*---------------------------------------------------------------------------*/
int
modbus_line_input_byte(unsigned char c)
{
  static uint8_t overflow = 0;

  if(!overflow) {
    /* Add character */
    if(ringbuf_put(&modbus_rxbuf, c) == 0) {
      /* Buffer overflow: ignore the rest of the line */
      overflow = 1;
    }
  } else {
    /* Buffer overflowed:
     * Keep trying to add it, otherwise skip */
    if(ringbuf_put(&modbus_rxbuf, c) != 0) {
      overflow = 0;
    }
  }

  /* Wake up consumer process */
  process_poll(&modbus_line_process);
  return 1;
}
/*---------------------------------------------------------------------------*/
void
modbus_line_timeout(void * arg)
{
  /* Wake up consumer process */
  process_poll(&modbus_line_process);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(modbus_line_process, ev, data)
{
  static struct ctimer ct;
  static char buf[BUFSIZE];
  static uint16_t ptr;

  PROCESS_BEGIN();
  
  ctimer_set(&ct, CLOCK_SECOND/8, modbus_line_timeout, &ptr);
  modbus_line_event_message = process_alloc_event();
  ptr = 0;

  while(1) {
    /* Fill application buffer until newline or empty */
    int c = ringbuf_get(&modbus_rxbuf);
    if(c == -1) {
      /* Buffer empty, wait for poll */
      PROCESS_YIELD();
      if (ctimer_expired(&ct) && ptr != 0) {
        /* Terminate */
        buf[0] = ptr;
        buf[++ptr] = (uint8_t)'\0';
        /* Broadcast event */
        process_post(PROCESS_BROADCAST, modbus_line_event_message, buf);
        /* Wait until all processes have handled the serial line event */
        if(PROCESS_ERR_OK ==
          process_post(PROCESS_CURRENT(), PROCESS_EVENT_CONTINUE, NULL)) {
          PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE);
        }
        ptr = 0;
      }
    } else {
      ctimer_restart(&ct);
      if(ptr < BUFSIZE-1) {
        buf[++ptr] = (uint8_t)c;
      } else {
        /* Ignore character */
      }
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
modbus_line_init(void)
{
  ringbuf_init(&modbus_rxbuf, ring_buffer, RING_BUF_SIZE);
  process_start(&modbus_line_process, NULL);
}
