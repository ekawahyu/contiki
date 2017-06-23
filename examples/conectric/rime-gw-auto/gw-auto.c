/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         Testing the abc layer in Rime
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "net/rime/rime.h"
#include "net/netstack.h"
#include "random.h"
#include "flash-logging.h"

#include "dev/serial-line.h"

#include "dev/uart-arch.h"
#include "debug.h"

#include <stdio.h>


// Device
extern volatile uint16_t deep_sleep_requested;
static struct etimer et;
// BMB - move this to .h
#define SN_SIZE 12
static uint8_t gw_data[SN_SIZE] = {0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x31, 0x35, 0x36, 0x38, 0x39};

// BMB - move this to .h for messages
enum {
     GW_MSG_TYPE_BOOT = 0x01,
     GW_MSG_TYPE_SN = 0x80
};

// Messaging 
static uint8_t message[72];
static uint8_t pkt_counter = 0;

// Logging
static uint8_t logData[4]= { 0x00, 0x00, 0x00, 0x00};

/* Flash Logging */
#define CLOCK_HR_MULT ((clock_time_t)(60 * 60))  
enum
{
  SUB_RESERVED = 0x00,    // reserved
  SUB_SEND     = 0x01,    // send data event 
};

/*---------------------------------------------------------------------------*/
PROCESS(gw_process, "Submeter");
PROCESS(serial_in_process, "Serial example");
PROCESS(flash_log_process, "Flash Log process");

AUTOSTART_PROCESSES(&gw_process, &serial_in_process, &flash_log_process);

/*---------------------------------------------------------------------------*/

// gw send S/N query to sub periodically 
static void gw_send(struct trickle_conn *c, uint8_t type, uint8_t counter, uint8_t *tx_data, uint8_t size)
{
  uint8_t * pmessage = message;
  uint8_t cnt;
  
  /* 1st chunk */
  pmessage = message;
  memset(message, 0, size+8);
  *pmessage++ = 0;
  *pmessage++ = 0;
  *pmessage++ = 0xFF;
  *pmessage++ = 0xFF;
  *pmessage++ = type;
  *pmessage++ = counter;

  for (cnt=0; cnt<size; cnt++)
    *pmessage++ = tx_data[cnt];

  packetbuf_copyfrom(message, size+6);
  trickle_send(c);
}

static void
trickle_recv(struct trickle_conn *c)
{
  // when receiving trickle message pass to serial port
  
  printf("%d.%d: trickle message received '%s'\n",
	 linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
	 (char *)packetbuf_dataptr());
  printf("/n");
}

/*--------------------------------------------------------------------------*/
const static struct trickle_callbacks trickle_call = {trickle_recv};
static struct trickle_conn trickle;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(gw_process, ev, data)
{
  uint8_t boot_reason;
  
  PROCESS_EXITHANDLER(trickle_close(&trickle);)

  PROCESS_BEGIN();

  trickle_open(&trickle, CLOCK_SECOND, 128, &trickle_call);

  etimer_set(&et, CLOCK_SECOND);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

  boot_reason = clock_reset_cause();

  gw_send(&trickle, GW_MSG_TYPE_BOOT, pkt_counter++, &boot_reason, 1);
    
  deep_sleep_requested = 0;

  while(1) {

    etimer_set(&et, 10 * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    gw_send(&trickle, GW_MSG_TYPE_SN, pkt_counter++, gw_data, SN_SIZE);
  }
  
  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(serial_in_process, ev, data)
{
  PROCESS_BEGIN();

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message && data != NULL);
    printf("Serial_RX: %s\n", (char*)data);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(flash_log_process, ev, data)
{
  static struct etimer ft;
  
  PROCESS_BEGIN();

  flashlogging_init();
  
  while (1)
  {
    etimer_set(&ft, 12 * CLOCK_HR_MULT * (CLOCK_SECOND));  
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&ft));
    
    flashlogging_write_fullclock(FLASH_LOGGING_CMP_ID, 0);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void invoke_process_before_sleep(void)
{
  deep_sleep_requested = 0;
}
/*---------------------------------------------------------------------------*/
