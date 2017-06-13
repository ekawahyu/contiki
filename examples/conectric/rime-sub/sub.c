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

#include "dev/button-sensor.h"
#include "dev/adc-sensor.h"
#include "dev/rs485-arch.h"
#include "dev/serial-line.h"
#include "dev/modbus-line.h"

#include "dev/uart-arch.h"
#include "debug.h"

#include <stdio.h>

#define BUFSIZE 256
#define TRICKLE_CHANNEL 145

/* Sub Globals */
static uint8_t message[72];
static uint8_t * pmessage = NULL;
extern volatile uint8_t deep_sleep_requested;

static int pos;
static uint8_t submeter_data[BUFSIZE];

/*---------------------------------------------------------------------------*/
PROCESS(sub_process, "Submeter");
PROCESS(serial_in_process, "Serial example");
PROCESS(modbus_in_process, "Modbus example");
#if BUTTON_SENSOR_ON
PROCESS(buttons_test_process, "Button Test Process");
AUTOSTART_PROCESSES(&sub_process, &serial_in_process, &modbus_in_process, &buttons_test_process);
#else
AUTOSTART_PROCESSES(&sub_process, &serial_in_process, &modbus_in_process);
#endif
/*---------------------------------------------------------------------------*/
static void
trickle_recv(struct trickle_conn *c)
{
  printf("%d.%d: trickle message received '%s'\n",
	 linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
	 (char *)packetbuf_dataptr());
}
const static struct trickle_callbacks trickle_call = {trickle_recv};
static struct trickle_conn trickle;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(sub_process, ev, data)
{
  static unsigned int batt;
  static uint8_t counter;
  static float sane;
  static int dec;
  static float frac;
  static uint8_t *sensor_data;
  static struct etimer et;
  static int pos_counter;

  PROCESS_EXITHANDLER(trickle_close(&trickle);)

  PROCESS_BEGIN();

  trickle_open(&trickle, CLOCK_SECOND, TRICKLE_CHANNEL, &trickle_call);

  etimer_set(&et, CLOCK_SECOND);

  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

  memset(message, 0, 72);
  message[0] = 0;
  message[1] = 0;
  message[2] = 0xFF;
  message[3] = 0xFF;
  message[4] = 0x60;
  message[5] = counter++;
  message[6] = clock_reset_cause();

  packetbuf_copyfrom(message, 7);
  trickle_send(&trickle);

  deep_sleep_requested = 0;

  while(1) {

    etimer_set(&et, 10 * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    pos = 0;

    /* Low level RS485 test */
    uart_arch_writeb(0x2F);
    uart_arch_writeb(0x3F);

    uart_arch_writeb(0x30);
    uart_arch_writeb(0x30);
    uart_arch_writeb(0x30);
    uart_arch_writeb(0x30);
    uart_arch_writeb(0x30);
    uart_arch_writeb(0x30);
    uart_arch_writeb(0x30);
    uart_arch_writeb(0x31);
    uart_arch_writeb(0x35);
    uart_arch_writeb(0x36);
    uart_arch_writeb(0x38);
    uart_arch_writeb(0x39);

    /*uart_arch_writeb(0x30);
    uart_arch_writeb(0x30);
    uart_arch_writeb(0x30);
    uart_arch_writeb(0x33);
    uart_arch_writeb(0x30);
    uart_arch_writeb(0x30);
    uart_arch_writeb(0x30);
    uart_arch_writeb(0x30);
    uart_arch_writeb(0x32);
    uart_arch_writeb(0x39);
    uart_arch_writeb(0x33);
    uart_arch_writeb(0x32);

    uart_arch_writeb(0x30);
    uart_arch_writeb(0x30);*/

    uart_arch_writeb(0x21);
    uart_arch_writeb(0x0D);
    uart_arch_writeb(0x0A);

    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    batt = adc_sensor.value(ADC_SENSOR_TYPE_VDD);
    sane = batt * 3 * 1.15 / 2047;
    dec = sane;
    frac = sane - dec;
    sensor_data = (uint8_t*)data;

    /* 1st chunk */
    pmessage = message;
    memset(message, 0, 72);
    *pmessage++ = 0;
    *pmessage++ = 0;
    *pmessage++ = 0xFF;
    *pmessage++ = 0xFF;
    *pmessage++ = 0x80;
    *pmessage++ = counter++;
    *pmessage++ = (char)(dec*10)+(char)(frac*10);
    *pmessage++ = 0x81;

    for (pos_counter=0; pos_counter<64; pos_counter++)
      *pmessage++ = submeter_data[pos_counter];

    packetbuf_copyfrom(message, 72);
    trickle_send(&trickle);

    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    /* 2nd chunk */
    pmessage = message;
    memset(message, 0, 72);
    *pmessage++ = 0;
    *pmessage++ = 0;
    *pmessage++ = 0xFF;
    *pmessage++ = 0xFF;
    *pmessage++ = 0x80;
    *pmessage++ = counter++;
    *pmessage++ = (char)(dec*10)+(char)(frac*10);
    *pmessage++ = 0x82;

    for (pos_counter=64; pos_counter<128; pos_counter++)
      *pmessage++ = submeter_data[pos_counter];

    packetbuf_copyfrom(message, 72);
    trickle_send(&trickle);

    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    /* 3rd chunk */
    pmessage = message;
    memset(message, 0, 72);
    *pmessage++ = 0;
    *pmessage++ = 0;
    *pmessage++ = 0xFF;
    *pmessage++ = 0xFF;
    *pmessage++ = 0x80;
    *pmessage++ = counter++;
    *pmessage++ = (char)(dec*10)+(char)(frac*10);
    *pmessage++ = 0x83;

    for (pos_counter=128; pos_counter<192; pos_counter++)
      *pmessage++ = submeter_data[pos_counter];

    packetbuf_copyfrom(message, 72);
    trickle_send(&trickle);

    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    /* 4th chunk */
    pmessage = message;
    memset(message, 0, 72);
    *pmessage++ = 0;
    *pmessage++ = 0;
    *pmessage++ = 0xFF;
    *pmessage++ = 0xFF;
    *pmessage++ = 0x80;
    *pmessage++ = counter++;
    *pmessage++ = (char)(dec*10)+(char)(frac*10);
    *pmessage++ = 0x84;

    for (pos_counter=192; pos_counter<256; pos_counter++)
      *pmessage++ = submeter_data[pos_counter];

    packetbuf_copyfrom(message, 72);
    trickle_send(&trickle);

    for (pos_counter=0; pos_counter<BUFSIZE; pos_counter++)
      puthex(submeter_data[pos_counter]);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
#if BUTTON_SENSOR_ON
PROCESS_THREAD(buttons_test_process, ev, data)
{
  struct sensors_sensor *sensor;
  static uint8_t counter;
  static uint8_t button;

  PROCESS_BEGIN();

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event);

    sensor = (struct sensors_sensor *)data;
    if(sensor == &button_1_sensor) {
      button = 0x71;
      process_post(&sub_process, PROCESS_EVENT_CONTINUE, &button);
    }
    if(sensor == &button_2_sensor) {
      button = 0x72;
      process_post(&sub_process, PROCESS_EVENT_CONTINUE, &button);
    }
  }

  PROCESS_END();
}
#endif
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
PROCESS_THREAD(modbus_in_process, ev, data)
{
  PROCESS_BEGIN();

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == modbus_line_event_message && data != NULL);
    //printf("Modbus_RX: %s\n", (char*)data);
    //puthex(*(unsigned char*)data & 0x7F);
    submeter_data[pos++] = *(unsigned char*)data & 0x7F;

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void invoke_process_before_sleep(void)
{
  deep_sleep_requested = 0;
}
/*---------------------------------------------------------------------------*/
