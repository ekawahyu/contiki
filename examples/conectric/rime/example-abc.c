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
#include "random.h"

#include "dev/button-sensor.h"
#include "dev/sht21/sht21-sensor.h"
#include "dev/adc-sensor.h"
#include "netstack.h"
#include "cc2530-rf.h"

#include "dev/leds.h"

#include <stdio.h>

static char message[40];
extern volatile uint8_t sleep_requested;

/*---------------------------------------------------------------------------*/
PROCESS(example_abc_process, "ABC example");
AUTOSTART_PROCESSES(&example_abc_process);
/*---------------------------------------------------------------------------*/
static void
abc_recv(struct abc_conn *c)
{
  memset(message, 0, strlen(message));
  memcpy(message, (char *)packetbuf_dataptr(), packetbuf_datalen());
  printf("abc message received (%d) '%s'\n", strlen(message), message);
}
static const struct abc_callbacks abc_call = {abc_recv};
static struct abc_conn abc;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_abc_process, ev, data)
{
  static struct etimer et;
  static unsigned int batt, temp, humid;

  PROCESS_EXITHANDLER(abc_close(&abc);)

  PROCESS_BEGIN();
  SENSORS_ACTIVATE(sht21_sensor);

  abc_open(&abc, 128, &abc_call);

  while(1) {

    /*etimer_set(&et, CLOCK_SECOND * 2 + random_rand() % (CLOCK_SECOND * 2));

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    packetbuf_copyfrom("Hello", 6);
    abc_send(&abc);
    printf("abc message sent\n");*/

    PROCESS_WAIT_EVENT();

    NETSTACK_RADIO.off();
    batt = adc_sensor.value(ADC_SENSOR_TYPE_VDD);
    temp = sht21_sensor.value(SHT21_SENSOR_TEMP_ACQ);

    PROCESS_WAIT_EVENT();

    NETSTACK_RADIO.off();
    temp = sht21_sensor.value(SHT21_SENSOR_TEMP_RESULT);

    humid = sht21_sensor.value(SHT21_SENSOR_HUMIDITY_ACQ);

    PROCESS_WAIT_EVENT();

    NETSTACK_RADIO.off();
    humid = sht21_sensor.value(SHT21_SENSOR_HUMIDITY_RESULT);

    memset(message, 0, 40);
    //sprintf(message, "CH=%i\tB=%i\tT=%i\tRH=%i", CC2530_RF_CHANNEL, batt, temp, humid);
    message[0] = 0;
    message[1] = 0;
    message[2] = 0xFF;
    message[3] = 0xFF;
    message[4] = 0x10;
    message[5] = 0xAA;
    message[6] = 0xBB;
    message[7] = 0xDE;
    message[8] = 0xAD;
    message[9] = 0xBE;
    message[10]= 0xEF;

    NETSTACK_RADIO.on();
    packetbuf_copyfrom(message, 11);
    abc_send(&abc);
    //printf("abc message sent\n");
  }

  SENSORS_DEACTIVATE(sht21_sensor);

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void acquired_sensor(void)
{
  process_post_synch(&example_abc_process, PROCESS_EVENT_CONTINUE, NULL);

  sleep_requested = 1;
}
/*---------------------------------------------------------------------------*/
