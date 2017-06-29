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

#include "dev/button-sensor.h"
#include "dev/sht21/sht21-sensor.h"
#include "dev/adc-sensor.h"

#include <stdio.h>

static uint8_t message[40];
extern volatile uint16_t deep_sleep_requested;

/* Flash Logging */
static uint8_t logData[4]= { 0x00, 0x00, 0x00, 0x00};

#define LOGGING_REF_TIME_PD ((clock_time_t)(12 * CLOCK_SECOND * 60 * 60))
enum
{
  OC_RESERVED = 0x00,    // reserved
  OC_SEND     = 0x01,    // send data event 
};

/*---------------------------------------------------------------------------*/
PROCESS(pir_abc_process, "PIR Sensor");
PROCESS(flash_log_process, "Flash Log process");
#if BUTTON_SENSOR_ON
PROCESS(buttons_test_process, "Button Test Process");
AUTOSTART_PROCESSES(&pir_abc_process, &buttons_test_process, &flash_log_process);
#else
AUTOSTART_PROCESSES(&pir_abc_process, &flash_log_process);
#endif
/*---------------------------------------------------------------------------*/
static void
abc_recv(struct abc_conn *c)
{
  memset(message, 0, sizeof(message));
  memcpy(message, (char *)packetbuf_dataptr(), packetbuf_datalen());
  printf("abc message received (%d) '%s'\n", strlen(message), message);
}
static const struct abc_callbacks abc_call = {abc_recv};
static struct abc_conn abc;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(pir_abc_process, ev, data)
{
  static unsigned int batt;
  static uint8_t counter;
  static float sane;
  static int dec;
  static float frac;
  static uint8_t *sensor_data;
  static struct etimer et;
  static uint8_t loop;

  PROCESS_EXITHANDLER(abc_close(&abc);)

  PROCESS_BEGIN();

  abc_open(&abc, 128, &abc_call);

  etimer_set(&et, CLOCK_SECOND);

  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

  memset(message, 0, 40);
  message[0] = 0;
  message[1] = 0;
  message[2] = 0xFF;
  message[3] = 0xFF;
  message[4] = 0x60;
  message[5] = counter++;
  message[6] = clock_reset_cause();

  loop = CONECTRIC_BURST_NUMBER;

  while(loop--) {
    packetbuf_copyfrom(message, 7);
    NETSTACK_MAC.on();
    abc_send(&abc);

    PROCESS_WAIT_EVENT();

    if (loop)
      deep_sleep_requested = 1 + random_rand() % (CLOCK_SECOND / 8);
    else
      deep_sleep_requested = CLOCK_SECOND;
  }

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE && data != NULL);

    batt = adc_sensor.value(ADC_SENSOR_TYPE_VDD);
    sane = batt * 3 * 1.15 / 2047;
    dec = sane;
    frac = sane - dec;
    sensor_data = (uint8_t*)data;

    memset(message, 0, 40);
    message[0] = 0;
    message[1] = 0;
    message[2] = 0xFF;
    message[3] = 0xFF;
    message[4] = 0x50;
    message[5] = counter++;
    message[6] = (char)(dec*10)+(char)(frac*10);
    message[7] = *sensor_data;

    // Log data that will be sent out over the air
    logData[0] = (char)(dec*10)+(char)(frac*10);
    logData[1] = *sensor_data;
    logData[2] = 0x00;
    logData[3] = 0x00;
    flashlogging_write4(RIME_OC_CMP_ID, OC_SEND, logData);  
    
    loop = CONECTRIC_BURST_NUMBER;

    while(loop--) {
      packetbuf_copyfrom(message, 8);
      NETSTACK_MAC.on();
      abc_send(&abc);

      PROCESS_WAIT_EVENT();

      if (loop)
        deep_sleep_requested = 1 + random_rand() % (CLOCK_SECOND / 8);
      else
        deep_sleep_requested = 10 * CLOCK_SECOND;
    }

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
#if BUTTON_SENSOR_ON
PROCESS_THREAD(buttons_test_process, ev, data)
{
  struct sensors_sensor *sensor;
  static struct etimer et;
  static uint8_t counter;
  static uint8_t button;

  PROCESS_BEGIN();

  etimer_set(&et, CLOCK_SECOND * 10);

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event);

    if (etimer_expired(&et)) {

      sensor = (struct sensors_sensor *)data;
      if(sensor == &button_1_sensor) {
        button = 0x51;
        process_post(&pir_abc_process, PROCESS_EVENT_CONTINUE, &button);
      }
      if(sensor == &button_2_sensor) {
        button = 0x52;
        process_post(&pir_abc_process, PROCESS_EVENT_CONTINUE, &button);
      }
      etimer_restart(&et);
    }
    else {
      etimer_restart(&et);
    }
  }

  PROCESS_END();
}
#endif
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(flash_log_process, ev, data)
{
  static struct etimer et;
  
  PROCESS_BEGIN();

  flashlogging_init();
  
  while (1)
  {
    etimer_set(&et, LOGGING_REF_TIME_PD);  
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    
    flashlogging_write_fullclock(FLASH_LOGGING_CMP_ID, 0);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
invoke_process_before_sleep(void)
{
  process_post_synch(&pir_abc_process, PROCESS_EVENT_CONTINUE, NULL);

  /* TODO can we try to sleep at close to the max sleep timer?
   * like 500 * CLOCK_SECOND
   */
  if (deep_sleep_requested == 0) deep_sleep_requested = 60 * CLOCK_SECOND;
}
/*---------------------------------------------------------------------------*/
