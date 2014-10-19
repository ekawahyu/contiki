/*
 * example-mote.c
 *
 * Created on: May 21, 2014
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

#include "contiki.h"
#include "net/rime/rime.h"
#include "random.h"
#include "sys/clock.h"
#include "dev/serial-line.h"
#include "dev/adc-sensor.h"
#include "dev/gpio.h"
#include "dev/leds.h"
#include "dev/lpm.h"

#include <stdio.h>

#define DEBUG 1

#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else /* DEBUG */
#define PRINTF(...)
#endif /* DEBUG */

static char message[MESSAGE_LEN];
static char button_pressed = 0;
static char command_received = 0;

/*---------------------------------------------------------------------------*/
PROCESS(example_abc_process, "ABC example");
PROCESS(serial_in_process, "ABC example");
/*---------------------------------------------------------------------------*/
#if BUTTON_SENSOR_ON
PROCESS(buttons_process, "Button Process");
AUTOSTART_PROCESSES(&example_abc_process, &serial_in_process, &buttons_process);
#else
AUTOSTART_PROCESSES(&example_abc_process, &serial_in_process);
#endif
/*---------------------------------------------------------------------------*/
static void
abc_recv_cb(struct abc_conn *c)
{
  static clock_time_t curr_time;
  static clock_time_t time_diff;
  static unsigned int repeat;

  memset(message, 0, MESSAGE_LEN);
  memcpy(message, (char *)packetbuf_dataptr(), packetbuf_datalen());
  PRINTF("abc message received '%s'\n", message);

  if (memcmp(message,"fire coils", 10) == 0) {
    PRINTF("Firing now...\n");
    gpio_clear(GPIO2 | GPIO4);
    gpio_set(GPIO1 | GPIO3 | GPIO5);
    repeat = 13;
    while (repeat--) clock_delay_usec(65000);
    gpio_clear(GPIO2 | GPIO4);
    gpio_clear(GPIO1 | GPIO3 | GPIO5);
    PRINTF("Firing done!\n");
  }
  if (memcmp(message,"reload coils", 12) == 0) {
    PRINTF("Reloading now...\n");
    gpio_clear(GPIO1 | GPIO3);
    gpio_set(GPIO2 | GPIO4 | GPIO5);
    repeat = 13;
    while (repeat--) clock_delay_usec(65000);
    gpio_clear(GPIO2 | GPIO4);
    gpio_clear(GPIO1 | GPIO3 | GPIO5);
    PRINTF("Reloading done!\n");
  }
  if (memcmp(message,"get temperature", 15) == 0) {
    command_received = GET_TEMPERATURE;
  }
  if (memcmp(message,"get battery level", 17) == 0) {
    command_received = GET_BATTERY_LEVEL;
  }
}

static void
abc_sent_cb(struct abc_conn *c, int status, int num_tx)
{
  PRINTF("abc message sent\n");
  /* Listening delay for any incoming message */
  clock_delay_usec(10000);
}

static const struct abc_callbacks abc_call = {abc_recv_cb, abc_sent_cb};
static struct abc_conn abc;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_abc_process, ev, data)
{
  static struct etimer et;
  static int counter = 0;
  static rv;
  static const struct sensors_sensor *sensor;
  static float sane = 0;
  static int dec;
  static float frac;

  PROCESS_EXITHANDLER(abc_close(&abc);)

  PROCESS_BEGIN();

  abc_open(&abc, 128, &abc_call);

  while(1) {
    /* Delay ~1 second */
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    sensor = sensors_find(ADC_SENSOR);

    memset(message, 0, MESSAGE_LEN);
    if (sensor && (command_received != NO_COMMAND)) {
      if (command_received == GET_TEMPERATURE) {
      rv = sensor->value(ADC_SENSOR_TYPE_TEMP);
        if(rv != -1) {
          sane = 25 + ((rv - 1480) / 4.5) + 1;
          dec = sane;
          frac = sane - dec;
          sprintf(message, "I am awake:%d.%02u C (%d)", dec, (unsigned int)(frac*100), rv);
        }
      }
      else if (command_received == GET_BATTERY_LEVEL) {
        rv = sensor->value(ADC_SENSOR_TYPE_VDD);
        if(rv != -1) {
          sane = rv * 3 * 1.15 / 2047;
          dec = sane;
          frac = sane - dec;
          sprintf(message, "I am awake:%d.%02u V (%d)", dec, (unsigned int)(frac*100), rv);
        }
      }
      else {
        sprintf(message, "Bad command!");
      }
    }
    else {
      sprintf(message, "I am awake(0x098C):%i", counter++);
    }

    command_received = NO_COMMAND;
    packetbuf_copyfrom(message, strlen(message));
    abc_send(&abc);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
#if BUTTON_SENSOR_ON
PROCESS_THREAD(buttons_process, ev, data)
{
  struct sensors_sensor *sensor;
  static struct etimer et;

  PROCESS_BEGIN();

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event);

    /* If we woke up after a sensor event, inform what happened */
    sensor = (struct sensors_sensor *)data;
    if(sensor == &button_sensor) {
      PRINTF("Button Press\n");
      leds_toggle(LEDS_GREEN);
      button_pressed = 1;
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
    PRINTF("Serial_RX: %s\n", (char*)data);
    if (memcmp(data,"fire coils", 10) == 0) command_received = FIRE_COILS;
    if (memcmp(data,"reload coils", 12) == 0) command_received = RELOAD_COILS;
    if (memcmp(data,"get temperature", 15) == 0) command_received = GET_TEMPERATURE;
    if (memcmp(data,"get battery level", 17) == 0) command_received = GET_BATTERY_LEVEL;
  }

  PROCESS_END();
}
