/*
 * example-dongle.c
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
#include "dev/modbus-line.h"
#include "dev/button-sensor.h"
#include "dev/leds.h"
#include "dev/lpm.h"

#include <stdio.h>

#define DEBUG 1

#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else /* DEBUG */
#define PRINTF(...)
#endif /* DEBUG */

/* TODO workaround to prevent sleep while expecting a reply */
unsigned char app_busy = 0;

static char message[MESSAGE_LEN];
static char button_pressed = 0;
static char command_received = 0;

/*---------------------------------------------------------------------------*/
PROCESS(example_abc_process, "ABC example");
PROCESS(serial_in_process, "Serial example");
PROCESS(modbus_in_process, "Modbus example");
/*---------------------------------------------------------------------------*/
#if BUTTON_SENSOR_ON
PROCESS(buttons_process, "Button Process");
AUTOSTART_PROCESSES(&example_abc_process, &serial_in_process, &modbus_in_process, &buttons_process);
#else
AUTOSTART_PROCESSES(&example_abc_process, &serial_in_process, &modbus_in_process);
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
  PRINTF("abc message received '%s' (%i dBm)\n", message, packetbuf_attr(PACKETBUF_ATTR_RSSI));

  if (memcmp(message,"I am awake", 10) == 0) {
    time_diff = clock_time() - curr_time;
    curr_time = clock_time();
    process_post_synch(&example_abc_process, PROCESS_EVENT_CONTINUE, &time_diff);
  }
}

static void
abc_sent_cb(struct abc_conn *c, int status, int num_tx)
{
  PRINTF("abc message sent\n");
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
    /* example of direct response to a sleepy mote */
    PROCESS_WAIT_EVENT_UNTIL(ev==PROCESS_EVENT_CONTINUE);

    memset(message, 0, MESSAGE_LEN);

    if (command_received == GET_TEMPERATURE) sprintf(message, "get temperature:%i", 1);
    if (command_received == GET_BATTERY_LEVEL) sprintf(message, "get battery level:%i", 1);

    packetbuf_copyfrom(message, strlen(message));

    if (button_pressed || command_received) {
      button_pressed = 0;
      command_received = 0;
      abc_send(&abc);
    }
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
    if (memcmp(data,"get temperature", 15) == 0) command_received = GET_TEMPERATURE;
    if (memcmp(data,"get battery level", 17) == 0) command_received = GET_BATTERY_LEVEL;
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(modbus_in_process, ev, data)
{
  PROCESS_BEGIN();

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == modbus_line_event_message && data != NULL);
    PRINTF("Modbus_RX: %s\n", (char*)data);
  }

  PROCESS_END();
}

