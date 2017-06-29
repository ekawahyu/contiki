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
#include "dev/adc-sensor.h"
#include "dev/rs485-arch.h"
#include "dev/serial-line.h"
#include "dev/modbus-line.h"

#include "dev/uart-arch.h"
#include "debug.h"

#include <stdio.h>


// Device
extern volatile uint16_t deep_sleep_requested;
static struct etimer et;
// BMB - move this to .h
#define SN_SIZE 12
static uint8_t sn_data[SN_SIZE];

// Messaging 
static uint8_t message[72];
static uint8_t pkt_counter = 0;

// BMB - move this to .h for messages
enum {
     SUB_MSG_TYPE_BOOT = 0x01,
     SUB_MSG_TYPE_EKM_DATA1 = 0x10,
     SUB_MSG_TYPE_EKM_DATA2 = 0x11,
     SUB_MSG_TYPE_EKM_DATA3 = 0x12,
     SUB_MSG_TYPE_EKM_DATA4 = 0x13,
     GW_MSG_TYPE_SN = 0x80
};

// Logging
static uint8_t logData[4]= { 0x00, 0x00, 0x00, 0x00};

// EKM 
#define BUFSIZE 256
static uint16_t ekm_in_pos;
static uint8_t submeter_data[BUFSIZE];

/* Flash Logging */
#define CLOCK_HR_MULT ((clock_time_t)(60 * 60))  
enum
{
  SUB_RESERVED = 0x00,    // reserved
  SUB_SEND     = 0x01,    // send data event 
};

/*---------------------------------------------------------------------------*/
PROCESS(sub_process, "Submeter");
PROCESS(serial_in_process, "Serial example");
PROCESS(modbus_in_process, "Modbus example");
PROCESS(modbus_out_process, "Modbus write");
PROCESS(flash_log_process, "Flash Log process");
#if BUTTON_SENSOR_ON
PROCESS(buttons_test_process, "Button Test Process");
AUTOSTART_PROCESSES(&sub_process, &serial_in_process, &modbus_in_process, &modbus_out_process, &buttons_test_process, &flash_log_process);
#else
AUTOSTART_PROCESSES(&sub_process, &serial_in_process, &modbus_in_process, &modbus_out_process, &flash_log_process);
#endif
/*---------------------------------------------------------------------------*/

// sub send data using trickle
static void sub_send(struct trickle_conn *c, uint8_t type, uint8_t counter, uint8_t battery, uint8_t *tx_data, uint8_t size)
{
  uint8_t * pmessage = message;
  uint8_t cnt;
  
  pmessage = message;
  memset(message, 0, size+8);
  *pmessage++ = 0;
  *pmessage++ = 0;
  *pmessage++ = 0xFF;
  *pmessage++ = 0xFF;
  *pmessage++ = type;
  *pmessage++ = counter;
  *pmessage++ = battery;

  for (cnt=0; cnt<size; cnt++)
    *pmessage++ = tx_data[cnt];

  packetbuf_copyfrom(message, size+7);
  trickle_send(c);
}

static void
trickle_recv(struct trickle_conn *c)
{
  uint8_t *ekm_sn;
  
  printf("%d.%d: trickle message received '%s'\n",
	 linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
	 (char *)packetbuf_dataptr());
  printf("/n");
  ekm_sn = packetbuf_dataptr();
  
  // handle S/N request message from GW
  if(ekm_sn[4] == GW_MSG_TYPE_SN)
  {
    memcpy(sn_data, &ekm_sn[6], SN_SIZE);
    process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, sn_data);
  }
}

const static struct trickle_callbacks trickle_call = {trickle_recv};
static struct trickle_conn trickle;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(sub_process, ev, data)
{
  static unsigned int batt;
  static float sane;
  static int dec;
  static float frac;
  static uint8_t *sensor_data;
  uint8_t boot_reason;
    
  PROCESS_EXITHANDLER(trickle_close(&trickle);)

  PROCESS_BEGIN();

  trickle_open(&trickle, CLOCK_SECOND, 145, &trickle_call);

  etimer_set(&et, CLOCK_SECOND);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

  boot_reason = clock_reset_cause();
  sub_send(&trickle, SUB_MSG_TYPE_BOOT, pkt_counter++, 0, &boot_reason, 1);

  deep_sleep_requested = 0;

  while(1) {

    // wait for event 
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE && data != NULL);

    if(*(uint16_t *)data == 0x100)
    {
      // collect battery data
      batt = adc_sensor.value(ADC_SENSOR_TYPE_VDD);
      sane = batt * 3 * 1.15 / 2047;
      dec = sane;
      frac = sane - dec;
      sensor_data = (uint8_t*)data;
      batt = (char)(dec*10)+(char)(frac*10);
  
      // Log data prior to sending out over the air
      logData[0] = pkt_counter;
      logData[1] = batt;
      logData[2] = 0x84;
      logData[3] = 0x00;
      flashlogging_write4(RIME_SUB_CMP_ID, SUB_SEND, logData);  
    
      sub_send(&trickle, SUB_MSG_TYPE_EKM_DATA1, pkt_counter++, batt, &submeter_data[0], 64);

      etimer_set(&et, CLOCK_SECOND);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

      sub_send(&trickle, SUB_MSG_TYPE_EKM_DATA2, pkt_counter++, batt, &submeter_data[64], 64);

      etimer_set(&et, CLOCK_SECOND);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

      sub_send(&trickle, SUB_MSG_TYPE_EKM_DATA3, pkt_counter++, batt, &submeter_data[128], 64);

      etimer_set(&et, CLOCK_SECOND);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

      sub_send(&trickle, SUB_MSG_TYPE_EKM_DATA4, pkt_counter++, batt, &submeter_data[192], 64);
    }
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

  static uint8_t datasize;
  static uint8_t* dataptr;
  static uint16_t crc;
  
  while(1) {
    
    PROCESS_WAIT_EVENT_UNTIL(ev == modbus_line_event_message && data != NULL);
    
    dataptr = &((uint8_t *)data)[1];
    datasize = ((uint8_t *)data)[0];
    
    // copy data into submeter buffer (mask high bit)
    for(uint8_t cnt = 0; cnt < datasize-2; cnt++)
    {
      puthex((dataptr[cnt]) & 0x7F);
      submeter_data[ekm_in_pos++] = (dataptr[cnt]) & 0x7F;
    }
    // copy crc without masking
    puthex((dataptr[datasize-2]));
    submeter_data[ekm_in_pos++] = (dataptr[datasize-2]);
    puthex((dataptr[datasize-1]));
    submeter_data[ekm_in_pos++] = (dataptr[datasize-1]);
    printf("\n");
    
    if(ekm_in_pos >= 0xFF)
    {        
      ekm_in_pos = 0x100;  // known value
      process_post(&sub_process, PROCESS_EVENT_CONTINUE, &ekm_in_pos);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(modbus_out_process, ev, data)
{
  uint8_t *serial_data;
  PROCESS_BEGIN();

  while(1) {
    
    // wait for event 
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE && data != NULL);
    serial_data = (uint8_t *)data;
    
    // reset modbus input index
    ekm_in_pos = 0;
    
    /* modbus write */
    uart_arch_writeb(0x2F);
    uart_arch_writeb(0x3F);

    uart_arch_writeb(serial_data[0]);
    uart_arch_writeb(serial_data[1]);
    uart_arch_writeb(serial_data[2]);
    uart_arch_writeb(serial_data[3]);
    uart_arch_writeb(serial_data[4]);
    uart_arch_writeb(serial_data[5]);
    uart_arch_writeb(serial_data[6]);
    uart_arch_writeb(serial_data[7]);
    uart_arch_writeb(serial_data[8]);
    uart_arch_writeb(serial_data[9]);
    uart_arch_writeb(serial_data[10]);
    uart_arch_writeb(serial_data[11]);

    uart_arch_writeb(0x21);
    uart_arch_writeb(0x0D);
    uart_arch_writeb(0x0A);  
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
