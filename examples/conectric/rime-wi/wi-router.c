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
 *         Example for using the trickle code in Rime
 *         with ranking and multihop
 * \author
 *         Adam Dunkels <adam@sics.se>
 *         Ekawahyu Susilo <ekawahyu@yahoo.com>
 */

// General
#include <stdio.h>

// Contiki
#include "contiki.h"
#include "debug.h"
#include "net/rime/rime.h"
#include "random.h"

// Conectric Device
#include "flash-logging.h"
#include "flash-state.h"
//#include "dev/button-sensor.h"
#include "dev/rs485-arch.h"
#include "dev/serial-line.h"  // REMOVE AFTER DEBUG
#include "dev/modbus-line.h"
#include "dev/modbus-crc16.h"
#include "dev/uart-arch.h"
#include "dev/leds.h"

// Conectric Network
#include "examples/conectric/conectric-messages.h"

#define DEBUG 0

#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#define PUTSTRING(...) putstring(__VA_ARGS__)
#define PUTHEX(...) puthex(__VA_ARGS__)
#define PUTBIN(...) putbin(__VA_ARGS__)
#define PUTDEC(...) putdec(__VA_ARGS__)
#define PUTCHAR(...) putchar(__VA_ARGS__)
#else
#define PRINTF(...)
#define PUTSTRING(...)
#define PUTHEX(...)
#define PUTBIN(...)
#define PUTDEC(...)
#define PUTCHAR(...)
#endif

typedef union {
  uint16_t len;
  uint16_t value;
} modbus_union;

typedef struct {
  uint8_t id;
  uint8_t fc;
  uint16_t addr;
  modbus_union third;
} modbus_request;

//static void compose_request_to_packetbuf(
//    uint8_t * request, uint8_t seqno, linkaddr_t * ereceiver);
static void compose_response_to_packetbuf(
    uint8_t * request, uint8_t seqno, linkaddr_t * ereceiver);
static linkaddr_t * call_decision_maker(void * incoming, uint8_t type);

#define REQUEST_HEADER_LEN    4

//#define MESSAGE_BYTEREQ       1
//#define MESSAGE_BYTECMD       2
#define MESSAGE_ABC_RECV      3
#define MESSAGE_TRICKLE_RECV  4
#define MESSAGE_MHOP_RECV     5 /* uses mhop_message_recv to store message */
#define MESSAGE_MHOP_FWD      6 /* uses mhop_message_recv to store message */

static message_recv abc_message_recv;
static message_recv trickle_message_recv;
static message_recv mhop_message_recv;

extern volatile uint16_t deep_sleep_requested;

/* Flash Logging */
//static uint8_t logData[4]= {0x00, 0x00, 0x00, 0x00};

#define LOGGING_REF_TIME_PD ((clock_time_t)(12U * CLOCK_SECOND * 60U * 60U))

// Trickle state
//static uint16_t rank = 255;

// device general
#define DEVICE_SUP_TIMEOUT ((clock_time_t)(CLOCK_SECOND * 60U * 30U)) // BMB

// WI state
#define WI_ROOM_STATUS_BIT_SHIFT 7
#define WI_HEATING_BIT_SHIFT 6
#define WI_COOLING_BIT_SHIFT 5
#define WI_DOOR_STATUS_BIT_SHIFT 4
#define WI_OCC_STATUS_BIT_SHIFT 3

// WI Message
#define WI_REQUEST_REQID_MSK     (uint8_t) 0x80 // 10000000b
#define WI_REQUEST_MORE_MSK      (uint8_t) 0x40 // 01000000b
#define WI_REQUEST_ERR_MSK       (uint8_t) 0x20 // 00100000b
#define WI_REQUEST_FRAG_MSK      (uint8_t) 0x0F // 00001111b
#define WI_FRAG_SIZE              32
#define WI_MAX_PYLD_SIZE          64
uint8_t wi_last_request_id = WI_REQUEST_REQID_MSK;

// RHT Event Management
//#define RHT_EVENT_STATUS_UNUSED 0
//#define RHT_EVENT_STATUS_INUSE  1
//#define RHT_EVENT_STATUS_SENT   2

typedef struct {
   linkaddr_t   rht_addr;
   uint16_t     temp;
   uint16_t     hum;
   uint8_t      status;
   uint8_t      seqno;
} rht_event_t;

#define MAX_RHT_EVENTS 32   // move this to config
#define RHT_EVENT_SIZE 6   // move this to config
//static uint8_t rht_old_start;
//static uint8_t rht_new_start;
//static uint8_t rht_end;
static rht_event_t rht_event_list[MAX_RHT_EVENTS]; 

// Child Sensor Table Management
#define CHILD_SENSOR_STATUS_UNUSED 0
#define CHILD_SENSOR_STATUS_INUSE  1

// for tracking sensor device_state - link health
#define DEVICE_STATE_LINK_MSK 0xE0
#define DEVICE_STATE_CONN_MSK 0x03
#define CONECTRIC_BURST_NUMBER 5  // move this to common config file
//#define DEVICE_STATE_REPORT_HIST 2 // number of collections of bursts to avg over (more than 2 will not average up over time because of divide)
#define DEVICE_STATE_STABLE 3
#define DEVICE_STATE_UNSTABLE 2
#define DEVICE_STATE_DISCONNECT 1
#define DEVICE_STATE_UNKNOWN 0

// BMB: move to common header
#define SW_BUTTON_OPEN          0x71
#define SW_BUTTON_CLOSED        0x72

typedef struct {
  linkaddr_t    sensor_addr;
  uint8_t       rssi;
  uint8_t       batt;
  uint8_t       device_state;
  uint8_t       status;
  uint8_t       seqno_cnt;  // 4 bits seq no, 4 bits cnt of bursts received
  struct ctimer sup_timer;
} child_sensor_t;

#define MAX_CHILD_SENSORS  10   // move this to config
#define CHILD_SENSOR_SIZE  5   // move this to config
static child_sensor_t child_sensors[MAX_CHILD_SENSORS];

typedef struct {
  uint16_t timestamp;
  uint16_t onboard_temp;
  uint16_t setpoint_temp;
  uint8_t heating;
  uint8_t cooling;
  uint8_t fan_speed;
  uint8_t stage_cooling_heating;
  uint8_t num_rht_events;
  uint8_t num_child_sensors;
} wi_state_t;

static wi_state_t wi_state;
// Can't be over 256 or we need 2 bytes for storing size
#define WI_MSG_MAX_SIZE (uint8_t)(sizeof(wi_state_t) + (MAX_CHILD_SENSORS * CHILD_SENSOR_SIZE) + (MAX_RHT_EVENTS * RHT_EVENT_SIZE))    // move this to config
// byte 0 is size
static uint8_t wi_msg[WI_MSG_MAX_SIZE+1];    // move this to FLASH???

#if CC2530_CONF_MAC_FROM_PRIMARY
#if defined __IAR_SYSTEMS_ICC__
  volatile unsigned char *gmacp = &X_IEEE_ADDR;
#else
  __xdata unsigned char *gmacp = &X_IEEE_ADDR;
#endif
#else
  __code unsigned char *gmacp = (__code unsigned char *)0xFFE8;
#endif

/* RS485 */
//#define BUFSIZE 256
//static uint16_t rs485_in_pos;
//static uint8_t rs485_buffer[BUFSIZE];

/* EKM Messaging */
//#define RS485_DATA_MAX_SIZE 20
//static uint8_t rs485_data_request;
//static linkaddr_t rs485_data_recv;
//static uint8_t rs485_data_payload[RS485_DATA_MAX_SIZE];

// Serial Debug
static uint8_t dump_buffer = 0;

/*---------------------------------------------------------------------------*/
/*
 * TODO add pre-built command line to collect git information on IAR
 *
 */
#ifndef CONECTRIC_VERSION_STRING
#define CONECTRIC_VERSION_STRING "Contiki-unknown"
#endif
#ifndef CONECTRIC_PROJECT_STRING
#define CONECTRIC_PROJECT_STRING "unknow"
#endif

/*---------------------------------------------------------------------------*/
PROCESS(example_abc_process, "ConBurst");
PROCESS(example_trickle_process, "ConTB");
PROCESS(example_multihop_process, "ConMHop");
//PROCESS(serial_in_process, "SerialIn");
//PROCESS(modbus_in_process, "ModbusIn");
//PROCESS(modbus_out_process, "ModbusOut");
PROCESS(modbus_wi_test, "WITest");
PROCESS(flash_process, "FlashLog");
//#if BUTTON_SENSOR_ON
//PROCESS(buttons_test_process, "ButtonTest");
//AUTOSTART_PROCESSES(
//    &example_abc_process,
//    &example_trickle_process,
//    &example_multihop_process,
////    &serial_in_process,
////    &modbus_in_process,
////    &modbus_out_process,
//    &modbus_wi_test,
//    &flash_process,
//    &buttons_test_process);
//#else
AUTOSTART_PROCESSES(
    &example_abc_process,
    &example_trickle_process,
    &example_multihop_process,
//    &serial_in_process,
//    &modbus_in_process,
//    &modbus_out_process,
    &modbus_wi_test,
    &flash_process);
//#endif

/*---------------------------------------------------------------------------*/
/// MOVE THIS TO GENERAL Conectric Network Functions
static uint8_t
shortaddr_cmp(linkaddr_t * addr1, linkaddr_t * addr2)
{
  return (addr1->u8[0] == addr2->u8[0] && addr1->u8[1] == addr2->u8[1]);
}

/*---------------------------------------------------------------------------*/
// RHT Event Management Functions
//static void rht_event_list_init(void)
//{
//    for(int x=0; x<MAX_RHT_EVENTS; x++)
//      rht_event_list[x].status = RHT_EVENT_STATUS_UNUSED;
//}

static void rht_event_list_add(linkaddr_t addr, uint8_t seqno, uint16_t temp, uint16_t hum)
{
  uint8_t cnt = 0;
  // determine if already recorded
  while(cnt < wi_state.num_rht_events)
  {
    if(linkaddr_cmp(&rht_event_list[cnt].rht_addr, &addr) && rht_event_list[cnt].seqno == seqno)
      return;
    cnt++;
  }
  
  if(wi_state.num_rht_events < MAX_RHT_EVENTS)
  {
    linkaddr_copy(&rht_event_list[wi_state.num_rht_events].rht_addr, &addr); 
    rht_event_list[wi_state.num_rht_events].temp = temp;
    rht_event_list[wi_state.num_rht_events].hum = hum;
    rht_event_list[wi_state.num_rht_events].seqno = seqno;
    wi_state.num_rht_events++;
  }
}

/*---------------------------------------------------------------------------*/
// Child Sensor Management Functions

// Manage sensor Supervisory timeout
static void
sensor_timeout(void * arg)
{
  /* Wake up consumer process */
  process_poll(&example_abc_process);
}

static void child_sensors_init(void)
{
  for(int x=0; x < MAX_CHILD_SENSORS; x++)
    child_sensors[x].status = CHILD_SENSOR_STATUS_UNUSED;
}

static child_sensor_t * child_sensors_find(linkaddr_t addr)
{
  for(int x=0; x < MAX_CHILD_SENSORS; x++)
    if(child_sensors[x].status == CHILD_SENSOR_STATUS_INUSE
       && (shortaddr_cmp(&child_sensors[x].sensor_addr, &addr)))
      return &child_sensors[x];
  return NULL;
}

static void child_sensor_restore()
{
  uint8_t * state;
  uint8_t size;
  linkaddr_t child_addr;
  uint8_t ptr;  // parameter to timeout function
  
  size = flashstate_read(FLASH_STATE_WI_SENSOR_LIST, state);
  
  wi_state.num_child_sensors = (uint8_t)(size >> 1);  // 2 bytes per child
      
  for(int ct = 0; ct < wi_state.num_child_sensors; ct++)
  {
    child_addr.u8[0] = *state++;
    child_addr.u8[1] = *state++;;
    linkaddr_copy(&child_sensors[ct].sensor_addr, &child_addr); 
    child_sensors[ct].rssi = 0x00;
    child_sensors[ct].batt = 0x00;
    child_sensors[ct].device_state = (CONECTRIC_BURST_NUMBER << 5) + DEVICE_STATE_UNKNOWN;  // initialize to no missed packets
    child_sensors[ct].status = CHILD_SENSOR_STATUS_INUSE;
    child_sensors[ct].seqno_cnt = CONECTRIC_BURST_NUMBER;
    ctimer_set(&child_sensors[ct].sup_timer, DEVICE_SUP_TIMEOUT, sensor_timeout, &ptr); // start supervisory timer
  }
}

static void process_route_request(uint8_t * payload)
{
  uint16_t timestamp;
  uint8_t num_sensors;
  linkaddr_t child_addr;
  uint8_t * sensor_list;
  volatile uint8_t length, request;
  
  length = *payload++;  // request length
  request = *payload++;  // request ID
  
  if(length - 2 >= 3)  // ensure there's a ROUTE_REQUEST payload to process
  {
    timestamp = (*payload++) << 8 + (*payload++);
    num_sensors = *payload++;
        
    // check for num_sensors > MAX_CHILD_SENSORS
    wi_state.num_child_sensors = (num_sensors < MAX_CHILD_SENSORS) ? num_sensors : MAX_CHILD_SENSORS;
    
    // save pointer to where addresses are located in payload
    sensor_list = payload;
    
    for(int ct=0; ct<MAX_CHILD_SENSORS; ct++)
    {
      if(ct < num_sensors) {
        child_addr.u8[0] = *payload++;
        child_addr.u8[1] = *payload++;;
        linkaddr_copy(&child_sensors[ct].sensor_addr, &child_addr); 
        child_sensors[ct].rssi = 0x00;
        child_sensors[ct].batt = 0x00;
        child_sensors[ct].device_state = (CONECTRIC_BURST_NUMBER << 5) + DEVICE_STATE_UNKNOWN;  // initialize to no missed packets
        child_sensors[ct].status = CHILD_SENSOR_STATUS_INUSE;
        child_sensors[ct].seqno_cnt = CONECTRIC_BURST_NUMBER;
        ctimer_set(&child_sensors[ct].sup_timer, DEVICE_SUP_TIMEOUT, sensor_timeout, &timestamp); // start supervisory timer
      }
      else // clear unused entries
      {
        child_sensors[ct].status = CHILD_SENSOR_STATUS_UNUSED;
      }
    }
    
    // save state in Flash
    flashstate_write(FLASH_STATE_WI_SENSOR_LIST, sensor_list, (uint8_t)(wi_state.num_child_sensors >> 1));
  }
}
/*---------------------------------------------------------------------------*/
// WI State Functions

// REMOVE THIS
//static void wi_test()
//{
//  wi_state.timestamp = clock_seconds();
//  wi_state.onboard_temp += 0x0101;
//  wi_state.setpoint_temp += 0x0101;
//  wi_state.heating += 0x01;
//  wi_state.cooling += 0x01;
//  wi_state.fan_speed += 0x01;
//  wi_state.stage_cooling_heating += 0x01;
  
//  wi_state.num_rht_events = 0x08;
//  rht_event_list[0].rht_addr.u16 = 0x1111;
//  rht_event_list[0].temp = 0x1111;
//  rht_event_list[0].hum = 0x1111;
//  rht_event_list[0].status = RHT_EVENT_STATUS_INUSE;
//  rht_event_list[1].rht_addr.u16 = 0x2222;
//  rht_event_list[1].temp = 0x2222;
//  rht_event_list[1].hum = 0x2222;
//  rht_event_list[1].status = RHT_EVENT_STATUS_INUSE;
//  rht_event_list[2].rht_addr.u16 = 0x3333;
//  rht_event_list[2].temp = 0x3333;
//  rht_event_list[2].hum = 0x3333;
//  rht_event_list[2].status = RHT_EVENT_STATUS_INUSE;
//  rht_event_list[3].rht_addr.u16 = 0x4444;
//  rht_event_list[3].temp = 0x4444;
//  rht_event_list[3].hum = 0x4444;
//  rht_event_list[3].status = RHT_EVENT_STATUS_INUSE;
//  rht_event_list[4].rht_addr.u16 = 0x1111;
//  rht_event_list[4].temp = 0x1111;
//  rht_event_list[4].hum = 0x1111;
//  rht_event_list[4].status = RHT_EVENT_STATUS_INUSE;
//  rht_event_list[5].rht_addr.u16 = 0x2222;
//  rht_event_list[5].temp = 0x2222;
//  rht_event_list[5].hum = 0x2222;
//  rht_event_list[5].status = RHT_EVENT_STATUS_INUSE;
//  rht_event_list[6].rht_addr.u16 = 0x3333;
//  rht_event_list[6].temp = 0x3333;
//  rht_event_list[6].hum = 0x3333;
//  rht_event_list[6].status = RHT_EVENT_STATUS_INUSE;
//  rht_event_list[7].rht_addr.u16 = 0x4444;
//  rht_event_list[7].temp = 0x4444;
//  rht_event_list[7].hum = 0x4444;
//  rht_event_list[7].status = RHT_EVENT_STATUS_INUSE;
  
//  wi_state.num_child_sensors = 0x00;
//  child_sensors[0].sensor_addr.u16 = 0x1111;
//  child_sensors[0].rssi = 0x11;
//  child_sensors[0].batt = 0x11;
//  child_sensors[0].device_state = 0x11;
//  child_sensors[0].status = CHILD_SENSOR_STATUS_INUSE;
//  child_sensors[1].sensor_addr.u16 = 0x2222;
//  child_sensors[1].rssi = 0x22;
//  child_sensors[1].batt = 0x22;
//  child_sensors[1].device_state = 0x22;
//  child_sensors[1].status = CHILD_SENSOR_STATUS_INUSE;
//  child_sensors[2].sensor_addr.u16 = 0x3333;
//  child_sensors[2].rssi = 0x33;
//  child_sensors[2].batt = 0x33;
//  child_sensors[2].device_state = 0x33;
//  child_sensors[2].status = CHILD_SENSOR_STATUS_INUSE;
//}

static void wi_state_init()
{
  // random default values added for test purposes - set back to 00
  wi_state.timestamp = 0x0000;
  wi_state.onboard_temp = 0x0000;
  wi_state.setpoint_temp = 0x0000;
  wi_state.heating = 0x00;
  wi_state.cooling = 0x00;
  wi_state.fan_speed = 0x00;
  wi_state.stage_cooling_heating = 0x00;
  wi_state.num_rht_events = 0x00;
  wi_state.num_child_sensors = 0x00;
}

//static void wi_state_clear()
//{
//  wi_state.timestamp = 0x0000;
//  wi_state.num_rht_events = 0x00;
//}

 // save WI State to mem (and update flash pointers?)
static void wi_state_write()
{
  uint8_t size, cnt, num_evts;
  uint8_t * pyld;
 
  // zero out previous message
  memset(wi_msg, 0, WI_MSG_MAX_SIZE+1);
  
  // timestamp
  wi_state.timestamp = clock_seconds();
  
  // Write WI State to memory
  size = sizeof(wi_state_t);
  pyld = &wi_msg[1];            // set pyld pointer to start of mem (skip 1st byte for size)
  *pyld++ = (uint8_t)(wi_state.timestamp >> 8);
  *pyld++ = (uint8_t)(wi_state.timestamp);
  *pyld++ = (uint8_t)(wi_state.onboard_temp >> 8);
  *pyld++ = (uint8_t)(wi_state.onboard_temp);
  *pyld++ = (uint8_t)(wi_state.setpoint_temp >> 8);
  *pyld++ = (uint8_t)(wi_state.setpoint_temp);
  *pyld++ = wi_state.heating;
  *pyld++ = wi_state.cooling;
  *pyld++ = wi_state.fan_speed;
  *pyld++ = wi_state.stage_cooling_heating;
  *pyld++ = wi_state.num_rht_events;
//  for(cnt=0; cnt<MAX_RHT_EVENTS; cnt++)
//  {
//    if(rht_event_list[cnt].status == RHT_EVENT_STATUS_INUSE)
//    {
//      *pyld++ = rht_event_list[cnt].rht_addr.u8[0];
//      *pyld++ = rht_event_list[cnt].rht_addr.u8[1];
//      *pyld++ = (uint8_t)(rht_event_list[cnt].temp >> 8);
//      *pyld++ = (uint8_t)(rht_event_list[cnt].temp);
//      *pyld++ = (uint8_t)(rht_event_list[cnt].hum >> 8);
//      *pyld++ = (uint8_t)(rht_event_list[cnt].hum);
//      size += RHT_EVENT_SIZE;
//      rht_event_list[cnt].status = RHT_EVENT_STATUS_UNUSED;
//    }
//  }
  num_evts = wi_state.num_rht_events;
  for(cnt=0; cnt<num_evts; cnt++)
  {
    *pyld++ = rht_event_list[cnt].rht_addr.u8[0];
    *pyld++ = rht_event_list[cnt].rht_addr.u8[1];
    *pyld++ = (uint8_t)(rht_event_list[cnt].temp >> 8);
    *pyld++ = (uint8_t)(rht_event_list[cnt].temp);
    *pyld++ = (uint8_t)(rht_event_list[cnt].hum >> 8);
    *pyld++ = (uint8_t)(rht_event_list[cnt].hum);
    size += RHT_EVENT_SIZE;
    wi_state.num_rht_events--;
  }

  *pyld++ = wi_state.num_child_sensors;
  for(cnt=0; cnt<MAX_CHILD_SENSORS; cnt++)
    {
      if(child_sensors[cnt].status == CHILD_SENSOR_STATUS_INUSE)
      {
        *pyld++ = child_sensors[cnt].sensor_addr.u8[0];
        *pyld++ = child_sensors[cnt].sensor_addr.u8[1];
        *pyld++ = child_sensors[cnt].rssi;
        *pyld++ = child_sensors[cnt].batt;
        *pyld++ = child_sensors[cnt].device_state;
        size += CHILD_SENSOR_SIZE;
      }
  }
  // write size 
  wi_msg[0] = size;  
  
//  puthex(WI_MSG_MAX_SIZE);
//  putstring("w");
//  for(cnt = 0; cnt < WI_MSG_MAX_SIZE+1; cnt++)
//  {
//    puthex(wi_msg[cnt]);
//  }
//  putstring("\n");
  
}

// build the wi msg payload 
// parameters: 
//      wi_request: Incoming request
//      header: Return packet header
//      mem_idx: index into memory where WI State is saved
// return size of payload
static uint8_t wi_msg_build(uint8_t * wi_request, uint8_t * header, uint8_t *mem_idx)
{
  *mem_idx = 0;
  
  uint8_t req_header = *wi_request++;
  uint8_t fragment = req_header & WI_REQUEST_FRAG_MSK;
  uint8_t total_size, size;
  
  // response header
  *header = req_header;
  
  // handle request for saved data
  if((uint8_t)(req_header & WI_REQUEST_REQID_MSK) == wi_last_request_id)
  {
     total_size = wi_msg[0];
     if(fragment * WI_FRAG_SIZE > total_size)  // requested data out of bounds
     {  
       // ERROR
       *header |= WI_REQUEST_ERR_MSK;
       size = 0;
     }
     else  // send in WI_MAX_PYLD_SIZE byte chunks
     {
       size = (total_size - (fragment * WI_FRAG_SIZE) < WI_MAX_PYLD_SIZE) ? (total_size - (fragment * WI_FRAG_SIZE)) : WI_MAX_PYLD_SIZE;
       if(size == WI_MAX_PYLD_SIZE && total_size > ((fragment * WI_FRAG_SIZE) + WI_MAX_PYLD_SIZE)) 
          *header |= WI_REQUEST_MORE_MSK;
       // handle error case where there is no data to read
       if(size == 0)
         *header |= WI_REQUEST_ERR_MSK;
       
       *mem_idx = fragment * WI_FRAG_SIZE+1;
//       
//       putstring("i");
//       puthex(*mem_idx);
//       putstring("\n");
     } 
  }
  else
  {    
    wi_last_request_id ^= WI_REQUEST_REQID_MSK;  // toggle ID for re-request or follow up reads
    
    // test simulate wi state
    // wi_test();  // REMOVE THIS
  
    wi_state_write();
    
//    putstring("i");
//    putstring("1");
//    putstring("\n");
//    
    *mem_idx = 1;
    total_size = wi_msg[0];
    size = (total_size < WI_MAX_PYLD_SIZE) ? total_size : WI_MAX_PYLD_SIZE;
    if(size == WI_MAX_PYLD_SIZE && total_size > WI_MAX_PYLD_SIZE) 
      *header |= WI_REQUEST_MORE_MSK;
  }
  return size; 
}

/*---------------------------------------------------------------------------*/
/// MOVE THIS TO GENERAL Conectric Network Functions
static uint8_t
packetbuf_and_attr_copyto(message_recv * message, uint8_t message_type)
{
  uint8_t packetlen, hdrlen;
  uint8_t *dataptr;

  /* Backup the previous senders and receivers */
  linkaddr_copy(&message->prev_sender, &message->sender);
  linkaddr_copy(&message->prev_esender, &message->esender);
  linkaddr_copy(&message->prev_receiver, &message->receiver);
  linkaddr_copy(&message->prev_ereceiver, &message->ereceiver);

  /* Copy the packet attributes to avoid them being overwritten or
   * cleared by an application program that uses the packet buffer for
   * its own needs
   */
  memset(message, 0, sizeof(message));
  message->timestamp = clock_seconds();
  message->message_type = message_type;
  linkaddr_copy(&message->sender, packetbuf_addr(PACKETBUF_ADDR_SENDER));
  linkaddr_copy(&message->esender, packetbuf_addr(PACKETBUF_ADDR_ESENDER));
  linkaddr_copy(&message->ereceiver, packetbuf_addr(PACKETBUF_ADDR_ERECEIVER));
  message->rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
  message->hops = packetbuf_attr(PACKETBUF_ATTR_HOPS);

  /* Copy the packetbuf to avoid them being overwritten or
   * cleared by an application program that uses the packet buffer for
   * its own needs
   */
  packetlen = packetbuf_copyto(message->message);

  /* Decoding payload and its length */
  hdrlen = message->message[0];
  message->seqno = message->message[1];
  message->payload = &message->message[0] + hdrlen;
  message->length = message->message[hdrlen];

  /* Decoding request byte */
//  dataptr = message->payload;
//  message->request = *++dataptr;
  message->request = message->payload[1];
    
  return packetlen;
}
/*---------------------------------------------------------------------------*/
// DEBUG
static void
dump_packetbuf(void)
{
  static uint16_t len;
  static char * packetbuf;

  putstring(">");

  len = packetbuf_hdrlen();
  packetbuf = (char *)packetbuf_hdrptr();
  while(len--) puthex(*packetbuf++);

  len = packetbuf_datalen();
  packetbuf = (char *)packetbuf_dataptr();
  while(len--) puthex(*packetbuf++);

  putstring("\n");
}
/*---------------------------------------------------------------------------*/
// DEBUG
static void
dump_payload(void)
{
  static uint16_t len;
  static char * packetbuf;

  putstring(">");

  len = packetbuf_datalen();
  packetbuf = (char *)packetbuf_dataptr();
  while(len--) puthex(*packetbuf++);

  putstring("\n");
}

/*---------------------------------------------------------------------------*/
static void
abc_recv(struct abc_conn *c)
{
  packetbuf_and_attr_copyto(&abc_message_recv, MESSAGE_ABC_RECV);
  
    /* TODO only the sink should dump packetbuf,
     * but routers have to store sensors data
     */
//    if (dump_buffer)
//      dump_packetbuf();
//    else
//      dump_payload();
//
//    PRINTF("%d.%d: found sensor %d.%d (%d) - %lu\n",
//        linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
//        abc_message_recv.sender.u8[0], abc_message_recv.sender.u8[1],
//        abc_message_recv.rssi, abc_message_recv.timestamp);

    call_decision_maker(&abc_message_recv, MESSAGE_ABC_RECV);
  
}
static const struct abc_callbacks abc_call = {abc_recv};
static struct abc_conn abc;
/*---------------------------------------------------------------------------*/
static void
trickle_recv(struct trickle_conn *c)
{
  uint8_t * dataptr;

  packetbuf_and_attr_copyto(&trickle_message_recv, MESSAGE_TRICKLE_RECV);

  /* Decoding ereceiver address from message, no built-in trickle attribute */
  dataptr = trickle_message_recv.payload;
  trickle_message_recv.ereceiver.u8[1] = *--dataptr;
  trickle_message_recv.ereceiver.u8[0] = *--dataptr;

  /* Get the rank to the sink */
//  rank = trickle_rank(c);

  /* TODO store neighbors as a list here */

  PRINTF("%d.%d: found neighbor %d.%d (%d) - %lu\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      trickle_message_recv.sender.u8[0], trickle_message_recv.sender.u8[1],
      trickle_message_recv.rssi, trickle_message_recv.timestamp);

  call_decision_maker(&trickle_message_recv, MESSAGE_TRICKLE_RECV);
}
const static struct trickle_callbacks trickle_call = {trickle_recv};
static struct trickle_conn trickle;
/*---------------------------------------------------------------------------*/
/*
 * This function is called at the final recepient of the multihop message.
 */
static void
multihop_recv(struct multihop_conn *c, const linkaddr_t *sender,
     const linkaddr_t *prevhop,
     uint8_t hops)
{
  packetbuf_and_attr_copyto(&mhop_message_recv, MESSAGE_MHOP_RECV);

  /* TODO only the sink should dump packetbuf */
  if (dump_buffer)
    dump_packetbuf();
  else
    dump_payload();

  PRINTF("%d.%d: multihop message from %d.%d - (%d hops) - %lu\n",
        linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
        mhop_message_recv.esender.u8[0], mhop_message_recv.esender.u8[1],
        mhop_message_recv.hops, clock_seconds());

  call_decision_maker(&mhop_message_recv, MESSAGE_MHOP_RECV);
}
/*---------------------------------------------------------------------------*/
/*
 * This function is called to forward a packet. It returns a forwarding address
 * determined by trickle message ranking. If no neighbor is found, the function
 * returns NULL to signal to the multihop layer to drop the packet.
 */
static linkaddr_t *
multihop_forward(struct multihop_conn *c,
  const linkaddr_t *originator, const linkaddr_t *dest,
  const linkaddr_t *prevhop, uint8_t hops)
{
  static linkaddr_t * forward_addr;

  packetbuf_and_attr_copyto(&mhop_message_recv, MESSAGE_MHOP_RECV);

  forward_addr = call_decision_maker(&mhop_message_recv, MESSAGE_MHOP_FWD);

  PRINTF("%d.%d: multihop forwarding address is %d.%d - %d hops\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      forward_addr->u8[0], forward_addr->u8[1], mhop_message_recv.hops);

  return forward_addr;
}
static const struct multihop_callbacks multihop_call = {multihop_recv, multihop_forward};
static struct multihop_conn multihop;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_abc_process, ev, data)
{
  PROCESS_EXITHANDLER(abc_close(&abc);)

  PROCESS_BEGIN();

  // didn't want the overhead of another process, 
  // so initializing here as it relates to sensors that report through abc
  child_sensors_init();
  //rht_event_list_init();
  wi_state_init();
  
  abc_open(&abc, 128, &abc_call);

  while(1) {
    // manage timers for child devices - supervisory timeout
    PROCESS_YIELD();
    
    for(int x=0; x < MAX_CHILD_SENSORS; x++)
    {
      // determine which timer expired
      if (child_sensors[x].status == CHILD_SENSOR_STATUS_INUSE && ctimer_expired(&(child_sensors[x].sup_timer))) {
        child_sensors[x].device_state = (child_sensors[x].device_state == DEVICE_STATE_STABLE) ? DEVICE_STATE_UNSTABLE : DEVICE_STATE_DISCONNECT;
      }  
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_trickle_process, ev, data)
{
//  static linkaddr_t to;
//  static uint8_t counter = 0;
//  static uint8_t * request;

  PROCESS_EXITHANDLER(trickle_close(&trickle);)

  PROCESS_BEGIN();

  /* Open a trickle connection */
  trickle_open(&trickle, CLOCK_SECOND, 145, &trickle_call);

  /* Loop forever */
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE && data != NULL);

    /* Compose packetbuf for request comes from serial port with '<' marking
     * or request comes from radio with no '<' marking
     */
//    request = (uint8_t *)data;
//    if (*request == '<')
//      compose_request_to_packetbuf(request, counter++, &to);
//    else
//      compose_response_to_packetbuf(request, counter++, &to);
//
//    /* Send the rank to 1 (source of trickle) */
//    trickle_set_rank(1);
//
//    /* Send the packet */
//    trickle_send(&trickle);
//
//    PRINTF("%d.%d: route request sent to %d.%d - %lu\n",
//        linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
//        to.u8[0], to.u8[1], clock_seconds());
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_multihop_process, ev, data)
{
  //static struct etimer et;
  static linkaddr_t to;
  static uint8_t counter = 0;
  static uint8_t * request;

  PROCESS_EXITHANDLER(multihop_close(&multihop);)

  PROCESS_BEGIN();

  /* Open a multihop connection */
  multihop_open(&multihop, 135, &multihop_call);

  /* Loop forever */
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE && data != NULL);

    /* Compose packetbuf for request comes from serial port with '<' marking
     * or request comes from radio with no '<' marking
     */
    request = (uint8_t *)data;
//    if (*request == '<')
//      compose_request_to_packetbuf(request, counter++, &to);
//    else
    compose_response_to_packetbuf(request, counter++, &to);

    /* TODO delay count (for now) 1 seconds for (local) trickle to subside */
    //etimer_set(&et, CLOCK_SECOND);
    //PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    /* Send the packet */
    multihop_send(&multihop, &to);

    PRINTF("%d.%d: multihop sent to %d.%d - %lu\n",
        linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
        to.u8[0], to.u8[1], clock_seconds());
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
//#if BUTTON_SENSOR_ON
//PROCESS_THREAD(buttons_test_process, ev, data)
//{
//  struct sensors_sensor *sensor;
//  static uint8_t counter;
//  static uint8_t button;
//
//  PROCESS_BEGIN();
//
//  while(1) {
//
//    PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event);
//
//    sensor = (struct sensors_sensor *)data;
//    if(sensor == &button_1_sensor) {
//      button = 0x71;
//      process_post(&sub_process, PROCESS_EVENT_CONTINUE, &button);
//    }
//    if(sensor == &button_2_sensor) {
//      button = 0x72;
//      process_post(&sub_process, PROCESS_EVENT_CONTINUE, &button);
//    }
//  }
//
//  PROCESS_END();
//}
//#endif
/*---------------------------------------------------------------------------*/
//PROCESS_THREAD(serial_in_process, ev, data)
//{
//  static uint8_t * request;
//  static uint8_t counter;
//  static uint8_t hex_string[2];
//  static uint8_t bytereq[128];
//
//  PROCESS_BEGIN();
//
//  while(1) {
//
//    PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message && data != NULL);
//    PRINTF("Serial_RX: %s (len=%d)\n", (uint8_t *)data, strlen(data));
//    printf("%s\n", (uint8_t *)data);
//
//    request = (uint8_t *)data;
//    memset(bytereq, 0, sizeof(bytereq));
//
//    if (request[0] == '<') {
//
//      bytereq[0] = '<';
//      counter = 2;
//
//      /* do conversion from hex string to hex bytes */
//      while(*++request != '\0') {
//
//        /* remove space */
//        if (*request == ' ') continue;
//
//        /* single digit hex string 0-9, A-F, a-f adjustment */
//        if (*request >= 0x30 && *request <= 0x39)
//          *request -= 0x30;
//        else if (*request >= 0x41 && *request <= 0x46)
//          *request -= 0x37;
//        else if (*request >= 0x61 && *request <= 0x66)
//                  *request -= 0x57;
//        else /* skip all input other than hex number */
//          continue;
//
//        hex_string[counter % 2] = *request;
//
//        /* combinining two digits hex bytes into one and store it */
//        if (counter++ % 2)
//          bytereq[(counter >> 1)-1] = (hex_string[0] << 4) + hex_string[1];
//      }
//
//      call_decision_maker(bytereq, MESSAGE_BYTEREQ);
//
//    }
//    else {
//
//      counter = 0;
//
//      /* passthrough until end of line found */
//      while(*request != '\0') {
//        /* remove space */
//        if (*request == ' ') {
//          request++;
//          continue;
//        }
//        if (*request >= 0x61 && *request <= 0x7A)
//          *request -= 0x20;
//        bytereq[counter++] = *request++;
//      }
//
//      call_decision_maker(bytereq, MESSAGE_BYTECMD);
//
//    }
//  }
//
//  PROCESS_END();
//}
/*---------------------------------------------------------------------------*/
//PROCESS_THREAD(modbus_in_process, ev, data)
//{
//  static uint8_t datasize;
//  static uint8_t* dataptr;
//  static uint16_t crc;
//  uint8_t cnt;
//
//  PROCESS_BEGIN();
//
//  /* FIXME workaround to send at least one character out so that
//   * PROCESS_WAIT_EVENT() can receive modbus_line_event_message
//   */
//  uart_arch_writeb(0);
//
//  while(1) {
//
//    PROCESS_WAIT_EVENT_UNTIL(ev == modbus_line_event_message && data != NULL);
//    dataptr = data;
//    printf("got modbus data (%d)\n", *dataptr);
//    dataptr++;
//    while (*dataptr != 0) {
//      puthex(*dataptr++);
//    }
//    putstring("\n");
//    
//  }
//
//  PROCESS_END();
//}
/*---------------------------------------------------------------------------*/
//PROCESS_THREAD(modbus_out_process, ev, data)
//{
//  static struct etimer et;
//  static message_recv * message;
//  static uint8_t * serial_data;
//  static uint8_t reqlen;
//  static uint8_t req;
//  static uint8_t len;
//
//  PROCESS_BEGIN();
//
//  while(1) {
//
//    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE && data != NULL);
//
//    message = (message_recv *)data;
//    serial_data = message->payload;
//    reqlen = *serial_data++;
//    req = *serial_data++;
//
//    len = reqlen - 2;
//
//    /* reset modbus input index */
//    rs485_in_pos = 0;
//
//    /* modbus write */
//    putstring("modbus_out_process: ");
//    while(len--) {
//      puthex(*serial_data);
//      uart_arch_writeb(*serial_data++);
//    }
//    putstring("\n");
//    
//    // store message information from last S/N query for transmission later (don't assume the message structure is still valid)
//    rs485_data_request = message->request;
//    linkaddr_copy(&rs485_data_recv, &message->ereceiver);
//    memcpy(rs485_data_payload, message->payload, message->length);
//  }
//
//  PROCESS_END();
//}
/*---------------------------------------------------------------------------*/
void
fill_modbus_payload(uint8_t * payload, modbus_request * modreq)
{
  static uint16_t crc16_result;
  static uint8_t * modbus_payload;

  *payload++ = 10; /* length, not part of modbus */
  *payload++ = 0;  /* request, not part of modbus */
  modbus_payload = payload;
  *payload++ = modreq->id; /* slave id */
  *payload++ = modreq->fc; /* function code */
  *payload++ = (modreq->addr & 0xFF00) >> 8; /* register starting address H */
  *payload++ = modreq->addr & 0x00FF;        /* register starting address L */
  *payload++ = (modreq->third.len & 0xFF00) >> 8; /* value/total number of registers requested H */
  *payload++ = modreq->third.len & 0x00FF;        /* value/total number of registers requested L */
  crc16_result = modbus_crc16_calc(modbus_payload, 6);
  *payload++ = crc16_result & 0x00FF;
  *payload++ = (crc16_result & 0xFF00) >> 8;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(modbus_wi_test, ev, data)
{
  static struct etimer et;
  static uint8_t payload[20];
  static message_recv message;
//  static uint16_t crc16_result;
//  static uint16_t counter;
  static uint8_t i;
  static modbus_request modreq[] =
  {
      /*** read register holding from 1-20 one at a time ***/
      {.id = 0x01, .fc = 0x03, .addr = 0x0001, .third.len = 0x0001},
      {.id = 0x01, .fc = 0x03, .addr = 0x0002, .third.len = 0x0001},
      {.id = 0x01, .fc = 0x03, .addr = 0x0003, .third.len = 0x0001},
      {.id = 0x01, .fc = 0x03, .addr = 0x0004, .third.len = 0x0001},
      {.id = 0x01, .fc = 0x03, .addr = 0x0005, .third.len = 0x0001},
      {.id = 0x01, .fc = 0x03, .addr = 0x0006, .third.len = 0x0001},
      {.id = 0x01, .fc = 0x03, .addr = 0x0007, .third.len = 0x0001},
      {.id = 0x01, .fc = 0x03, .addr = 0x0008, .third.len = 0x0001},
      {.id = 0x01, .fc = 0x03, .addr = 0x0009, .third.len = 0x0001},
      {.id = 0x01, .fc = 0x03, .addr = 0x000a, .third.len = 0x0001},
      {.id = 0x01, .fc = 0x03, .addr = 0x000b, .third.len = 0x0001},
      {.id = 0x01, .fc = 0x03, .addr = 0x000c, .third.len = 0x0001},
      {.id = 0x01, .fc = 0x03, .addr = 0x000d, .third.len = 0x0001},
      {.id = 0x01, .fc = 0x03, .addr = 0x000e, .third.len = 0x0001},
      {.id = 0x01, .fc = 0x03, .addr = 0x000f, .third.len = 0x0001},
      {.id = 0x01, .fc = 0x03, .addr = 0x0010, .third.len = 0x0001},
      {.id = 0x01, .fc = 0x03, .addr = 0x0011, .third.len = 0x0001},
      {.id = 0x01, .fc = 0x03, .addr = 0x0012, .third.len = 0x0001},
      {.id = 0x01, .fc = 0x03, .addr = 0x0013, .third.len = 0x0001},
      {.id = 0x01, .fc = 0x03, .addr = 0x0014, .third.len = 0x0001},

      /*** read register holding from 1-20 all at once ***/
      {.id = 0x01, .fc = 0x03, .addr = 0x0001, .third.len = 0x0014},

      /* read register holding from 1-20 one at a time FROM DEVICE ID=2,
       * WI should not respond to any of these read request
       */
      {.id = 0x02, .fc = 0x03, .addr = 0x0001, .third.len = 0x0001},
      {.id = 0x02, .fc = 0x03, .addr = 0x0002, .third.len = 0x0001},
      {.id = 0x02, .fc = 0x03, .addr = 0x0003, .third.len = 0x0001},
      {.id = 0x02, .fc = 0x03, .addr = 0x0004, .third.len = 0x0001},
      {.id = 0x02, .fc = 0x03, .addr = 0x0005, .third.len = 0x0001},
      {.id = 0x02, .fc = 0x03, .addr = 0x0006, .third.len = 0x0001},
      {.id = 0x02, .fc = 0x03, .addr = 0x0007, .third.len = 0x0001},
      {.id = 0x02, .fc = 0x03, .addr = 0x0008, .third.len = 0x0001},
      {.id = 0x02, .fc = 0x03, .addr = 0x0009, .third.len = 0x0001},
      {.id = 0x02, .fc = 0x03, .addr = 0x000a, .third.len = 0x0001},
      {.id = 0x02, .fc = 0x03, .addr = 0x000b, .third.len = 0x0001},
      {.id = 0x02, .fc = 0x03, .addr = 0x000c, .third.len = 0x0001},
      {.id = 0x02, .fc = 0x03, .addr = 0x000d, .third.len = 0x0001},
      {.id = 0x02, .fc = 0x03, .addr = 0x000e, .third.len = 0x0001},
      {.id = 0x02, .fc = 0x03, .addr = 0x000f, .third.len = 0x0001},
      {.id = 0x02, .fc = 0x03, .addr = 0x0010, .third.len = 0x0001},
      {.id = 0x02, .fc = 0x03, .addr = 0x0011, .third.len = 0x0001},
      {.id = 0x02, .fc = 0x03, .addr = 0x0012, .third.len = 0x0001},
      {.id = 0x02, .fc = 0x03, .addr = 0x0013, .third.len = 0x0001},
      {.id = 0x02, .fc = 0x03, .addr = 0x0014, .third.len = 0x0001},

      /*** dimming up backlight ***/
      {.id = 0x01, .fc = 0x06, .addr = 0x0008, .third.value = 5},
      {.id = 0x01, .fc = 0x06, .addr = 0x0008, .third.value = 6},
      {.id = 0x01, .fc = 0x06, .addr = 0x0008, .third.value = 7},
      {.id = 0x01, .fc = 0x06, .addr = 0x0008, .third.value = 8},
      {.id = 0x01, .fc = 0x06, .addr = 0x0008, .third.value = 9},
      {.id = 0x01, .fc = 0x06, .addr = 0x0008, .third.value = 10},

      /*** dimming down backlight ***/
      {.id = 0x01, .fc = 0x06, .addr = 0x0008, .third.value = 10},
      {.id = 0x01, .fc = 0x06, .addr = 0x0008, .third.value = 9},
      {.id = 0x01, .fc = 0x06, .addr = 0x0008, .third.value = 8},
      {.id = 0x01, .fc = 0x06, .addr = 0x0008, .third.value = 7},
      {.id = 0x01, .fc = 0x06, .addr = 0x0008, .third.value = 6},
      {.id = 0x01, .fc = 0x06, .addr = 0x0008, .third.value = 5},

      /*** set backlight delay off to 10 seconds ***/
      {.id = 0x01, .fc = 0x06, .addr = 0x0009, .third.value = 10},

      /*** set power off ***/
      {.id = 0x01, .fc = 0x06, .addr = 0x000a, .third.value = 0},

      /*** set power on ***/
      {.id = 0x01, .fc = 0x06, .addr = 0x000a, .third.value = 1},

      /*** set home icon 0-9 ***/
      {.id = 0x01, .fc = 0x06, .addr = 0x000b, .third.value = 0x0030},
      {.id = 0x01, .fc = 0x06, .addr = 0x000b, .third.value = 0x0031},
      {.id = 0x01, .fc = 0x06, .addr = 0x000b, .third.value = 0x0032},
      {.id = 0x01, .fc = 0x06, .addr = 0x000b, .third.value = 0x0033},
      {.id = 0x01, .fc = 0x06, .addr = 0x000b, .third.value = 0x0034},
      {.id = 0x01, .fc = 0x06, .addr = 0x000b, .third.value = 0x0035},
      {.id = 0x01, .fc = 0x06, .addr = 0x000b, .third.value = 0x0036},
      {.id = 0x01, .fc = 0x06, .addr = 0x000b, .third.value = 0x0037},
      {.id = 0x01, .fc = 0x06, .addr = 0x000b, .third.value = 0x0038},
      {.id = 0x01, .fc = 0x06, .addr = 0x000b, .third.value = 0x0039},

      /*** set home icon A-F, L, H, P, U, and then off ***/
      {.id = 0x01, .fc = 0x06, .addr = 0x000b, .third.value = 'A'},
      {.id = 0x01, .fc = 0x06, .addr = 0x000b, .third.value = 'B'},
      {.id = 0x01, .fc = 0x06, .addr = 0x000b, .third.value = 'C'},
      {.id = 0x01, .fc = 0x06, .addr = 0x000b, .third.value = 'D'},
      {.id = 0x01, .fc = 0x06, .addr = 0x000b, .third.value = 'E'},
      {.id = 0x01, .fc = 0x06, .addr = 0x000b, .third.value = 'F'},
      {.id = 0x01, .fc = 0x06, .addr = 0x000b, .third.value = 'L'},
      {.id = 0x01, .fc = 0x06, .addr = 0x000b, .third.value = 'H'},
      {.id = 0x01, .fc = 0x06, .addr = 0x000b, .third.value = 'P'},
      {.id = 0x01, .fc = 0x06, .addr = 0x000b, .third.value = 'U'},
      {.id = 0x01, .fc = 0x06, .addr = 0x000b, .third.value = 0},

      /*** set days icon 1-7 then off ***/
      {.id = 0x01, .fc = 0x06, .addr = 0x000c, .third.value = 0x0001},
      {.id = 0x01, .fc = 0x06, .addr = 0x000c, .third.value = 0x0002},
      {.id = 0x01, .fc = 0x06, .addr = 0x000c, .third.value = 0x0004},
      {.id = 0x01, .fc = 0x06, .addr = 0x000c, .third.value = 0x0008},
      {.id = 0x01, .fc = 0x06, .addr = 0x000c, .third.value = 0x0010},
      {.id = 0x01, .fc = 0x06, .addr = 0x000c, .third.value = 0x0020},
      {.id = 0x01, .fc = 0x06, .addr = 0x000c, .third.value = 0x0040},
      {.id = 0x01, .fc = 0x06, .addr = 0x000c, .third.value = 0},

      /*** set buzzer to play 1-5 ***/
      {.id = 0x01, .fc = 0x06, .addr = 0x000d, .third.value = 1},
      {.id = 0x01, .fc = 0x06, .addr = 0x000d, .third.value = 2},
      {.id = 0x01, .fc = 0x06, .addr = 0x000d, .third.value = 3},
      {.id = 0x01, .fc = 0x06, .addr = 0x000d, .third.value = 4},
      {.id = 0x01, .fc = 0x06, .addr = 0x000d, .third.value = 5},

      /*** clock showing 00:00 and then 99:99 ***/
      {.id = 0x01, .fc = 0x06, .addr = 0x000e, .third.value = 0x0000},
      {.id = 0x01, .fc = 0x06, .addr = 0x000e, .third.value = 0x6363},

      /*** set temperature to 10.0 deg C and increment it by 0.5 deg C ***/
      {.id = 0x01, .fc = 0x06, .addr = 0x000f, .third.value = 100},
      {.id = 0x01, .fc = 0x06, .addr = 0x000f, .third.value = 105},
      {.id = 0x01, .fc = 0x06, .addr = 0x000f, .third.value = 110},
      {.id = 0x01, .fc = 0x06, .addr = 0x000f, .third.value = 115},

      /*** set low temperature from 10.0 deg C to 20.0 deg C ***/
      {.id = 0x01, .fc = 0x06, .addr = 0x0010, .third.value = 100},
      {.id = 0x01, .fc = 0x06, .addr = 0x0010, .third.value = 110},
      {.id = 0x01, .fc = 0x06, .addr = 0x0010, .third.value = 120},
      {.id = 0x01, .fc = 0x06, .addr = 0x0010, .third.value = 130},
      {.id = 0x01, .fc = 0x06, .addr = 0x0010, .third.value = 140},
      {.id = 0x01, .fc = 0x06, .addr = 0x0010, .third.value = 150},
      {.id = 0x01, .fc = 0x06, .addr = 0x0010, .third.value = 160},
      {.id = 0x01, .fc = 0x06, .addr = 0x0010, .third.value = 170},
      {.id = 0x01, .fc = 0x06, .addr = 0x0010, .third.value = 180},
      {.id = 0x01, .fc = 0x06, .addr = 0x0010, .third.value = 190},
      {.id = 0x01, .fc = 0x06, .addr = 0x0010, .third.value = 200},

      /*** set high temperature from 20.0 to 32.0 deg C ***/
      {.id = 0x01, .fc = 0x06, .addr = 0x0011, .third.value = 200},
      {.id = 0x01, .fc = 0x06, .addr = 0x0011, .third.value = 210},
      {.id = 0x01, .fc = 0x06, .addr = 0x0011, .third.value = 220},
      {.id = 0x01, .fc = 0x06, .addr = 0x0011, .third.value = 230},
      {.id = 0x01, .fc = 0x06, .addr = 0x0011, .third.value = 240},
      {.id = 0x01, .fc = 0x06, .addr = 0x0011, .third.value = 250},
      {.id = 0x01, .fc = 0x06, .addr = 0x0011, .third.value = 260},
      {.id = 0x01, .fc = 0x06, .addr = 0x0011, .third.value = 270},
      {.id = 0x01, .fc = 0x06, .addr = 0x0011, .third.value = 280},
      {.id = 0x01, .fc = 0x06, .addr = 0x0011, .third.value = 290},
      {.id = 0x01, .fc = 0x06, .addr = 0x0011, .third.value = 300},
      {.id = 0x01, .fc = 0x06, .addr = 0x0011, .third.value = 310},
      {.id = 0x01, .fc = 0x06, .addr = 0x0011, .third.value = 320},

      /*** set temp compensation to +5.0 deg C ***/
      {.id = 0x01, .fc = 0x06, .addr = 0x0012, .third.value = 50},

      /*** set temp compensation to +4.5 deg C ***/
      {.id = 0x01, .fc = 0x06, .addr = 0x0012, .third.value = 45},

      /*** set temp compensation to -5.0 deg C ***/
      {.id = 0x01, .fc = 0x06, .addr = 0x0012, .third.value = 0x00CE},

      /*** set temp compensation to -4.5 deg C ***/
      {.id = 0x01, .fc = 0x06, .addr = 0x0012, .third.value = 0x00D3},

      /*** show RH data ***/
      {.id = 0x01, .fc = 0x06, .addr = 0x0013, .third.value = 0},

      /*** show WI ID ***/
      {.id = 0x01, .fc = 0x06, .addr = 0x0013, .third.value = 1},

      /*** read status bytes ***/
      {.id = 0x01, .fc = 0x06, .addr = 0x0014, .third.len = 0x0001},
  };

  PROCESS_BEGIN();

  while(1) {

    for (i = 0; i < (sizeof(modreq)/6); i++) {
      message.payload = payload;
      fill_modbus_payload(payload, &modreq[i]);
      // process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE, &message);
      etimer_set(&et, CLOCK_SECOND);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    }

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(flash_process, ev, data)
{
  static struct etimer et;

  PROCESS_BEGIN();

  flashlogging_init();
  flashstate_init();

  child_sensor_restore();
  
  while (1)
  {
    etimer_set(&et, LOGGING_REF_TIME_PD);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    flashlogging_write_fullclock(FLASH_LOGGING_CMP_ID, 0);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
//static void
//compose_request_to_packetbuf(uint8_t * request,
//    uint8_t seqno, linkaddr_t * ereceiver)
//{
//  static uint8_t packet_buffer[128];
//  uint8_t * packet = packet_buffer;
//  uint8_t * header = NULL;
//  uint8_t * route = NULL;
//  linkaddr_t dest;
//  uint8_t req;
//  uint8_t reqlen;
//  uint8_t datalen;
//  uint8_t routelen;
//  uint8_t i;
//
//  /* Request from serial port, skip the '<' */
//  request++;
//
//  reqlen     = *request++;
//  req        = *request++;
//  dest.u8[0] = *request++;
//  dest.u8[1] = *request++;
//  routelen   = *request++;
//
//  if (ereceiver) linkaddr_copy(ereceiver, &dest);
//
//  /* Filling in packetbuf with data and skip routing table */
//  memset(packet_buffer, 0, sizeof(packet_buffer));
//  datalen = reqlen - routelen - REQUEST_HEADER_LEN;
//  *packet++ = datalen + 2;
//  *packet++ = req;
//  route = request;
//  request += (routelen - 1);
//  i = datalen;
//  while (i--) *packet++ = *request++;
//
//  packetbuf_copyfrom(packet_buffer, datalen+2);
//
//  routelen--; /* get rid of the length byte */
//
//  packetbuf_hdralloc(6 + routelen);
//
//  header = (uint8_t *)packetbuf_hdrptr();
//  *header++ = 6 + routelen;   /* header len */
//  *header++ = seqno;          /* seqno */
//  *header++ = 0;              /* hop count */
//  *header++ = 0;              /* number of hops */
//  *header++ = dest.u8[0];     /* destination addr H */
//  *header++ = dest.u8[1];     /* destination addr L */
//  while(routelen--)
//        *header++ = *route++; /* routing table */
//}
/*---------------------------------------------------------------------------*/
static void
compose_response_to_packetbuf(uint8_t * radio_request,
    uint8_t seqno, linkaddr_t * ereceiver)
{
  static uint8_t packet_buffer[128];
  uint8_t * packet = packet_buffer;
  uint8_t * header = NULL;
  uint8_t mem_idx;
  uint8_t req;
//  uint8_t reqlen;
  uint8_t response = 0;
  uint8_t responselen;
//  uint8_t chunk_number = 0;
//  uint8_t chunk_size = 0;
  uint8_t i;
  uint8_t wi_request = 0;
  
//  reqlen = *radio_request++;
  *radio_request++;
  req    = *radio_request++;

  responselen = 2;

  /* Responses to trickle requests */
  if (req == CONECTRIC_ROUTE_REQUEST) {
    response = CONECTRIC_ROUTE_REPLY;
    linkaddr_copy(ereceiver, &trickle_message_recv.esender);
  }
//  if (req == CONECTRIC_ROUTE_REQUEST_BY_SN) {
//    response = CONECTRIC_ROUTE_REPLY;
//    linkaddr_copy(ereceiver, &trickle_message_recv.esender);
//  }

  /* Responses to multihop requests */
//  if (req == CONECTRIC_MULTIHOP_PING) {
//    response = CONECTRIC_MULTIHOP_PING_REPLY;
//    linkaddr_copy(ereceiver, &mhop_message_recv.esender);
//  }
//  if (req == CONECTRIC_POLL_RS485) {
//    response = CONECTRIC_POLL_RS485_REPLY;
//    responselen += 2;
//    linkaddr_copy(ereceiver, &mhop_message_recv.esender);
//  }
//  if (req == CONECTRIC_POLL_RS485_CHUNK) {
//    response = CONECTRIC_POLL_RS485_CHUNK_REPLY;
//    chunk_number = *radio_request++;
//    chunk_size   = *radio_request++;
//    responselen += chunk_size;
//    linkaddr_copy(ereceiver, &mhop_message_recv.esender);
//  }
  if (req == CONECTRIC_POLL_WI) {
    // Update length based on WI State structure
    responselen += wi_msg_build(radio_request, &wi_request, &mem_idx) + 1;  // add 1 for header
    response = CONECTRIC_POLL_WI_REPLY;
    linkaddr_copy(ereceiver, &mhop_message_recv.esender);
  }
//  if (req == CONECTRIC_GET_LONG_MAC) {
//    response = CONECTRIC_GET_LONG_MAC_REPLY;
//    responselen += 8;
//    linkaddr_copy(ereceiver, &mhop_message_recv.esender);
//  }

  memset(packet_buffer, 0, sizeof(packet_buffer));
  *packet++ = responselen;
  *packet++ = response;

  i = responselen-2;
  
  if (req == CONECTRIC_POLL_WI) {
    // add header
    *packet++ = wi_request;
    for(uint8_t x=0; x<(i-1); x++) {
      *packet++ = wi_msg[mem_idx+x];
      //puthex(pyld[x]);
    }
  }
  
//  if (req == CONECTRIC_POLL_RS485) {
//    /* FIXME this has to be calculated from RS485 reply length */
//    *packet++ = 0x04; /* number of chunks available to poll */
//    *packet++ = 0x40; /* chunk size */
//  }

//  if (req == CONECTRIC_POLL_RS485_CHUNK) {
//    for (i = 0; i < chunk_size; i++)
//      *packet++ = rs485_buffer[(chunk_size*chunk_number) + i];
//  }

//  if (req == CONECTRIC_GET_LONG_MAC) {
//    gmacp = &X_IEEE_ADDR;
//    while (i--) {
//      *packet++ = gmacp[i];
//      puthex(gmacp[i]);
//    }
//    putstring("\n");
//  }

  packetbuf_copyfrom(packet_buffer, responselen);

  packetbuf_hdralloc(6);

  header = (uint8_t *)packetbuf_hdrptr();
  *header++ = 6;                /* header len */
  *header++ = seqno;            /* seqno */
  *header++ = 0;                /* hop count */
  *header++ = 0;                /* number of hops */
  *header++ = ereceiver->u8[0]; /* destination addr H */
  *header++ = ereceiver->u8[1]; /* destination addr L */
}
/*---------------------------------------------------------------------------*/
static linkaddr_t *
call_decision_maker(void * incoming, uint8_t type)
{
  static linkaddr_t forward_addr, source_addr;
  message_recv * message = (message_recv *)incoming;
//  uint8_t * bytereq = (uint8_t *)incoming;
//  uint8_t request;
  uint8_t seqno, mhops, hdrlen;
  uint8_t * header;
  int i;
  static child_sensor_t * sensor_list_entry;

  /*******************************************************/
  /***** INTERPRETING COMMAND LINES FROM SERIAL PORT *****/
  /*******************************************************/
  /*
   * BYTECMD Protocol:
   * - It starts with any char, but '<'
   * - Non-capital letter inputs get capitalized automatically
   *
   */
//  if (type == MESSAGE_BYTECMD) {
//
//    /* Command line interpreter */
//    if (bytereq[0] == 'M' && bytereq[1] == 'R') {
//      gmacp = &X_IEEE_ADDR;
//      for(i = 7; i >= 0; i--) puthex(gmacp[i]);
//      putstring("\n");
//    }
//
//    else if (bytereq[0] == 'D' && bytereq[1] == 'P') {
//      dump_buffer = 0;
//      putstring("Ok DP\n");
//    }
//
//    else if (bytereq[0] == 'D' && bytereq[1] == 'B') {
//      dump_buffer = 1;
//      putstring("Ok DB\n");
//    }
//
//    else if (bytereq[0] == 'V' && bytereq[1] == 'E' && bytereq[2] == 'R') {
//      putstring(CONTIKI_VERSION_STRING "\n");
//      putstring(CONECTRIC_PROJECT_STRING "\n");
//    }
//
//    /* Unknown command */
//    else {
//      puthex(linkaddr_node_addr.u8[0]);
//      putstring(".");
//      puthex(linkaddr_node_addr.u8[1]);
//      putstring(": Bad command!\n");
//    }
//
//  }

  /*******************************************************/
  /***** INTERPRETING REQUEST BYTES FROM SERIAL PORT *****/
  /*******************************************************/
  /*
   * BYTEREQ Protocol:
   * -----------------
   * [<][Len][Req][DestH][DestL][RLen][R1H][R1L]...[RnH][RnL][Data0][Data1]...
   *
   * [Len]  = request byte length including [Len], but excluding [<]
   * [RLen] = routing table length including [RLen] itself
   * [RnH]  = the last hop address H ---> [DestH]
   * [RnL]  = the last hop address L ---> [DestL]
   *
   */
//  else if (type == MESSAGE_BYTEREQ) {
//
//    request = bytereq[2];
//
//    /* Request bytes to be sent as trickle */
//    if (request == CONECTRIC_ROUTE_REQUEST 
//        // || request == CONECTRIC_ROUTE_REQUEST_BY_SN
//          )
//      process_post(&example_trickle_process, PROCESS_EVENT_CONTINUE, bytereq);
//
//    /* Request bytes to be sent as multihop */
//    else if (
//        request == CONECTRIC_MULTIHOP_PING ||
//        request == CONECTRIC_POLL_RS485  ||
//        request == CONECTRIC_POLL_RS485_CHUNK  ||
//        request == CONECTRIC_POLL_SENSORS  ||
//        request == CONECTRIC_GET_LONG_MAC)
//      process_post(&example_multihop_process, PROCESS_EVENT_CONTINUE, bytereq);
//
//    /* Unknown request */
//    else {
//      puthex(linkaddr_node_addr.u8[0]);
//      putstring(".");
//      puthex(linkaddr_node_addr.u8[0]);
//      putstring(": Unknown request - 0x");
//      puthex(request);
//      putstring("\n");
//    }
//
//  }

  /*******************************************************/
  /***** RULES TO ASSIGN MULTIHOP FORWARDING ADDRESS *****/
  /*******************************************************/
  if (type == MESSAGE_MHOP_FWD) {

    /* multihop message received but need to be forwarded */
    seqno = mhop_message_recv.seqno;
    mhops = mhop_message_recv.hops;
    hdrlen = mhop_message_recv.message[0];

    /* Discard current packet header */
    packetbuf_copyfrom(mhop_message_recv.payload, mhop_message_recv.length);

    packetbuf_set_addr(PACKETBUF_ADDR_ESENDER, &mhop_message_recv.esender);
    packetbuf_set_addr(PACKETBUF_ADDR_ERECEIVER, &mhop_message_recv.ereceiver);
    packetbuf_set_attr(PACKETBUF_ATTR_HOPS, mhops);

    /* Compose header */
    if (mhop_message_recv.request == CONECTRIC_ROUTE_REPLY) {
      packetbuf_hdralloc(hdrlen+2);
      header = (uint8_t *)packetbuf_hdrptr();
      *header++ = hdrlen+2;
    }
    else {
      packetbuf_hdralloc(hdrlen);
      header = (uint8_t *)packetbuf_hdrptr();
      *header++ = hdrlen;
    }

    *header++ = seqno;
    *header++ = mhops;
    *header++ = mhop_message_recv.message[3]; /* TODO assign max hops here */
    *header++ = mhop_message_recv.ereceiver.u8[0];
    *header++ = mhop_message_recv.ereceiver.u8[1];

    /* multihop request with built-in routing table */
    if (
        //mhop_message_recv.request == CONECTRIC_MULTIHOP_PING ||
        mhop_message_recv.request == CONECTRIC_POLL_RS485  ||
        mhop_message_recv.request == CONECTRIC_POLL_RS485_CHUNK  ||
        mhop_message_recv.request == CONECTRIC_POLL_WI  
        // || mhop_message_recv.request == CONECTRIC_GET_LONG_MAC
          ) {
      forward_addr.u8[0] = mhop_message_recv.message[4 + (mhops << 1)];
      forward_addr.u8[1] = mhop_message_recv.message[5 + (mhops << 1)];
    }
    /* multihop reply, no routing table */
    if (
        //mhop_message_recv.request == CONECTRIC_MULTIHOP_PING_REPLY ||
        mhop_message_recv.request == CONECTRIC_POLL_RS485_REPLY ||
        mhop_message_recv.request == CONECTRIC_POLL_RS485_CHUNK_REPLY ||
        mhop_message_recv.request == CONECTRIC_POLL_WI_REPLY 
        // || mhop_message_recv.request == CONECTRIC_GET_LONG_MAC_REPLY
          ) {
      linkaddr_copy(&forward_addr, &mhop_message_recv.prev_sender);
      packetbuf_set_addr(PACKETBUF_ADDR_ESENDER, &mhop_message_recv.esender);
      packetbuf_set_addr(PACKETBUF_ADDR_ERECEIVER, &mhop_message_recv.prev_esender);
    }
    /* multihop reply, update routing table on every hop */
    if (mhop_message_recv.request == CONECTRIC_ROUTE_REPLY) {
      *header++ = linkaddr_node_addr.u8[0];
      *header++ = linkaddr_node_addr.u8[1];
      linkaddr_copy(&forward_addr, &trickle_message_recv.sender);
    }

    for (i = 6; i < hdrlen; i++) {
      *header++ = mhop_message_recv.message[i];
    }

    return &forward_addr;
  }

  /* Sensor message received */
  else if(type == MESSAGE_ABC_RECV)
  {
    uint8_t first_burst = 0x00;
    
    /* General ABC Processing */
    source_addr = message->sender;
        
    sensor_list_entry = child_sensors_find(source_addr);
    if(sensor_list_entry == NULL)
      return NULL; // not in child list

    sensor_list_entry->rssi = (uint8_t)(message->rssi >> 8);
    sensor_list_entry->batt =  message->payload[2];

    seqno = message->seqno;
   
    // keep track of burst count for link status (device_state)
    if((seqno & 0x0F) == (uint8_t)((sensor_list_entry->seqno_cnt & 0xF0) >> 4))
    {
      sensor_list_entry->seqno_cnt++;
      // received multiple bursts, set state to stable connection
      sensor_list_entry->device_state = ((sensor_list_entry->device_state & DEVICE_STATE_LINK_MSK) + DEVICE_STATE_STABLE);
    }
    else // new burst
    {
      int8_t health;
      
      first_burst = 0x01;
      
      // update device_state based on last burst data (increment / decrement by 1)
      health = ((sensor_list_entry->device_state & DEVICE_STATE_LINK_MSK) >> 5);
      health += ((sensor_list_entry->seqno_cnt & 0x0F) >= 3) ? 1 : -1;
      if(health < 0) 
        health = 0;
      if(health > CONECTRIC_BURST_NUMBER) 
        health = CONECTRIC_BURST_NUMBER;
      sensor_list_entry->device_state = (sensor_list_entry->device_state & ~DEVICE_STATE_LINK_MSK) + ((uint8_t)(health) << 5);
      sensor_list_entry->seqno_cnt = ((seqno & 0x0F) << 4) + 1;    
      // restart Supevisory timer
      ctimer_restart(&(sensor_list_entry->sup_timer));
    }

    // Handle specific messages
    if(first_burst == 0x01) 
    {
      if(message->request == CONECTRIC_SENSOR_BROADCAST_RHT)
      {
        static uint16_t temp, hum;
          
        // write to event list
        temp = (uint16_t)((message->payload[3] << 8) + message->payload[4]);
        hum = (uint16_t)((message->payload[5] << 8) + message->payload[6]);
        rht_event_list_add(message->sender, seqno, temp, hum);
      }
      else if(message->request == CONECTRIC_SENSOR_BROADCAST_SW)
      {
        uint8_t button;
        
        button = message->payload[4];

        // WI occupied algorithm for SW event
        // BMB: THIS OBVIOUSLY NEEDS TO BE CHANGED - Just placeholder for now
        if(button == SW_BUTTON_OPEN)
        {
          wi_state.stage_cooling_heating |= (1 << WI_DOOR_STATUS_BIT_SHIFT);
          wi_state.stage_cooling_heating ^= (1 << WI_ROOM_STATUS_BIT_SHIFT);
        }
        else // door closed event
        {
          wi_state.stage_cooling_heating &= ~(1 << WI_DOOR_STATUS_BIT_SHIFT);          
        }
      }
      else if(message->request == CONECTRIC_SENSOR_BROADCAST_OC)
      {
        // WI occupied algorithm for OC event
        // BMB: THIS OBVIOUSLY NEEDS TO BE CHANGED - Just placeholder for now
        wi_state.stage_cooling_heating |= (1 << WI_OCC_STATUS_BIT_SHIFT) + (1 << WI_ROOM_STATUS_BIT_SHIFT);
      }
    } 
  }
  
  /*******************************************************/
  /***** ALL THE REST ARE HANDLED HERE *******************/
  /*******************************************************/
  else {
    
    /* trickle message received */
    if (message->request == CONECTRIC_ROUTE_REQUEST)
    {
      // Trigger Route Reply Send
      if (shortaddr_cmp(&message->ereceiver, &linkaddr_node_addr))
      {
        // handle Route Request Payload      
        process_route_request(message->payload);    
        process_post(&example_multihop_process, PROCESS_EVENT_CONTINUE,
            message->payload);
      }
    }
    
//    if (message->request == CONECTRIC_ROUTE_REQUEST_BY_SN)
//      if (message->ereceiver.u8[0] == 0xFF && message->ereceiver.u8[1] == 0xFF)
//        process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE,
//            message);

    /* multihop message received */
    if (
        // message->request == CONECTRIC_MULTIHOP_PING ||
        // message->request == CONECTRIC_POLL_RS485_CHUNK  ||
        message->request == CONECTRIC_POLL_WI  
        // || message->request == CONECTRIC_GET_LONG_MAC
          )
    {
      // Trigger Reply
      if (shortaddr_cmp(&message->ereceiver, &linkaddr_node_addr))
        process_post(&example_multihop_process, PROCESS_EVENT_CONTINUE,
            message->payload);
    }
    
//    if (message->request == CONECTRIC_POLL_RS485)
//      if (shortaddr_cmp(&message->ereceiver, &linkaddr_node_addr))
//        process_post(&modbus_out_process, PROCESS_EVENT_CONTINUE,
//            message);

  }

  return NULL;
}
/*---------------------------------------------------------------------------*/
void
invoke_process_before_sleep(void)
{
  deep_sleep_requested = 0;
}
/*---------------------------------------------------------------------------*/
