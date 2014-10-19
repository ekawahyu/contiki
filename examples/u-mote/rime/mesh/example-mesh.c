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
 *         An example of how the Mesh primitive can be used.
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "net/rime/rime.h"
#include "net/rime/mesh.h"

#include "dev/button-sensor.h"

#include "dev/leds.h"

#include <stdio.h>
#include <string.h>

#define MESSAGE "Hello"

static struct mesh_conn mesh;
int mesh_is_connected(void); /* TODO clean up this function prototype */
/*---------------------------------------------------------------------------*/
PROCESS(example_mesh_process, "Mesh example");
AUTOSTART_PROCESSES(&example_mesh_process);
/*---------------------------------------------------------------------------*/
static void
sent(struct mesh_conn *c)
{
  printf("packet sent\n");
}

static void
timedout(struct mesh_conn *c)
{
  printf("packet timedout\n");
}

static void
recv(struct mesh_conn *c, const linkaddr_t *from, uint8_t hops)
{
  printf("Data received from %x.%x: %s (%d)\n",
  from->u8[0], from->u8[1],
  (char *)packetbuf_dataptr(), packetbuf_datalen());
}

const static struct mesh_callbacks callbacks = {recv, sent, timedout};
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_mesh_process, ev, data)
{
  static struct etimer periodic;
  static linkaddr_t addr;
  static int ret;

  PROCESS_EXITHANDLER(mesh_close(&mesh);)
  PROCESS_BEGIN();

  mesh_open(&mesh, 132, &callbacks);

  SENSORS_ACTIVATE(button_sensor);

  addr.u8[0] = 0xEE;
  addr.u8[1] = 0x35;

  while(1) {

    if (mesh_is_connected())
      etimer_set(&periodic, 30*CLOCK_SECOND);
    else
      etimer_set(&periodic, 10*CLOCK_SECOND);

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic));
    etimer_reset(&periodic);

    /* Send a message to node number 1. */

    if (linkaddr_cmp(&linkaddr_node_addr, &addr) == 0) {
      packetbuf_copyfrom(MESSAGE, strlen(MESSAGE));
      addr.u8[0] = 0xEE;
      addr.u8[1] = 0x35;
      ret = mesh_send(&mesh, &addr);
      printf("mesh_send: %i (%d)\n", ret, (unsigned int)clock_seconds());
    }
    else printf("I am the sink\n");
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
