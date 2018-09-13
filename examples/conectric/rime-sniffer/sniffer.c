/*
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

/* General */
#include <stdio.h>

/* Contiki */
#include "contiki.h"
#include "cc253x.h"
#include "debug.h"
#include "net/rime/rime.h"
#include "net/rime/conectric.h"
#include "random.h"

/* Conectric Device */
#include "dev/serial-line.h"

/* Conectric Network */
#include "../command.h"
#include "../conectric-messages.h"

#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static uint8_t dump_header = 0;
/*---------------------------------------------------------------------------*/
void
dump_packet_buffer(uint8_t mode)
{
  dump_header = mode;
}
/*---------------------------------------------------------------------------*/
struct conectric_conn conectric;
/*---------------------------------------------------------------------------*/
PROCESS(sniffer_process, "Sniffer process");
PROCESS(serial_in_process, "SerialIn");
AUTOSTART_PROCESSES(&sniffer_process, &serial_in_process);
/*---------------------------------------------------------------------------*/
static void
sent(struct conectric_conn *c)
{
  PRINTF("packet sent\n");
}

static void
recv(struct conectric_conn *c, const linkaddr_t *from, uint8_t hops)
{
  PRINTF("%d.%d: data from %d.%d len %d hops %d\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      from->u8[0], from->u8[1], packetbuf_datalen(), hops);
}

static void
localbroadcast(struct conectric_conn *c, const linkaddr_t *from)
{
  PRINTF("%d.%d: localbc from %d.%d: len %d\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      from->u8[0], from->u8[1], packetbuf_datalen());
}

static void
netbroadcast(struct conectric_conn *c, const linkaddr_t *from, uint8_t hops)
{
  PRINTF("%d.%d: netbc from %d.%d: len %d hops %d\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      from->u8[0], from->u8[1], packetbuf_datalen(), hops);
}

static void
sink(struct conectric_conn *c, const linkaddr_t *from, uint8_t hops)
{
  PRINTF("%d.%d: sink from %d.%d len %d hops %d\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      from->u8[0], from->u8[1], packetbuf_datalen(), hops);
}

const static struct conectric_callbacks callbacks = {
    recv, sent, localbroadcast, netbroadcast, sink
};
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(sniffer_process, ev, data)
{
  static struct etimer et;

  PROCESS_EXITHANDLER(conectric_close(&conectric);)

  PROCESS_BEGIN();

  PRINTF("Sniffer started\n");

  /* Turn off RF Address Recognition - We need to accept all frames */
  FRMFILT0 &= ~0x01;

  conectric_init();
  conectric_open(&conectric, 132, &callbacks);
  conectric_set_collect(&conectric, 1);

  /* Wait until system is completely booted up and ready */
  etimer_set(&et, CLOCK_SECOND);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE && data != NULL);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(serial_in_process, ev, data)
{
  static uint8_t * event;

  PROCESS_BEGIN();

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message && data != NULL);
    PRINTF("Serial_RX: %s (len=%d)\n", (uint8_t *)data, strlen(data));
    printf("%s\n", (uint8_t *)data);

    event = command_interpreter((uint8_t *)data);

    if (event) {
      process_post(&sniffer_process, PROCESS_EVENT_CONTINUE, event);
      PRINTF("Command: %i\n", event[2]);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
invoke_process_before_sleep(void)
{

}
/*---------------------------------------------------------------------------*/
