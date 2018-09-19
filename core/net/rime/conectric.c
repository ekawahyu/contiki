/*
 * conectric.c
 *
 * Created on: Feb 13, 2018
 *     Author: Ekawahyu Susilo
 *
 * Copyright (c) 2018, Conectric Network, LLC.
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
#include "lib/list.h"
#include "lib/memb.h"
#include "net/rime/rime.h"
#include "net/rime/sink.h"
#include "net/rime/conectric.h"
#include "random.h"

#include <stddef.h> /* For offsetof */

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define SINK_NETBC          0xBCBC
#define SIGNAL_MAX_PAYLOAD  20
#define SIGNAL_HEADER_SIZE  6

struct sink_msg {
  uint16_t netbc;
};

struct signal_msg {
  uint8_t header_len;
  uint8_t seqno;
  uint8_t hop;
  uint8_t ttl;
  uint8_t addrh;
  uint8_t addrl;
  uint8_t payload_len;
  uint8_t payload[SIGNAL_MAX_PAYLOAD];
};

static uint8_t is_sink;
static uint8_t is_collect;

/*---------------------------------------------------------------------------*/
static void
broadcast_received(struct broadcast_conn *bc, const linkaddr_t *from)
{
  struct conectric_conn *c = (struct conectric_conn *)
    ((char *)bc - offsetof(struct conectric_conn, broadcast));

  PRINTF("%d.%d: broadcast received from %d.%d\n",
   linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
   from->u8[0], from->u8[1]);

  if(c->cb->localbroadcast_recv) {
    c->cb->localbroadcast_recv(c, from);
  }
}
/*---------------------------------------------------------------------------*/
static void
netbc_received(struct multicast_conn *mc, const linkaddr_t *originator)
{
#if CONECTRIC_CONF_ROUTER
  struct sink_msg *msg = packetbuf_dataptr();
#endif
  struct conectric_conn *c = (struct conectric_conn *)
    ((char *)mc - offsetof(struct conectric_conn, netbc));

  uint8_t hops = packetbuf_attr(PACKETBUF_ATTR_HOPS);

  PRINTF("%d.%d: netbc received from %d.%d hops %d\n",
   linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
   originator->u8[0], originator->u8[1], hops);

#if CONECTRIC_CONF_ROUTER
  if (msg->netbc == SINK_NETBC) {
    if(c->cb->sink_recv && linkaddr_cmp(originator, &linkaddr_node_addr) == 0) {
      sink_add(originator, hops);
      c->cb->sink_recv(c, originator, hops);
    }
  }
  else {
#endif
    if(c->cb->netbroadcast_recv) {
      c->cb->netbroadcast_recv(c, originator, hops);
    }
#if CONECTRIC_CONF_ROUTER
  }
#endif
}
/*---------------------------------------------------------------------------*/
static void
netuc_received(struct multicast_conn *mc, const linkaddr_t *originator)
{
  struct conectric_conn *c = (struct conectric_conn *)
    ((char *)mc - offsetof(struct conectric_conn, netuc));

  uint8_t hops = packetbuf_attr(PACKETBUF_ATTR_HOPS);

  PRINTF("%d.%d: netuc received from %d.%d hops %d\n",
   linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
   originator->u8[0], originator->u8[1], hops);

  if(c->cb->recv) {
    c->cb->recv(c, originator, hops);
  }
}
/*---------------------------------------------------------------------------*/
static void send_sink_netbc(struct multicast_conn *mc);
/*---------------------------------------------------------------------------*/
static void
timer_callback(void *ptr)
{
  struct multicast_conn *c = ptr;
  send_sink_netbc(c);
}
/*---------------------------------------------------------------------------*/
static void
send_sink_netbc(struct multicast_conn *mc)
{
  struct sink_msg * msg;
  struct conectric_conn *c = (struct conectric_conn *)
    ((char *)mc - offsetof(struct conectric_conn, netbc));

  packetbuf_clear();
  msg = packetbuf_dataptr();
  packetbuf_set_datalen(sizeof(struct sink_msg));

  msg->netbc = SINK_NETBC;
  multicast_send(&c->netbc, &multicast_node_addr.host);

  PRINTF("%d.%d: sending sink broadcast\n",
   linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);

  ctimer_set(&c->interval_timer, c->interval, timer_callback, mc);
}
/*---------------------------------------------------------------------------*/
static const struct broadcast_callbacks broadcast_call = {
    broadcast_received, NULL};
static const struct multicast_callbacks netbc_call = {
    netbc_received, NULL};
static const struct multicast_callbacks netuc_call = {
    netuc_received, NULL};
/*---------------------------------------------------------------------------*/
void
conectric_init(void) {
#if CONECTRIC_CONF_ROUTER
  sink_init();
#endif
  multicast_linkaddr_init();
}
/*---------------------------------------------------------------------------*/
void
conectric_open(struct conectric_conn *c, uint16_t channels,
	  const struct conectric_callbacks *callbacks)
{
  multicast_open(&c->netbc, &multicast_node_addr, &netbc_call);
#if CONECTRIC_CONF_ROUTER
  multicast_open(&c->netbc, &multicast_router_addr, &netbc_call);
#endif
  multicast_open(&c->netuc, &multicast_linklocal_addr, &netuc_call);
  broadcast_open(&c->broadcast, channels, &broadcast_call);
  c->cb = callbacks;
  is_sink = 0;
  is_collect = 0;
}
/*---------------------------------------------------------------------------*/
void
conectric_close(struct conectric_conn *c)
{
  broadcast_close(&c->broadcast);
  multicast_close(&c->netbc);
  multicast_close(&c->netuc);
  ctimer_stop(&c->interval_timer);
}
/*---------------------------------------------------------------------------*/
int
conectric_send(struct conectric_conn *c, const linkaddr_t *to)
{
  PRINTF("%d.%d: conectric_send to %d.%d\n",
	 linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
	 to->u8[0], to->u8[1]);

  if (to->u8[0] == 255 && to->u8[1] == 255) {
    broadcast_send(&c->broadcast);
  }
  else if (to->u8[0] == 0 && to->u8[1] == 0) {
    multicast_send(&c->netbc, &multicast_node_addr.host);
  }
  else {
    multicast_send(&c->netuc, to);
  }

  if(c->cb->sent != NULL) {
    c->cb->sent(c);
  }

  return 1;
}
/*---------------------------------------------------------------------------*/
linkaddr_t *
conectric_send_to_sink(struct conectric_conn *c)
{
  static struct sink_entry * sink_available;

  if (is_sink) return NULL;

  sink_available = sink_lookup(NULL);

  if (sink_available) {
    PRINTF("%d.%d: conectric_send_to_sink to %d.%d\n",
        linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
        sink_available->addr.u8[0], sink_available->addr.u8[1]);
    multicast_send(&c->netuc, &sink_available->addr);
  }
  else {
    PRINTF("%d.%d: conectric_send_to_sink will multicast it\n",
        linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
    multicast_send(&c->netbc, &multicast_node_addr.host);
  }

  if(c->cb->sent != NULL) {
    c->cb->sent(c);
  }

  return &sink_available->addr;
}
/*---------------------------------------------------------------------------*/
int
conectric_send_signal(struct conectric_conn *c, uint8_t * signal, uint8_t len)
{
  static uint8_t seqno = 0;
  struct signal_msg * msg;

  if (len > SIGNAL_MAX_PAYLOAD) return 0;

  PRINTF("%d.%d: sending signal (%d)\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1], len);

  packetbuf_clear();
  msg = packetbuf_dataptr();
  msg->header_len = SIGNAL_HEADER_SIZE;
  msg->seqno = seqno++;
  msg->hop = 0;
  msg->ttl = 0;
  msg->addrh = linkaddr_node_addr.u8[0];
  msg->addrl = linkaddr_node_addr.u8[1];
  msg->payload_len = len + 1;
  packetbuf_set_datalen(msg->header_len + msg->payload_len);

  while(len--) {
    msg->payload[len] = signal[len];
  }

  broadcast_send(&c->broadcast);

  if(c->cb->sent != NULL) {
    c->cb->sent(c);
  }

  return 1;
}
/*---------------------------------------------------------------------------*/
#if CONECTRIC_CONF_ROUTER
void
conectric_set_sink(struct conectric_conn *c, clock_time_t interval,
    uint8_t sink)
{
  if (sink) sink = 1;
  if (is_sink == sink) return;

  is_sink = sink;

  if (is_sink) {
    is_collect = 0;
    c->interval = interval;
    send_sink_netbc(&c->netbc);
  }
  else {
    ctimer_stop(&c->interval_timer);
  }
}
/*---------------------------------------------------------------------------*/
void
conectric_set_collect(struct conectric_conn *c, uint8_t collect)
{
  if (collect) collect = 1;
  if (is_collect == collect) return;

  is_collect = collect;

  if (is_collect) conectric_set_sink(c, 0, 0);
}
#endif
/*---------------------------------------------------------------------------*/
uint8_t
conectric_is_sink(void)
{
  return is_sink;
}
/*---------------------------------------------------------------------------*/
uint8_t
conectric_is_collect(void)
{
  return is_collect;
}
/*---------------------------------------------------------------------------*/
