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
#include "net/rime/rime.h"
#include "net/rime/route.h"
#include "net/rime/conectric.h"

#include <stddef.h> /* For offsetof */

#define PACKET_TIMEOUT (CLOCK_SECOND * 5)

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define SINK_NETBC  0xBCBC

struct sink_msg {
  uint16_t netbc;
};

static int conectric_status;

/*---------------------------------------------------------------------------*/
static int
netflood_received(struct netflood_conn *nf, const linkaddr_t *from,
         const linkaddr_t *originator, uint8_t seqno, uint8_t hops)
{
  struct sink_msg *msg = packetbuf_dataptr();
  struct conectric_conn *c = (struct conectric_conn *)
    ((char *)nf - offsetof(struct conectric_conn, netflood));

  PRINTF("%d.%d: broadcast received from %d.%d hops %d seqno %d\n",
   linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
   originator->u8[0], originator->u8[1],
   hops, seqno);

  if (msg->netbc == SINK_NETBC) {
    if(c->cb->sink_recv) {
      c->cb->sink_recv(c, originator, hops);
    }
  }
  else {
    if(c->cb->netbroadcast_recv) {
      c->cb->netbroadcast_recv(c, originator, hops);
    }
  }

  return 1; /* Continue flooding the network */
}
/*---------------------------------------------------------------------------*/
static void
multihop_received(struct multihop_conn *multihop,
		     const linkaddr_t *from,
		     const linkaddr_t *prevhop, uint8_t hops)
{
  struct conectric_conn *c = (struct conectric_conn *)
    ((char *)multihop - offsetof(struct conectric_conn, multihop));

  struct route_entry *rt;

  /* Refresh the route when we hear a packet from a neighbor. */
  rt = route_lookup(from);
  if(rt != NULL) {
    route_refresh(rt);
  }
  
  if(c->cb->recv) {
    c->cb->recv(c, from, hops);
  }
}
/*---------------------------------------------------------------------------*/
static linkaddr_t *
multihop_forward(struct multihop_conn *multihop,
		    const linkaddr_t *originator,
		    const linkaddr_t *dest,
		    const linkaddr_t *prevhop, uint8_t hops)
{
  struct route_entry *rt;
  struct conectric_conn *c = (struct conectric_conn *)
    ((char *)multihop - offsetof(struct conectric_conn, multihop));

  rt = route_lookup(dest);
  if(rt == NULL) {
    if(c->queued_data != NULL) {
      queuebuf_free(c->queued_data);
    }

    PRINTF("%d.%d: data_packet_forward: queueing data, sending rreq\n",
        linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
    c->queued_data = queuebuf_new_from_packetbuf();
    linkaddr_copy(&c->queued_data_dest, dest);
    route_discovery_discover(&c->route_discovery_conn, dest, PACKET_TIMEOUT);

    return NULL;
  } else {
    route_refresh(rt);
  }
  
  return &rt->nexthop;
}
/*---------------------------------------------------------------------------*/
static void
found_route(struct route_discovery_conn *rdc, const linkaddr_t *dest)
{
  struct route_entry *rt;
  struct conectric_conn *c = (struct conectric_conn *)
    ((char *)rdc - offsetof(struct conectric_conn, route_discovery_conn));

  PRINTF("%d.%d: found_route\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);

  if(c->queued_data != NULL &&
     linkaddr_cmp(dest, &c->queued_data_dest)) {
    queuebuf_to_packetbuf(c->queued_data);
    queuebuf_free(c->queued_data);
    c->queued_data = NULL;

    rt = route_lookup(dest);
    if(rt != NULL) {
      multihop_resend(&c->multihop, &rt->nexthop);
      if(c->cb->sent != NULL) {
        c->cb->sent(c);
      }
    } else {
      if(c->cb->timedout != NULL) {
        c->cb->timedout(c);
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
route_timed_out(struct route_discovery_conn *rdc)
{
  struct conectric_conn *c = (struct conectric_conn *)
    ((char *)rdc - offsetof(struct conectric_conn, route_discovery_conn));

  if(c->queued_data != NULL) {
    queuebuf_free(c->queued_data);
    c->queued_data = NULL;
  }

  if(c->cb->timedout) {
    c->cb->timedout(c);
  }
}
/*---------------------------------------------------------------------------*/
static void send_sink_netbc(struct netflood_conn *nf);
/*---------------------------------------------------------------------------*/
static void
timer_callback(void *ptr)
{
  struct netflood_conn *c = ptr;
  send_sink_netbc(c);
}
/*---------------------------------------------------------------------------*/
static void
send_sink_netbc(struct netflood_conn *nf)
{
  struct sink_msg * msg;
  struct conectric_conn *c = (struct conectric_conn *)
    ((char *)nf - offsetof(struct conectric_conn, netflood));

  packetbuf_clear();
  msg = packetbuf_dataptr();
  packetbuf_set_datalen(sizeof(struct sink_msg));

  msg->netbc = SINK_NETBC;
  netflood_send(&c->netflood, c->netbc_id++);

  PRINTF("%d.%d: sending sink broadcast\n",
   linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);

  ctimer_set(&c->interval_timer, c->interval, timer_callback, nf);
}
/*---------------------------------------------------------------------------*/
static const struct netflood_callbacks netflood_call = {netflood_received, NULL, NULL};
static const struct multihop_callbacks multihop_call = { multihop_received, multihop_forward };
static const struct route_discovery_callbacks route_discovery_callbacks = { found_route, route_timed_out };
/*---------------------------------------------------------------------------*/
void
conectric_open(struct conectric_conn *c, uint16_t channels,
	  const struct conectric_callbacks *callbacks)
{
  route_init();
  netflood_open(&c->netflood, CLOCK_SECOND/8, channels, &netflood_call);
  multihop_open(&c->multihop, channels + 1, &multihop_call);
  route_discovery_open(&c->route_discovery_conn,
		       CLOCK_SECOND/8,
		       channels + 3,
		       &route_discovery_callbacks);
  c->cb = callbacks;
  c->is_sink = 0;
}
/*---------------------------------------------------------------------------*/
void
conectric_close(struct conectric_conn *c)
{
  multihop_close(&c->multihop);
  netflood_close(&c->netflood);
  route_discovery_close(&c->route_discovery_conn);
  ctimer_stop(&c->interval_timer);
}
/*---------------------------------------------------------------------------*/
int
conectric_send(struct conectric_conn *c, const linkaddr_t *to)
{
  int could_send;

  PRINTF("%d.%d: conectric_send to %d.%d\n",
	 linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
	 to->u8[0], to->u8[1]);

  if (to->u8[0] == 255 && to->u8[1] == 255) {
    could_send = netflood_send(&c->netflood, c->netbc_id++);
  }
  else {
    could_send = multihop_send(&c->multihop, to);
  }

  if(!could_send) {
    PRINTF("%d.%d: conectric_send: could not send\n",
        linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
    conectric_status = 0;
    return 0;
  }
  if(c->cb->sent != NULL) {
    c->cb->sent(c);
  }
  conectric_status = 1;
  return 1;
}
/*---------------------------------------------------------------------------*/
void
conectric_set_sink(struct conectric_conn *c, clock_time_t interval, uint8_t is_sink)
{
  c->is_sink = is_sink;
  c->interval = interval;
  if (is_sink) {
    send_sink_netbc(&c->netflood);
  }
  else {
    ctimer_stop(&c->interval_timer);
  }
}

int
conectric_ready(struct conectric_conn *c)
{
  return (c->queued_data == NULL);
}

int
conectric_is_connected(void)
{
  return conectric_status;
}
