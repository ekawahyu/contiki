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

#ifdef SINK_CONF_ENTRIES
#define NUM_SINK_ENTRIES SINK_CONF_ENTRIES
#else /* SINK_CONF_ENTRIES */
#define NUM_SINK_ENTRIES 8
#endif /* SINK_CONF_ENTRIES */

#ifdef SINK_CONF_DEFAULT_LIFETIME
#define DEFAULT_LIFETIME SINK_CONF_DEFAULT_LIFETIME
#else /* SINK_CONF_DEFAULT_LIFETIME */
#define DEFAULT_LIFETIME 60
#endif /* SINK_CONF_DEFAULT_LIFETIME */

#define SINK_NETBC  0xBCBC

LIST(sink_table);
MEMB(sink_mem, struct sink_entry, NUM_SINK_ENTRIES);

static struct ctimer t;

static int max_time = DEFAULT_LIFETIME;

struct sink_msg {
  uint16_t netbc;
};

/*---------------------------------------------------------------------------*/
static void
periodic(void *ptr)
{
  struct sink_entry *e;

  for(e = list_head(sink_table); e != NULL; e = list_item_next(e)) {
    e->time++;
    if(e->time >= max_time) {
      PRINTF("sink periodic: removing entry to %d.%d with cost %d\n",
       e->addr.u8[0], e->addr.u8[1],
       e->cost);
      list_remove(sink_table, e);
      memb_free(&sink_mem, e);
    }
  }

  ctimer_set(&t, CLOCK_SECOND, periodic, NULL);
}
/*---------------------------------------------------------------------------*/
void
sink_init(void)
{
  list_init(sink_table);
  memb_init(&sink_mem);

  ctimer_set(&t, CLOCK_SECOND, periodic, NULL);
}
/*---------------------------------------------------------------------------*/
struct sink_entry *
sink_lookup(const linkaddr_t *addr)
{
  struct sink_entry *e;
  uint8_t lowest_cost;
  struct sink_entry *best_entry;

  lowest_cost = -1;
  best_entry = NULL;

  /* Find the route with the lowest cost. */
  if (addr == NULL) {
    if (list_length(sink_table) != 0) {
      for (e = list_head(sink_table); e != NULL; e = list_item_next(e)) {
        if(e->cost < lowest_cost) {
          best_entry = e;
          lowest_cost = e->cost;
        }
      }
    }
  }
  else {
    for (e = list_head(sink_table); e != NULL; e = list_item_next(e)) {
      if(linkaddr_cmp(addr, &e->addr)) {
        if(e->cost < lowest_cost) {
          best_entry = e;
          lowest_cost = e->cost;
        }
      }
    }
  }
  return best_entry;
}
/*---------------------------------------------------------------------------*/
int
sink_add(const linkaddr_t *addr, uint8_t cost)
{
  struct sink_entry *e, *oldest = NULL;

  /* Avoid inserting duplicate entries. */
  e = sink_lookup(addr);
  if(e != NULL && linkaddr_cmp(&e->addr, addr)) {
    list_remove(sink_table, e);
  } else {
    /* Allocate a new entry or reuse the oldest entry with highest cost. */
    e = memb_alloc(&sink_mem);
    if(e == NULL) {
      /* Remove oldest entry. */
      for(e = list_head(sink_table); e != NULL; e = list_item_next(e)) {
        if(oldest == NULL || e->time >= oldest->time) {
          oldest = e;
        }
      }
      e = oldest;
      list_remove(sink_table, e);

      PRINTF("sink_add: removing entry to %d.%d with cost %d\n",
          e->addr.u8[0], e->addr.u8[1], e->cost);
    }
  }

  linkaddr_copy(&e->addr, addr);
  e->cost = cost;
  e->time = 0;

  /* New entry goes first. */
  list_push(sink_table, e);

  PRINTF("sink_add: new entry to %d.%d with cost %d\n",
      e->addr.u8[0], e->addr.u8[1], e->cost);

  return 0;
}
/*---------------------------------------------------------------------------*/
int
sink_num(void)
{
  struct sink_entry *e;
  int i = 0;

  for(e = list_head(sink_table); e != NULL; e = list_item_next(e)) {
    i++;
  }
  return i;
}
/*---------------------------------------------------------------------------*/
struct sink_entry *
sink_get(int num)
{
  struct sink_entry *e;
  int i = 0;

  for(e = list_head(sink_table); e != NULL; e = list_item_next(e)) {
    if(i == num) {
      return e;
    }
    i++;
  }
  return NULL;
}
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
      sink_add(originator, hops);
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
static const struct netflood_callbacks netflood_call = {
    netflood_received, NULL, NULL};
static const struct multihop_callbacks multihop_call = {
    multihop_received, multihop_forward};
static const struct route_discovery_callbacks route_discovery_callbacks = {
    found_route, route_timed_out};
/*---------------------------------------------------------------------------*/
void
conectric_open(struct conectric_conn *c, uint16_t channels,
	  const struct conectric_callbacks *callbacks)
{
  route_init();
  sink_init();
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
    return 0;
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
  int could_send;

  if (c->is_sink) return NULL;

  sink_available = sink_lookup(NULL);

  if (sink_available) {
    PRINTF("%d.%d: conectric_send_to_sink to %d.%d\n",
        linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
        sink_available->addr.u8[0], sink_available->addr.u8[1]);
    could_send = multihop_send(&c->multihop, &sink_available->addr);
  }
  else {
    PRINTF("%d.%d: conectric_send_to_sink will broadcast it\n",
        linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
    could_send = netflood_send(&c->netflood, c->netbc_id++);
  }

  if(!could_send) {
    PRINTF("%d.%d: conectric_send: could not send\n",
        linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
    return NULL;
  }
  if(c->cb->sent != NULL) {
    c->cb->sent(c);
  }

  return &sink_available->addr;
}
/*---------------------------------------------------------------------------*/
void
conectric_set_sink(struct conectric_conn *c, clock_time_t interval,
    uint8_t is_sink)
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
/*---------------------------------------------------------------------------*/
