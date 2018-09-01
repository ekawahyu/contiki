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
#include "random.h"

#include <stddef.h> /* For offsetof */

#define PACKET_TIMEOUT (CLOCK_SECOND * 5)
#define DUPLICATE_SINK_ENTRIES_INTERVAL (10) /* times the interval of sink periodic */

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
static uint8_t is_sink;
static uint8_t is_collect;
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
    /* Compare cost of duplicate entries and keep the lowest one */
    if (e->cost < cost && e->time < DUPLICATE_SINK_ENTRIES_INTERVAL) cost = e->cost;
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
  struct sink_msg *msg = packetbuf_dataptr();
  struct conectric_conn *c = (struct conectric_conn *)
    ((char *)mc - offsetof(struct conectric_conn, netbc));

  uint8_t hops = packetbuf_attr(PACKETBUF_ATTR_HOPS);

  PRINTF("%d.%d: netbc received from %d.%d hops %d\n",
   linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
   originator->u8[0], originator->u8[1], hops);

  if (msg->netbc == SINK_NETBC) {
    if(c->cb->sink_recv && linkaddr_cmp(originator, &linkaddr_node_addr) == 0) {
      sink_add(originator, hops);
      c->cb->sink_recv(c, originator, hops);
    }
  }
  else {
    if(c->cb->netbroadcast_recv) {
      c->cb->netbroadcast_recv(c, originator, hops);
    }
  }
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
  sink_init();
  multicast_linkaddr_init();
}
/*---------------------------------------------------------------------------*/
void
conectric_open(struct conectric_conn *c, uint16_t channels,
	  const struct conectric_callbacks *callbacks)
{
  multicast_open(&c->netbc, &multicast_node_addr, &netbc_call);
  multicast_open(&c->netbc, &multicast_router_addr, &netbc_call);
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
