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

static int conectric_status;

/*---------------------------------------------------------------------------*/
static void
abc_recv(struct abc_conn *c)
{

}
/*---------------------------------------------------------------------------*/
static void
trickle_recv(struct trickle_conn *c)
{

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

    PRINTF("data_packet_forward: queueing data, sending rreq\n");
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

  PRINTF("found_route\n");

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
static const struct abc_callbacks abc_call = { abc_recv };
static struct trickle_callbacks trickle_call = { trickle_recv };
static const struct multihop_callbacks multihop_call = { multihop_received, multihop_forward };
static const struct route_discovery_callbacks route_discovery_callbacks = { found_route, route_timed_out };
/*---------------------------------------------------------------------------*/
void
conectric_open(struct conectric_conn *c, uint16_t channels,
	  const struct conectric_callbacks *callbacks)
{
  route_init();
  abc_open(&c->abc, channels, &abc_call);
  trickle_open(&c->trickle, CLOCK_SECOND, channels + 1, &trickle_call);
  multihop_open(&c->multihop, channels + 2, &multihop_call);
  route_discovery_open(&c->route_discovery_conn,
		       CLOCK_SECOND/8,
		       channels + 3,
		       &route_discovery_callbacks);
  c->cb = callbacks;
}
/*---------------------------------------------------------------------------*/
void
conectric_close(struct conectric_conn *c)
{
  abc_close(&c->abc);
  trickle_close(&c->trickle);
  multihop_close(&c->multihop);
  route_discovery_close(&c->route_discovery_conn);
}
/*---------------------------------------------------------------------------*/
int
conectric_send(struct conectric_conn *c, const linkaddr_t *to)
{
  int could_send;

  PRINTF("%d.%d: conectric_send to %d.%d\n",
	 linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
	 to->u8[0], to->u8[1]);
  
  could_send = multihop_send(&c->multihop, to);

  if(!could_send) {
    PRINTF("conectric_send: could not send\n");
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
