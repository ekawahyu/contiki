/*
 * multicast.c
 *
 * Created on: Aug 2, 2018
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

#include "sys/cc.h"
#include "net/rime/rime.h"
#include "net/rime/multicast.h"
#include "net/rime/multicast-linkaddr.h"
#include "lib/random.h"

#include <stdio.h>

const linkaddr_t multicast_node_addr = {{0x00, 0x01}};
const linkaddr_t multicast_router_addr = {{0x00, 0x02}};

static const struct packetbuf_attrlist attributes[] =
  {
    MULTICAST_ATTRIBUTES PACKETBUF_ATTR_LAST
  };

#define IBURST_INTERVAL (CLOCK_SECOND / 16)
#define IBURST_COUNT    5

/*---------------------------------------------------------------------------*/
static void
recv(struct iburst_conn *ib, const linkaddr_t * originator, const linkaddr_t * sender, uint8_t hops)
{
  struct multicast_conn *c = (struct multicast_conn *)ib;
  const linkaddr_t *esender = packetbuf_addr(PACKETBUF_ADDR_ESENDER);
  const linkaddr_t *ereceiver = packetbuf_addr(PACKETBUF_ADDR_ERECEIVER);

  if(multicast_linkaddr_lookup(c->channel, ereceiver)) {
    if(c->cb->recv) {
      c->cb->recv(c, esender);
    }
  }

//  if(ereceiver->u8[0] == 0x00 && ereceiver->u8[1] == 0x01) {
//    if(c->cb->recv) {
//      c->cb->recv(c, esender);
//    }
//  }
//
//  if(ereceiver->u8[0] == 0x00 && ereceiver->u8[1] == 0x02 && c->is_router) {
//    if(c->cb->recv) {
//      c->cb->recv(c, esender);
//    }
//  }
}
/*---------------------------------------------------------------------------*/
static void
sent(struct iburst_conn *c)
{

}
/*---------------------------------------------------------------------------*/
static void
dropped(struct iburst_conn *c)
{

}
/*---------------------------------------------------------------------------*/
static const struct iburst_callbacks iburst = { recv, sent, dropped };
/*---------------------------------------------------------------------------*/
void
multicast_open(struct multicast_conn *c, uint16_t channel,
	  const struct multicast_callbacks *cb)
{
  iburst_open(&c->c, channel, IBURST_INTERVAL, IBURST_COUNT, &iburst);
  c->channel = channel;
  c->cb = cb;
  channel_set_attributes(channel, attributes);
}
/*---------------------------------------------------------------------------*/
void
multicast_close(struct multicast_conn *c)
{
  iburst_close(&c->c);
  if(c->q != NULL) {
    queuebuf_free(c->q);
    c->q = NULL;
  }
}
/*---------------------------------------------------------------------------*/
void
multicast_send(struct multicast_conn *c, linkaddr_t *dest)
{
  packetbuf_set_addr(PACKETBUF_ADDR_ERECEIVER, dest);
  if (iburst_send(&c->c, IBURST_INTERVAL, IBURST_COUNT)) {
    if(c->cb->sent) {
      c->cb->sent(c);
    }
  }
}
/*---------------------------------------------------------------------------*/
