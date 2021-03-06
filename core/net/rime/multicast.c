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

#include "net/rime/rime.h"
#include "net/rime/multicast.h"
#include "net/rime/multicast-linkaddr.h"

#include <stdio.h>

const struct multicast_netaddr multicast_node_addr = {
    .network.u8[0] = 0xFF,
    .network.u8[1] = 0x02,
    .host.u8[0] = 0x00,
    .host.u8[1] = 0x01
};
const struct multicast_netaddr multicast_router_addr = {
    .network.u8[0] = 0xFF,
    .network.u8[1] = 0x02,
    .host.u8[0] = 0x00,
    .host.u8[1] = 0x02
};
struct multicast_netaddr multicast_linklocal_addr = {
    .network.u8[0] = 0xFE,
    .network.u8[1] = 0x08,
    .host.u8[0] = 0x00,
    .host.u8[1] = 0x00
};

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
multicast_open(struct multicast_conn *c, const struct multicast_netaddr *na,
	  const struct multicast_callbacks *cb)
{
  /* It is supposed to be called once for the link local address setup,
   * but calling it multiple times don't hurt */
  linkaddr_copy(&multicast_linklocal_addr.host, &linkaddr_node_addr);

  /* Register multicast group/channel and scope */
  multicast_linkaddr_register(na->network.u16, &na->host);

  /* Open multicast group/channel only if it has not been done before */
  if(c->channel == 0) {
    iburst_open(&c->c, na->network.u16, IBURST_INTERVAL, IBURST_COUNT, &iburst);
    c->channel = na->network.u16;
    c->cb = cb;
    channel_set_attributes(na->network.u16, attributes);
  }
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
int
multicast_send(struct multicast_conn *c, const linkaddr_t *dest)
{
  packetbuf_set_addr(PACKETBUF_ADDR_ERECEIVER, dest);
  if (iburst_send(&c->c, IBURST_INTERVAL, IBURST_COUNT)) {
    if(c->cb->sent) {
      c->cb->sent(c);
    }
  }

  return 0;
}
/*---------------------------------------------------------------------------*/
