/*
 * iburst.c
 *
 * Created on: Jul 26, 2018
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
#include "net/rime/iburst.h"
#include "lib/random.h"

#include <stdio.h>

static const struct packetbuf_attrlist attributes[] =
  {
    IBURST_ATTRIBUTES PACKETBUF_ATTR_LAST
  };

/*---------------------------------------------------------------------------*/
static void
send(void *ptr)
{
  struct iburst_conn *c = ptr;

  if(c->q != NULL) {
    queuebuf_to_packetbuf(c->q);
    packetbuf_set_attr(PACKETBUF_ATTR_EPACKET_ID, c->seqno);
    broadcast_send(&c->c);
    if(c->cb->sent) {
      c->cb->sent(c);
    }
    if (++c->burstcnt < c->burstmax) {
      ctimer_set(&c->t, c->interval / 2 + (random_rand() % (c->interval / 2)), send, c);
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
fwd(struct broadcast_conn *broadcast)
{
  struct iburst_conn *c = (struct iburst_conn *)broadcast;

  c->burstcnt = 0;

  if(c->q != NULL) {
    queuebuf_free(c->q);
    c->q = NULL;
  }
  c->q = queuebuf_new_from_packetbuf();
  packetbuf_set_attr(PACKETBUF_ATTR_HOPS, c->hops);
  if (++c->burstcnt < c->burstmax) {
    ctimer_set(&c->t, c->interval / 2 + (random_rand() % (c->interval / 2)), send, c);
  }
  if(c->cb->sent) {
    c->cb->sent(c);
  }
}
/*---------------------------------------------------------------------------*/
static void
recv(struct broadcast_conn *broadcast, const linkaddr_t *sender)
{
  struct iburst_conn *c = (struct iburst_conn *)broadcast;
  uint16_t seqno = packetbuf_attr(PACKETBUF_ATTR_EPACKET_ID);

  c->originator = packetbuf_addr(PACKETBUF_ADDR_ESENDER);
  c->hops = packetbuf_attr(PACKETBUF_ATTR_HOPS) + 1;
  c->sender = packetbuf_addr(PACKETBUF_ADDR_SENDER);

  if(seqno == c->seqno) {
    if(c->cb->dropped) {
      c->cb->dropped(c);
    }
  }
  else if((int16_t)((seqno) - (c->seqno)) < 0) {
    c->seqno = seqno;
    if(c->cb->recv) {
      c->cb->recv(c, c->sender, c->originator, c->hops);
    }
    fwd((struct broadcast_conn *)c);
  }
  else {
    c->seqno = seqno;
    if(c->cb->recv) {
      c->cb->recv(c, c->originator, c->sender, c->hops);
    }
    fwd((struct broadcast_conn *)c);
  }
}
/*---------------------------------------------------------------------------*/
static void
sent(struct broadcast_conn *c, int status, int num_tx)
{

}
/*---------------------------------------------------------------------------*/
static const struct broadcast_callbacks broadcast = { recv, sent };
/*---------------------------------------------------------------------------*/
void
iburst_open(struct iburst_conn *c, uint16_t channel, clock_time_t interval,
    uint8_t burstmax, const struct iburst_callbacks *cb)
{
  broadcast_open(&c->c, channel, &broadcast);
  c->seqno = 0;
  c->hops = 0;
  c->burstcnt = 0;
  c->burstmax = burstmax;
  c->interval = interval;
  c->cb = cb;
  channel_set_attributes(channel, attributes);
}
/*---------------------------------------------------------------------------*/
void
iburst_close(struct iburst_conn *c)
{
  broadcast_close(&c->c);
  ctimer_stop(&c->t);
  if(c->q != NULL) {
    queuebuf_free(c->q);
    c->q = NULL;
  }
}
/*---------------------------------------------------------------------------*/
int
iburst_send(struct iburst_conn *c, clock_time_t interval, uint8_t burstmax)
{
  if(c->q != NULL) {
    /* If we are already about to send a packet, we cancel the old one. */
    queuebuf_free(c->q);
    c->q = NULL;
  }
  c->q = queuebuf_new_from_packetbuf();
  c->hops = 0;
  c->interval = interval;
  c->burstmax = burstmax;

  /* update packet's sequence number */
  c->seqno++;
  if(c->seqno > 255) c->seqno = 1;
  packetbuf_set_addr(PACKETBUF_ADDR_ESENDER, &linkaddr_node_addr);
  packetbuf_set_attr(PACKETBUF_ATTR_EPACKET_ID, c->seqno);
  packetbuf_set_attr(PACKETBUF_ATTR_HOPS, c->hops);

  /* send one packet immediately */
  c->burstcnt = 1;
  broadcast_send(&c->c);
  if(c->cb->sent) {
    c->cb->sent(c);
  }

  /* set callback timer to send the rest of bursts */
  if(c->q != NULL) {
    ctimer_set(&c->t, interval / 2 + (random_rand() % (interval / 2)), send, c);
    return 1;
  }

  return 0;
}
/*---------------------------------------------------------------------------*/
void
iburst_cancel(struct iburst_conn *c)
{
  ctimer_stop(&c->t);
  if(c->q != NULL) {
    queuebuf_free(c->q);
    c->q = NULL;
  }
}
/*---------------------------------------------------------------------------*/
