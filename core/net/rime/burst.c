/*
 * burst.c
 *
 * Created on: Jul 24, 2018
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
#include "net/rime/burst.h"
#include "lib/random.h"

#include <stdio.h>

#define SEQNO_LT(a, b) ((int16_t)((a) - (b)) <= 0 && (int16_t)((b) - (a)) <= 5 && (((a) != (0)) || ((b) != (255))))

static const struct packetbuf_attrlist attributes[] =
  {
    BURST_ATTRIBUTES PACKETBUF_ATTR_LAST
  };

/*---------------------------------------------------------------------------*/
static void
send(void *ptr)
{
  struct burst_conn *c = ptr;

  if(c->q != NULL) {
    queuebuf_to_packetbuf(c->q);
    packetbuf_set_attr(PACKETBUF_ATTR_EPACKET_ID, c->seqno);
    abc_send(&c->c);
    if(c->cb->sent) {
      c->cb->sent(c);
    }
    if (++c->burstcnt < c->burstmax) {
      ctimer_set(&c->t, c->interval / 2 + (random_rand() % (c->interval / 2)), send, c);
    }
    else {
      c->burstcnt = 0;
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
fwd(struct abc_conn *abc)
{
  struct burst_conn *c = (struct burst_conn *)abc;

  if(c->cb->recv) {
    c->cb->recv(c, c->originator);
  }
  if(c->q != NULL) {
    queuebuf_free(c->q);
    c->q = NULL;
  }
  c->burstcnt = c->burstmax - c->burstfwd;
  if (c->burstcnt != c->burstmax) {
//  if(c->burstfwd--) {
    c->q = queuebuf_new_from_packetbuf();
    ctimer_set(&c->t, c->interval / 2 + (random_rand() % (c->interval / 2)), send, c);
    if(c->cb->sent) {
      c->cb->sent(c);
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
recv(struct abc_conn *abc)
{
  struct burst_conn *c = (struct burst_conn *)abc;
  uint16_t seqno = packetbuf_attr(PACKETBUF_ATTR_EPACKET_ID);

  c->originator = packetbuf_addr(PACKETBUF_ADDR_ESENDER);

  if(seqno == c->seqno) {
    if(c->cb->dropped) {
      c->cb->dropped(c);
    }
  }
  else if((int16_t)((seqno) - (c->seqno)) < 0) {
    c->seqno = seqno;
    fwd((struct abc_conn *)c);
  }
  else {
    c->seqno = seqno;
    fwd((struct abc_conn *)c);
  }
}
/*---------------------------------------------------------------------------*/
static void
sent(struct abc_conn *c, int status, int num_tx)
{

}
/*---------------------------------------------------------------------------*/
static const struct abc_callbacks abc = { recv, sent };
/*---------------------------------------------------------------------------*/
void
burst_open(struct burst_conn *c, uint16_t channel, uint8_t burstmax,
    clock_time_t interval, const struct burst_callbacks *cb)
{
  abc_open(&c->c, channel, &abc);
  c->seqno = 253;
  c->burstcnt = 0;
  c->burstfwd = 1;
  c->burstmax = burstmax;
  c->interval = interval;
  c->cb = cb;
  channel_set_attributes(channel, attributes);
}
/*---------------------------------------------------------------------------*/
void
burst_close(struct burst_conn *c)
{
  abc_close(&c->c);
  ctimer_stop(&c->t);
  if(c->q != NULL) {
    queuebuf_free(c->q);
    c->q = NULL;
  }
}
/*---------------------------------------------------------------------------*/
int
burst_send(struct burst_conn *c, clock_time_t interval, uint8_t fwdburstcnt)
{
  if(c->q != NULL) {
    /* If we are already about to send a packet, we cancel the old one. */
    queuebuf_free(c->q);
    c->q = NULL;
  }
  c->q = queuebuf_new_from_packetbuf();
  c->interval = interval;

  if(c->burstfwd <= c->burstmax) {
    c->burstfwd = fwdburstcnt;
  }
  else {
    c->burstfwd = c->burstmax;
  }

  /* update packet's sequence number */
  c->seqno++;
  if(c->seqno > 255) c->seqno = 1;
  packetbuf_set_attr(PACKETBUF_ATTR_EPACKET_ID, c->seqno);
  packetbuf_set_addr(PACKETBUF_ADDR_ESENDER, &linkaddr_node_addr);

  /* send one packet immediately */
  c->burstcnt = 1;
  abc_send(&c->c);
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
burst_cancel(struct burst_conn *c)
{
  ctimer_stop(&c->t);
  if(c->q != NULL) {
    queuebuf_free(c->q);
    c->q = NULL;
  }
}
/*---------------------------------------------------------------------------*/
