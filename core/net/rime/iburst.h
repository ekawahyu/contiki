/*
 * iburst.h
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

#ifndef IBURST_H_
#define IBURST_H_

#include "net/rime/broadcast.h"
#include "net/queuebuf.h"
#include "sys/ctimer.h"

struct iburst_conn;

#define IBURST_ATTRIBUTES { PACKETBUF_ADDR_ESENDER, PACKETBUF_ADDRSIZE }, \
                          { PACKETBUF_ATTR_EPACKET_ID, PACKETBUF_ATTR_BIT * 8 },\
                          { PACKETBUF_ATTR_HOPS, PACKETBUF_ATTR_BIT * 8 }, \
                          BROADCAST_ATTRIBUTES

struct iburst_callbacks {
  void (* recv)(struct iburst_conn *c, const linkaddr_t * originator, const linkaddr_t * sender, uint8_t hops);
  void (* sent)(struct iburst_conn *c);
  void (* dropped)(struct iburst_conn *c);
};

struct iburst_conn {
  struct broadcast_conn c;
  const struct iburst_callbacks *cb;
  struct ctimer t;
  struct queuebuf *q;
  const linkaddr_t * originator;
  const linkaddr_t * sender;
  clock_time_t interval;
  uint16_t seqno;
  uint8_t hops;
  uint8_t burstcnt; /* for internal use, burst counter */
  uint8_t burstmax;
};

void iburst_open(struct iburst_conn *c, uint16_t channel, clock_time_t interval,
    uint8_t burstmax, const struct iburst_callbacks *cb);
void iburst_close(struct iburst_conn *c);
int  iburst_send(struct iburst_conn *c, clock_time_t interval, uint8_t burstmax);
void iburst_cancel(struct iburst_conn *c);

#endif /* IBURST_H_ */
