/*
 * multicast.h
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

#ifndef BURST_H_
#define BURST_H_

#include "net/rime/abc.h"
#include "net/queuebuf.h"
#include "sys/ctimer.h"

struct burst_conn;

#define BURST_ATTRIBUTES  { PACKETBUF_ADDR_ESENDER, PACKETBUF_ADDRSIZE }, \
                          { PACKETBUF_ATTR_EPACKET_ID, PACKETBUF_ATTR_BIT * 8 },\
                          ABC_ATTRIBUTES

struct burst_callbacks {
  void (* recv)(struct burst_conn *c, const linkaddr_t * originator);
  void (* sent)(struct burst_conn *c);
  void (* dropped)(struct burst_conn *c);
};

struct burst_conn {
  struct abc_conn c;
  const struct burst_callbacks *cb;
  struct ctimer t;
  struct queuebuf *q;
  const linkaddr_t * originator;
  clock_time_t interval;
  uint16_t seqno;
  uint8_t burstcnt; /* for internal use, burst counter */
  uint8_t burstfwd; /* forwarding burst between 1-burstmax */
  uint8_t burstmax;
};

void burst_open(struct burst_conn *c, uint16_t channel, uint8_t maxburst,
    clock_time_t interval, const struct burst_callbacks *cb);
void burst_close(struct burst_conn *c);
int  burst_send(struct burst_conn *c, clock_time_t interval, uint8_t fwdburstcnt);
void burst_cancel(struct burst_conn *c);

#endif /* burst_H_ */
