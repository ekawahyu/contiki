/*
 * multicast.h
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

#ifndef MULTICAST_H_
#define MULTICAST_H_

#include "net/rime/iburst.h"
#include "net/queuebuf.h"
#include "sys/ctimer.h"

extern const linkaddr_t multicast_node_addr;
extern const linkaddr_t multicast_router_addr;

struct multicast_conn;

#define MULTICAST_ATTRIBUTES  { PACKETBUF_ADDR_ERECEIVER, PACKETBUF_ADDRSIZE }, IBURST_ATTRIBUTES

struct multicast_callbacks {
  void (* recv)(struct multicast_conn *c, const linkaddr_t * originator);
  void (* sent)(struct multicast_conn *c);
};

struct multicast_conn {
  struct iburst_conn c;
  const struct multicast_callbacks *cb;
  struct queuebuf *q;
  uint16_t channel;
};

void multicast_open(struct multicast_conn *c, uint16_t channel, const struct multicast_callbacks *cb);
void multicast_close(struct multicast_conn *c);
void multicast_send(struct multicast_conn *c, linkaddr_t *dest);

#endif /* MULTICAST_H_ */
