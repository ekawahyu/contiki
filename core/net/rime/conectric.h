/*
 * conectric.h
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

#ifndef CONECTRIC_H_
#define CONECTRIC_H_

#include "net/queuebuf.h"
#include "net/rime/abc.h"
#include "net/rime/netflood.h"
#include "net/rime/trickle.h"
#include "net/rime/multihop.h"
#include "net/rime/route-discovery.h"

struct conectric_conn;

struct conectric_callbacks {
  /** Called when a packet is received. */
  void (* recv)(struct conectric_conn *c, const linkaddr_t *from, uint8_t hops);
  /** Called when a packet, sent with conectric_send(), is actually transmitted. */
  void (* sent)(struct conectric_conn *c);
  /** Called when a packet, sent with conectric_send(), times out and is dropped. */
  void (* timedout)(struct conectric_conn *c);
};

struct conectric_conn {
  struct netflood_conn netflood;
  struct multihop_conn multihop;
  struct route_discovery_conn route_discovery_conn;
  struct queuebuf *queued_data;
  linkaddr_t queued_data_dest;
  const struct conectric_callbacks *cb;
};

void conectric_open(struct conectric_conn *c, uint16_t channels, const struct conectric_callbacks *callbacks);
void conectric_close(struct conectric_conn *c);
int conectric_send(struct conectric_conn *c, const linkaddr_t *dest);
int conectric_ready(struct conectric_conn *c);

#endif /* CONECTRIC_H_ */
