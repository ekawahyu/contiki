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
#include "net/rime/broadcast.h"
#include "net/rime/multicast.h"
#include "net/rime/multicast-linkaddr.h"

struct sink_entry {
  struct sink_entry *next;
  linkaddr_t addr;
  uint8_t time;
  uint8_t cost;
};

struct sink_entry * sink_lookup(const linkaddr_t *addr);
int sink_add(const linkaddr_t *addr, uint8_t cost);
int sink_num(void);
struct sink_entry * sink_get(int num);

struct conectric_conn;

struct conectric_callbacks {
  /** Called when a packet is received. */
  void (* recv)(struct conectric_conn *c, const linkaddr_t *from, uint8_t hops);
  /** Called when a packet, sent with conectric_send(), is actually transmitted. */
  void (* sent)(struct conectric_conn *c);
  /** Called when a local broadcast is received. */
  void (* localbroadcast_recv)(struct conectric_conn *c, const linkaddr_t *from);
  /** Called when a network-wide broadcast is received. */
  void (* netbroadcast_recv)(struct conectric_conn *c, const linkaddr_t *from, uint8_t hops);
  /** Called when a network-wide sink broadcast is received. */
  void (* sink_recv)(struct conectric_conn *c, const linkaddr_t *from, uint8_t hops);
};

struct conectric_conn {
  struct broadcast_conn broadcast;
  struct multicast_conn netbc;
  struct multicast_conn netuc;
  struct multihop_conn multihop;
  struct route_discovery_conn route_discovery_conn;
  struct queuebuf *queued_data;
  linkaddr_t queued_data_dest;
  const struct conectric_callbacks *cb;
  struct ctimer interval_timer;
  clock_time_t interval;
};

void conectric_open(struct conectric_conn *c, uint16_t channels, const struct conectric_callbacks *callbacks);
void conectric_close(struct conectric_conn *c);
void conectric_set_sink(struct conectric_conn *c, clock_time_t interval, uint8_t sink);
void conectric_set_collect(struct conectric_conn *c, uint8_t collect);
int conectric_send(struct conectric_conn *c, const linkaddr_t *dest);
linkaddr_t * conectric_send_to_sink(struct conectric_conn *c);
uint8_t conectric_is_sink(void);
uint8_t conectric_is_collect(void);

#endif /* CONECTRIC_H_ */
