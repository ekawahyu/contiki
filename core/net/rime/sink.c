/*
 * sink.c
 *
 * Created on: Sep 18, 2018
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
#include "rime.h"
#include "sink.h"
#include "lib/list.h"
#include "lib/memb.h"

#define DUPLICATE_SINK_ENTRIES_INTERVAL (10) /* times the interval of sink periodic */

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#ifdef SINK_CONF_ENTRIES
#define NUM_SINK_ENTRIES SINK_CONF_ENTRIES
#else /* SINK_CONF_ENTRIES */
#define NUM_SINK_ENTRIES 8
#endif /* SINK_CONF_ENTRIES */

#ifdef SINK_CONF_DEFAULT_LIFETIME
#define DEFAULT_LIFETIME SINK_CONF_DEFAULT_LIFETIME
#else /* SINK_CONF_DEFAULT_LIFETIME */
#define DEFAULT_LIFETIME 60
#endif /* SINK_CONF_DEFAULT_LIFETIME */

LIST(sink_table);
MEMB(sink_mem, struct sink_entry, NUM_SINK_ENTRIES);

static struct ctimer t;
static int max_time = DEFAULT_LIFETIME;

/*---------------------------------------------------------------------------*/
static void
periodic(void *ptr)
{
  struct sink_entry *e;

  for(e = list_head(sink_table); e != NULL; e = list_item_next(e)) {
    e->time++;
    if(e->time >= max_time) {
      PRINTF("sink periodic: removing entry to %d.%d with cost %d\n",
       e->addr.u8[0], e->addr.u8[1],
       e->cost);
      list_remove(sink_table, e);
      memb_free(&sink_mem, e);
    }
  }

  ctimer_set(&t, CLOCK_SECOND, periodic, NULL);
}
/*---------------------------------------------------------------------------*/
void
sink_init(void)
{
  list_init(sink_table);
  memb_init(&sink_mem);

  ctimer_set(&t, CLOCK_SECOND, periodic, NULL);
}
/*---------------------------------------------------------------------------*/
struct sink_entry *
sink_lookup(const linkaddr_t *addr)
{
  struct sink_entry *e;
  uint8_t lowest_cost;
  struct sink_entry *best_entry;

  lowest_cost = -1;
  best_entry = NULL;

  /* Find the route with the lowest cost. */
  if (addr == NULL) {
    if (list_length(sink_table) != 0) {
      for (e = list_head(sink_table); e != NULL; e = list_item_next(e)) {
        if(e->cost < lowest_cost) {
          best_entry = e;
          lowest_cost = e->cost;
        }
      }
    }
  }
  else {
    for (e = list_head(sink_table); e != NULL; e = list_item_next(e)) {
      if(linkaddr_cmp(addr, &e->addr)) {
        if(e->cost < lowest_cost) {
          best_entry = e;
          lowest_cost = e->cost;
        }
      }
    }
  }
  return best_entry;
}
/*---------------------------------------------------------------------------*/
int
sink_add(const linkaddr_t *addr, uint8_t cost)
{
  struct sink_entry *e, *oldest = NULL;

  /* Avoid inserting duplicate entries. */
  e = sink_lookup(addr);
  if(e != NULL && linkaddr_cmp(&e->addr, addr)) {
    /* Compare cost of duplicate entries and keep the lowest one */
    if (e->cost < cost && e->time < DUPLICATE_SINK_ENTRIES_INTERVAL) cost = e->cost;
    list_remove(sink_table, e);
  } else {
    /* Allocate a new entry or reuse the oldest entry with highest cost. */
    e = memb_alloc(&sink_mem);
    if(e == NULL) {
      /* Remove oldest entry. */
      for(e = list_head(sink_table); e != NULL; e = list_item_next(e)) {
        if(oldest == NULL || e->time >= oldest->time) {
          oldest = e;
        }
      }
      e = oldest;
      list_remove(sink_table, e);

      PRINTF("sink_add: removing entry to %d.%d with cost %d\n",
          e->addr.u8[0], e->addr.u8[1], e->cost);
    }
  }

  linkaddr_copy(&e->addr, addr);
  e->cost = cost;
  e->time = 0;

  /* New entry goes first. */
  list_push(sink_table, e);

  PRINTF("sink_add: new entry to %d.%d with cost %d\n",
      e->addr.u8[0], e->addr.u8[1], e->cost);

  return 0;
}
/*---------------------------------------------------------------------------*/
int
sink_num(void)
{
  struct sink_entry *e;
  int i = 0;

  for(e = list_head(sink_table); e != NULL; e = list_item_next(e)) {
    i++;
  }
  return i;
}
/*---------------------------------------------------------------------------*/
struct sink_entry *
sink_get(int num)
{
  struct sink_entry *e;
  int i = 0;

  for(e = list_head(sink_table); e != NULL; e = list_item_next(e)) {
    if(i == num) {
      return e;
    }
    i++;
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
