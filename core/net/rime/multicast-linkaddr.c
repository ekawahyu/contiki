/*
 * multicast-linkaddr.c
 *
 * Created on: Aug 3, 2018
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
#include "net/rime/multicast-linkaddr.h"

#include <stdio.h>

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#ifdef MULTICAST_LINKADDR_CONF_ENTRIES
#define NUM_MULTICAST_LINKADDR_ENTRIES MULTICAST_LINKADDR_CONF_ENTRIES
#else /* MULTICAST_LINKADDR_CONF_ENTRIES */
#define NUM_MULTICAST_LINKADDR_ENTRIES 8
#endif /* MULTICAST_LINKADDR_CONF_ENTRIES */

LIST(multicast_linkaddr_table);
MEMB(multicast_linkaddr_mem, struct multicast_linkaddr_entry, NUM_MULTICAST_LINKADDR_ENTRIES);

/*---------------------------------------------------------------------------*/
void
multicast_linkaddr_init(void)
{
  list_init(multicast_linkaddr_table);
  memb_init(&multicast_linkaddr_mem);
}
/*---------------------------------------------------------------------------*/
struct multicast_linkaddr_entry *
multicast_linkaddr_lookup(uint16_t multicast_group, const linkaddr_t *addr)
{
  struct multicast_linkaddr_entry *e = NULL;

  if (addr != NULL) {
//    if (list_length(multicast_linkaddr_table) != 0) {
      for (e = list_head(multicast_linkaddr_table); e != NULL; e = list_item_next(e)) {
        if(linkaddr_cmp(addr, &e->addr)) {
          if(e->multicast_group == multicast_group) {
            return e;
          }
        }
      }
//    }
  }

  return e;
}
/*---------------------------------------------------------------------------*/
int
multicast_linkaddr_register(uint16_t multicast_group, const linkaddr_t *addr)
{
  struct multicast_linkaddr_entry *e = NULL;

  /* Avoid inserting duplicate entries. */
  e = multicast_linkaddr_lookup(multicast_group, addr);

  if(e == NULL) {
    /* Allocate a new entry */
    e = memb_alloc(&multicast_linkaddr_mem);
    if(e != NULL) {
      linkaddr_copy(&e->addr, addr);
      e->multicast_group = multicast_group;
      list_push(multicast_linkaddr_table, e);
      PRINTF("multicast_linkaddr_register: new entry %d.%d registered to %x\n",
          addr->u8[0], addr->u8[1], multicast_group);
    }
    else {
      /* Failed to register link address entry */
      return 1;
    }
  }

  return 0;
}
/*---------------------------------------------------------------------------*/
int
multicast_linkaddr_remove(uint16_t multicast_group, const linkaddr_t *addr)
{
  struct multicast_linkaddr_entry *e = NULL;

  /* Avoid inserting duplicate entries. */
  e = multicast_linkaddr_lookup(multicast_group, addr);

  if(e != NULL) {
    list_remove(multicast_linkaddr_table, e);
  }

  return 0;
}
/*---------------------------------------------------------------------------*/
int
multicast_linkaddr_num(void)
{
  struct multicast_linkaddr_entry *e;
  int i = 0;

  for(e = list_head(multicast_linkaddr_table); e != NULL; e = list_item_next(e)) {
    i++;
  }
  return i;
}
/*---------------------------------------------------------------------------*/
struct multicast_linkaddr_entry *
multicast_linkaddr_get(int num)
{
  struct multicast_linkaddr_entry *e;
  int i = 0;

  for(e = list_head(multicast_linkaddr_table); e != NULL; e = list_item_next(e)) {
    if(i == num) {
      return e;
    }
    i++;
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
