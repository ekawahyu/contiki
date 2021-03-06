/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         Implementation of the architecture-agnostic parts of the real-time timer module.
 * \author
 *         Adam Dunkels <adam@sics.se>
 *
 */

/**
 * \addtogroup rt
 * @{
 */

#include "sys/rtimer.h"
#include "contiki.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static struct rtimer *next_rtimer = NULL;

/*---------------------------------------------------------------------------*/
void
rtimer_init(void)
{
  rtimer_arch_init();
}
#if RTIMER_CONF_MULTIPLE
/*---------------------------------------------------------------------------*/
int
rtimer_set(struct rtimer *rtimer, rtimer_clock_t time,
	   rtimer_clock_t duration,
	   rtimer_callback_t func, void *ptr)
{
  struct rtimer *t;
  rtimer_clock_t rt_now, rt1, rt2;
  int first = 0;

  rtimer_arch_halt();

  PRINTF("rtimer_set time %d\n", time);

  rt_now = RTIMER_NOW();

  rtimer->func = func;
  rtimer->ptr = ptr;
  rtimer->time = time;
  rtimer->next = NULL;

  /* FIXME there can be only one rtimer value associated to one interrupt event.
   * increment it by one for additional 64us to the assigned value.
   */
  t = next_rtimer;
  while (t) {
	  if (t->time == rtimer->time) rtimer->time++;
	  t = t->next;
  }

  if(next_rtimer == NULL) {
    first = 1;
    next_rtimer = rtimer;
  }
  else {
    t = next_rtimer;

    /* if the new rtimer will occur earlier than the one currently running, schedule it */
    if (rt_now > rtimer->time)
      rt1 = rtimer->time + ~rt_now + 1;
    else
      rt1 = rtimer->time - rt_now;

    if (rt_now > t->time)
      rt2 = t->time + ~rt_now + 1;
    else
      rt2 = t->time - rt_now;

    if (rt1 < rt2) {
      first = 1;
      rtimer->next = t;
      next_rtimer = rtimer;
    }

    /* otherwise try to put it in order within the list */
    else {
      do {
        rtimer->next = t->next;

        if (rt_now > rtimer->time)
          rt1 = rtimer->time + ~rt_now + 1;
        else
          rt1 = rtimer->time - rt_now;

        if (rt_now > t->next->time)
          rt2 = t->next->time + ~rt_now + 1;
        else
          rt2 = t->next->time - rt_now;

        if ((rt1 < rt2) || (rtimer->next == NULL)) {
          t->next = rtimer;
          break;
        }
        else {
          t = rtimer->next;
        }
      } while(rtimer->next);
    }

  }

  if(first == 1) {
    rtimer_arch_schedule(next_rtimer->time);
  }

  rtimer_arch_continue();

  return RTIMER_OK;
}
#else
/*---------------------------------------------------------------------------*/
int
rtimer_set(struct rtimer *rtimer, rtimer_clock_t time,
     rtimer_clock_t duration,
     rtimer_callback_t func, void *ptr)
{
  int first = 0;

  PRINTF("rtimer_set time %d\n", time);

  if(next_rtimer == NULL) {
    first = 1;
  }

  rtimer->func = func;
  rtimer->ptr = ptr;

  rtimer->time = time;
  next_rtimer = rtimer;

  if(first == 1) {
    rtimer_arch_schedule(time);
  }
  return RTIMER_OK;
}
#endif
/*---------------------------------------------------------------------------*/
void
rtimer_run_next(void)
{
  struct rtimer *t;
  if(next_rtimer == NULL) {
    return;
  }
  t = next_rtimer;
#if RTIMER_CONF_MULTIPLE
  next_rtimer = t->next;
#else
  next_rtimer = NULL;
#endif
  t->func(t, t->ptr);
  if(next_rtimer != NULL) {
    rtimer_arch_schedule(next_rtimer->time);
  }
  return;
}
/*---------------------------------------------------------------------------*/
int
rtimer_is_scheduled(void)
{
  if (next_rtimer == NULL) return 0;
  else return 1;
}
/*---------------------------------------------------------------------------*/
/** @}*/
