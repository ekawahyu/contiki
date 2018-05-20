/**
 * \file
 *         Tests related to clocks and timers
 *         This is based on clock_test.c from the original sensinode port
 *
 * \author
 *         Zach Shelby <zach@sensinode.com> (Original)
 *         George Oikonomou - <oikonomou@users.sourceforge.net> (rtimer code)
 *
 */

#include "contiki.h"
#include "sys/clock.h"
#include "sys/rtimer.h"
#include "dev/leds.h"

#include <stdio.h>
/*---------------------------------------------------------------------------*/
#define TEST_CLOCK_DELAY_USEC 1
#define TEST_RTIMER           1
#define TEST_ETIMER           1
#define TEST_CLOCK_SECONDS    1
/*---------------------------------------------------------------------------*/
static struct etimer et;
extern volatile uint8_t deep_sleep_requested;

#if TEST_CLOCK_DELAY_USEC
static rtimer_clock_t start_count, end_count, diff;
#endif

#if TEST_CLOCK_SECONDS
static unsigned long sec;
#endif

#if TEST_ETIMER
static clock_time_t count;
#endif

#if TEST_RTIMER
static struct rtimer rt[5];
rtimer_clock_t rt_now, rt_for;
static clock_time_t ct;
#endif

static uint8_t i;
/*---------------------------------------------------------------------------*/
PROCESS(clock_test_process, "Clock test process");
AUTOSTART_PROCESSES(&clock_test_process);
/*---------------------------------------------------------------------------*/
#if TEST_RTIMER
void
rt_callback(struct rtimer *t, void *ptr)
{
  rt_now = RTIMER_NOW();
  ct = clock_time();
  printf("Task called at %u (clock = %u)\n", rt_now, ct);
}
#endif
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(clock_test_process, ev, data)
{

  PROCESS_BEGIN();

  etimer_set(&et, 2 * CLOCK_SECOND);

  PROCESS_YIELD();

#if TEST_CLOCK_DELAY_USEC
  printf("clock_delay_usec test, (1000 x i) usec:\n");
  i = 1;
  while(i < 7) {
    start_count = RTIMER_NOW();
    clock_delay_usec(1000 * i);
    end_count = RTIMER_NOW();
    diff = end_count - start_count;
    printf("Requested: %u usec, Real: %u rtimer ticks = ~%u us\n",
        1000 * i, diff, diff * 64);
    i++;
  }
#endif

#if TEST_RTIMER
  printf("Rtimer Test, 1 sec (%u rtimer ticks):\n", RTIMER_SECOND);
  i = 0;
  while(i < 5) {
    etimer_set(&et, CLOCK_SECOND/8);
    printf("=======================\n");
    ct = clock_time();
    rt_now = RTIMER_NOW();
    // rt_for = rt_now + (5-i) * (RTIMER_SECOND/2);
    rt_for = rt_now + 750;
    printf("Now=%u - For=%u\n", rt_now, rt_for);
    if(rtimer_set(&rt[i], rt_for, 1,
              (void (*)(struct rtimer *, void *))rt_callback, NULL) != RTIMER_OK) {
      printf("Error setting\n");
    }

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    i++;
  }
  etimer_set(&et, 3 * CLOCK_SECOND);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
#endif

#if TEST_ETIMER
  printf("Clock tick and etimer test, 1 sec (%u clock ticks):\n", CLOCK_SECOND);
  i = 0;
  while(i < 10) {
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    etimer_reset(&et);

    count = clock_time();
    printf("%u ticks\n", count);

    leds_toggle(LEDS_RED);
    i++;
  }
#endif

#if TEST_CLOCK_SECONDS
  printf("Clock seconds test (5s):\n");
  i = 0;
  while(i < 10) {
    etimer_set(&et, 5 * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    etimer_reset(&et);

    sec = clock_seconds();
    printf("%lu seconds\n", sec);

    leds_toggle(LEDS_GREEN);
    i++;
  }
#endif

  printf("Done!\n");

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void invoke_process_before_sleep(void)
{
  deep_sleep_requested = 5;
}
/*---------------------------------------------------------------------------*/
