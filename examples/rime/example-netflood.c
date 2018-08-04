
#include "net/rime/netflood.h"
#include "contiki.h"
#include "lib/random.h"

#include "dev/button-sensor.h"

#include <stdio.h>

/*---------------------------------------------------------------------------*/
PROCESS(example_netflood_process, "");
AUTOSTART_PROCESSES(&example_netflood_process);
/*---------------------------------------------------------------------------*/
static int
recv(struct netflood_conn *c, const linkaddr_t *from,
    const linkaddr_t *originator, uint8_t seqno, uint8_t hops)
{
  printf("recv from %d.%d (prev:%d.%d): '%s'\n",
      originator->u8[0], originator->u8[1],
      from->u8[0], from->u8[1],
      (char *)packetbuf_dataptr());

  /* Continue flooding the network */
  return 1;
}
static void
sent(struct netflood_conn *c)
{
  printf("sent\n");
}
static void
dropped(struct netflood_conn *c)
{
  printf("dropped\n");
}
static const struct netflood_callbacks netflood_call = {recv, sent, dropped};
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_netflood_process, ev, data)
{
  static struct netflood_conn c;
  static uint8_t seqno = 1;

  PROCESS_EXITHANDLER(netflood_close(&c));
  
  PROCESS_BEGIN();

  netflood_open(&c, (CLOCK_SECOND / 32 + random_rand() % (CLOCK_SECOND / 32)), 136, &netflood_call);

  SENSORS_ACTIVATE(button_sensor);

  while(1) {
    /* Wait for button click before sending the first message. */
    PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event && data == &button_sensor);

    packetbuf_copyfrom("Hello from Conectric to all of sensors, good morning everyone", 62);
    netflood_send(&c, seqno++);
    if (seqno > 255) seqno = 1;
  }
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
