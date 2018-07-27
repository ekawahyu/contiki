
#include "net/rime/burst.h"
#include "contiki.h"

#include "dev/button-sensor.h"

#include <stdio.h>

/*---------------------------------------------------------------------------*/
PROCESS(example_burst_process, "");
AUTOSTART_PROCESSES(&example_burst_process);
/*---------------------------------------------------------------------------*/
static void
recv(struct burst_conn *c, const linkaddr_t * originator)
{
  printf("recv from %d.%d (seqno=%d) '%s'\n",
      originator->u8[0], originator->u8[1],
      packetbuf_attr(PACKETBUF_ATTR_EPACKET_ID),
      (char *)packetbuf_dataptr());
}
static void
sent(struct burst_conn *c)
{
  printf("sent\n");
}
static void
dropped(struct burst_conn *c)
{
  printf("dropped\n");
}
static const struct burst_callbacks callbacks = { recv, sent, dropped };
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_burst_process, ev, data)
{
  static struct burst_conn c;

  PROCESS_EXITHANDLER(burst_close(&c));
  
  PROCESS_BEGIN();

  burst_open(&c, 136, CLOCK_SECOND / 8, 5, &callbacks);

  SENSORS_ACTIVATE(button_sensor);

  while(1) {
    /* Wait for button click before sending the first message. */
    PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event && data == &button_sensor);

    packetbuf_copyfrom("BurstMe", 8);
    burst_send(&c, CLOCK_SECOND / 8, 7);
  }
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
