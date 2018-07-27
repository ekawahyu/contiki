
#include "net/rime/iburst.h"
#include "contiki.h"

#include "dev/button-sensor.h"

#include <stdio.h>

/*---------------------------------------------------------------------------*/
PROCESS(example_iburst_process, "");
AUTOSTART_PROCESSES(&example_iburst_process);
/*---------------------------------------------------------------------------*/
static void
recv(struct iburst_conn *, const linkaddr_t * originatorc, const linkaddr_t * sender, uint8_t hops)
{
  printf("recv from %d.%d origin %d.%d - %d hops (seqno=%d) '%s'\n",
      sender->u8[0], sender->u8[1],
      originator->u8[0], originator->u8[1], hops,
      packetbuf_attr(PACKETBUF_ATTR_EPACKET_ID),
      (char *)packetbuf_dataptr());
}
static void
sent(struct iburst_conn *c)
{
  printf("sent\n");
}
static void
dropped(struct iburst_conn *c)
{
  printf("dropped\n");
}
static const struct iburst_callbacks callbacks = { recv, sent, dropped };
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_iburst_process, ev, data)
{
  static struct iburst_conn c;

  PROCESS_EXITHANDLER(iburst_close(&c));
  
  PROCESS_BEGIN();

  iburst_open(&c, 136, CLOCK_SECOND / 8, 2, &callbacks);

  SENSORS_ACTIVATE(button_sensor);

  while(1) {
    /* Wait for button click before sending the first message. */
    PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event && data == &button_sensor);

    packetbuf_copyfrom("BurstMe", 8);
    iburst_send(&c, CLOCK_SECOND / 8, 2);
  }
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
