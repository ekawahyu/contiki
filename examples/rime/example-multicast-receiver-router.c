
#include "net/rime/multicast.h"
#include "net/rime/multicast-linkaddr.h"
#include "contiki.h"

#include "dev/button-sensor.h"

#include <stdio.h>

/*---------------------------------------------------------------------------*/
PROCESS(example_multicast_process, "");
AUTOSTART_PROCESSES(&example_multicast_process);
/*---------------------------------------------------------------------------*/
static void
recv(struct multicast_conn *c, const linkaddr_t *from)
{
  printf("%d.%d: recv from %d.%d '%s'\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      from->u8[0], from->u8[1],
      (char *)packetbuf_dataptr());
}
static void
sent(struct multicast_conn *c)
{
  printf("%d.%d: sent\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
}
static const struct multicast_callbacks callbacks = { recv, sent};
/*---------------------------------------------------------------------------*/
static void
murecv(struct multicast_conn *c, const linkaddr_t *from)
{
  printf("%d.%d: recv from %d.%d '%s'\n",
      linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
      from->u8[0], from->u8[1],
      (char *)packetbuf_dataptr());
}
static void
musent(struct multicast_conn *c)
{
  printf("%d.%d: sent\n", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
}
static const struct multicast_callbacks mucallbacks = { murecv, musent};
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_multicast_process, ev, data)
{
  static struct multicast_conn c;
  static struct multicast_conn muc;

  PROCESS_EXITHANDLER(multicast_close(&c));
  
  PROCESS_BEGIN();

  multicast_linkaddr_init();
  multicast_open(&c, &multicast_node_addr, &callbacks);
  multicast_open(&c, &multicast_router_addr, &callbacks);
  multicast_open(&muc, &multicast_linklocal_addr, &mucallbacks);

  SENSORS_ACTIVATE(button_sensor);

  while(1) {
    /* Wait for button click before sending the first message. */
    PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event && data == &button_sensor);
  }
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
