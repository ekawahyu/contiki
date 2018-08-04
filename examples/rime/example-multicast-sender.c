
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
  static struct etimer et;
  static struct multicast_conn c;
  static struct multicast_conn muc;
  static linkaddr_t to;

  PROCESS_EXITHANDLER(multicast_close(&c));
  
  PROCESS_BEGIN();

  multicast_linkaddr_init();

  multicast_open(&c, multicast_node_addr.network.u16, &callbacks);
  multicast_linkaddr_register(multicast_node_addr.network.u16, &multicast_node_addr.host);
  multicast_linkaddr_register(multicast_router_addr.network.u16, &multicast_router_addr.host);

  multicast_open(&muc, multicast_linklocal_addr.network.u16, &mucallbacks);
  multicast_linkaddr_register(multicast_linklocal_addr.network.u16, &linkaddr_node_addr);

  SENSORS_ACTIVATE(button_sensor);

  while(1) {
    /* Wait for button click before sending the first message. */
    PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event && data == &button_sensor);

    /* Sending to all nodes */
    packetbuf_copyfrom("Hello from Conectric to all of nodes, good morning all", 55);
    multicast_send(&c, &multicast_node_addr.host);

    etimer_set(&et, CLOCK_SECOND * 5);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    /* Sending to all routers */
    packetbuf_copyfrom("Hello from Conectric to all of routers, good morning all", 57);
    multicast_send(&c, &multicast_router_addr.host);

    etimer_set(&et, CLOCK_SECOND * 5);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    /* Sending to a single node unicast-like */
    to.u8[0] = 10;
    to.u8[1] = 0;
    packetbuf_copyfrom("Hello from Conectric to only you, good morning", 47);
    multicast_send(&muc, (const linkaddr_t *)&to);
  }
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
