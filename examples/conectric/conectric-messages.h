#ifndef CONECTRIC_MESSAGES_H_
#define CONECTRIC_MESSAGES_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Conectric Network Message Types */
/*   Commissioning */
#define CONECTRIC_ROUTE_REQUEST                 0x01
#define CONECTRIC_ROUTE_REQUEST_BY_SN           0x02
#define CONECTRIC_ROUTE_REPLY                   0x03
#define CONECTRIC_IMG_UPDATE_BCST               0x10
#define CONECTRIC_IMG_UPDATE_DIR                0x11
#define CONECTRIC_IMG_ACK                       0x12
#define CONECTRIC_IMG_COMPLETE                  0x13
#define CONECTRIC_MULTIHOP_PING                 0x14
#define CONECTRIC_MULTIHOP_PING_REPLY           0x15
#define CONECTRIC_REBOOT_REQUEST                0x16
#define CONECTRIC_REBOOT_REPLY                  0x17
#define CONECTRIC_SET_LONG_MAC                  0x18
#define CONECTRIC_SET_LONG_MAC_REPLY            0x19
#define CONECTRIC_GET_LONG_MAC                  0x1A
#define CONECTRIC_GET_LONG_MAC_REPLY            0x1B

/*   Reporting */
#define CONECTRIC_SENSOR_BROADCAST_RHT          0x30
#define CONECTRIC_SENSOR_BROADCAST_SW           0x31
#define CONECTRIC_SENSOR_BROADCAST_OC           0x32
#define CONECTRIC_SUPERVISORY_REPORT            0x33
#define CONECTRIC_SENSOR_UPDATE                 0x34
#define CONECTRIC_SET_STATE                     0x35
#define CONECTRIC_RS485_POLL                    0x36
#define CONECTRIC_RS485_POLL_REPLY              0x37
#define CONECTRIC_RS485_POLL_CHUNK              0x38
#define CONECTRIC_RS485_POLL_CHUNK_REPLY        0x39
#define CONECTRIC_POLL_WI                       0x3A
#define CONECTRIC_POLL_WI_REPLY                 0x3B
#define CONECTRIC_POLL_SENSORS                  0x3C
#define CONECTRIC_POLL_SENSORS_REPLY            0x3D
#define CONECTRIC_POLL_NEIGHBORS                0x3E
#define CONECTRIC_POLL_NEIGHBORS_REPLY          0x3F
#define CONECTRIC_SENSOR_BROADCAST_PLS          0x40
#define CONECTRIC_SENSOR_BROADCAST_USB          0x41

#define CONECTRIC_RS485_POLL_REPLY_IN_CHUNK     0x42

/*   Reporting boot status */
#define CONECTRIC_DEVICE_BROADCAST_BOOT_STATUS  0x60
#define CONECTRIC_TEXT_MESSAGE                  0x61

/* Configuring */
#define CONECTRIC_RS485_CONFIG                  0x70


/* Any message with MSB bit set to 1 needs immediate attention */

/*   Reporting low battery */
#define CONECTRIC_DEVICE_BROADCAST_LOW_BATTERY  0x80


/* Message buffer for incoming and outgoing sensor broadcasts */
#define CONECTRIC_MESSAGE_LENGTH                40


/* Conectric Message Structure */
typedef struct {
  clock_time_t  timestamp;
  uint8_t       message[128];
  uint8_t       message_type;
  uint8_t       message_len;
  linkaddr_t    sender;
  linkaddr_t    prev_sender;
  linkaddr_t    receiver;
  linkaddr_t    prev_receiver;
  linkaddr_t    esender;
  linkaddr_t    prev_esender;
  linkaddr_t    ereceiver;
  linkaddr_t    prev_ereceiver;
  uint8_t       *payload;
  uint8_t       length;
  uint8_t       request;
  uint8_t       seqno;
  uint8_t       hops;
  uint8_t       maxhops;
  uint16_t      rssi;
} message_recv;


/* Conectric Request Structure */
typedef struct {
  uint8_t reqlen;
  uint8_t req;
  linkaddr_t dest;
  uint8_t hdr_next;
  uint8_t * payload;
} request_line;


#ifdef __cplusplus
}
#endif

#endif /* CONECTRIC_MESSAGES.H */
