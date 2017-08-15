#ifndef CONECTRIC_MESSAGES_H_
#define CONECTRIC_MESSAGES_H_

#ifdef __cplusplus
extern "C" {
#endif

// Conectric Network Message Types
//    Commissioning  
#define CONECTRIC_ROUTE_REQUEST                 0x01
#define CONECTRIC_ROUTE_REQUEST_BY_SN           0x02
#define CONECTRIC_ROUTE_REPLY                   0x03
//    Reporting
#define CONECTRIC_SENSOR_BROADCAST_RHT          0x30
#define CONECTRIC_SENSOR_BROADCAST_SW           0x31
#define CONECTRIC_SENSOR_BROADCAST_OC           0x32
#define CONECTRIC_SUPERVISORY_REPORT            0x33
#define CONECTRIC_SENSOR_UPDATE                 0x34
#define CONECTRIC_SET_STATE                     0x35
#define CONECTRIC_POLL_RS485                    0x36
#define CONECTRIC_POLL_RS485_REPLY              0x37
#define CONECTRIC_POLL_RS485_CHUNK              0x38
#define CONECTRIC_POLL_RS485_CHUNK_REPLY        0x39
#define CONECTRIC_POLL_WI                       0x3A
#define CONECTRIC_POLL_WI_REPLY                 0x3B
    
// Generic Message Structure
typedef struct {
  clock_time_t  timestamp;
  uint8_t       message_type;
  uint8_t       message[128];
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

#ifdef __cplusplus
}
#endif

#endif /* CONECTRIC_MESSAGES.H */