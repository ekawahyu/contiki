#ifndef CONECTRIC_MESSAGES_H_
#define CONECTRIC_MESSAGES_H_

#ifdef __cplusplus
extern "C" {
#endif

  // Conectric Network Message Types
enum {
  CONECTRIC_ATTR_NONE,          
  /* commissioning messages */  
  CONECTRIC_ROUTE_REQUEST,      
  CONECTRIC_ROUTE_REPLY,        
  CONECTRIC_ROUTE_REQUEST_BY_SN,
  // CONECTRIC_TIME_SYNC,
  /* reporting messages */
  CONECTRIC_SENSOR_BROADCAST_RHT,
  CONECTRIC_SENSOR_BROADCAST_OC,
  CONECTRIC_SENSOR_BROADCAST_SW,
  CONECTRIC_SUPERVISORY_REPORT,
  CONECTRIC_SENSOR_UPDATE,
  CONECTRIC_SET_STATE,
  CONECTRIC_POLL_RS485,
  CONECTRIC_POLL_RS485_REPLY,
  CONECTRIC_POLL_RS485_CHUNK,
  CONECTRIC_POLL_RS485_CHUNK_REPLY,
  CONECTRIC_POLL_WI,
  CONECTRIC_POLL_WI_REPLY,  
  
//  CONECTRIC_SET_LONG_MAC,
//  CONECTRIC_SET_LONG_MAC_REPLY,
//  CONECTRIC_GET_LONG_MAC,
//  CONECTRIC_GET_LONG_MAC_REPLY,
//  CONECTRIC_POLL_NEIGHBORS,
//  CONECTRIC_POLL_NEIGHBORS_REPLY,
//  CONECTRIC_MULTIHOP_PING,
//  CONECTRIC_MULTIHOP_PING_REPLY,
  CONECTRIC_ATTR_MAX
};

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