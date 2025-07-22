#ifndef COM_TYPES_H
#define COM_TYPES_H

#include <stdint.h>
#include <stdbool.h>


typedef enum {
    NONE = 0,
    CCP,
    CUSTOM, // Reserved for future use
} ProtocolType_t;

typedef struct
{
  uint8_t data[8];
} ccp_message_t;

typedef struct {
    ProtocolType_t Protocol;
    union {
        ccp_message_t ccp_msg;
    } msg;
} can_rx_message_t;



#endif // COMM_TYPES_H
