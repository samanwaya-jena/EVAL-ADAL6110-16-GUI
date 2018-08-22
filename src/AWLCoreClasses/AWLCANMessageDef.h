#ifndef AWLCANMESSAGEDEF_H
#define AWLCANMESSAGEDEF_H

// CAN Frame
typedef struct {
    uint32_t id;        // Message id
    uint32_t timestamp; // timestamp in milliseconds
    uint8_t  flags;     // [extended_id|1][RTR:1][reserver:6]
    uint8_t  len;       // Frame size (0.8)
    uint8_t  data[8];   // Databytes 0..7
    uint8_t  pad1;      // Padding required so that sizeof(AWLCANMessage) be 20 on all platforms
    uint8_t  pad2;      // ...
} AWLCANMessage;


// len
#define AWLCANMSG_LEN 8

// id
#define AWLCANMSG_ID_COMMANDMESSAGE 80

// New command id for Wagner
#define AWLCANMSG_ID_GETDATA        87
#define AWLCANMSG_ID_POLLMESSAGES   88
#define AWLCANMSG_ID_LIDARQUERY     89

/*
00: Command (0xC0 = SET_PARAMETER)
0xC1 = QUERY_PARAMETER)
0xC2 = RESPONSE_PARAMETER)
01: Type (0x01 = ALGO_SELECTED
0x02 = ALGO_PARAMETER
0x03 = AWL_REGISTER
0x04 = BIAS
0x05 = ADC_REGISTER
0x06 = PRESET
0x07 = GLOBAL_PARAMETER
0x08 = GPIO_CONTROL
0x11 = TRACKER_SELECTED
0x12 = TRACKER_PARAMETER
0x20 = DATE_TIME
0xD0 = RECORD_FILENAME (zero-terminated)
0xD1 = PLAYBACK_FILENAME (zero-terminated)
02-03: Address (U16_LE)
04-07: Value (x32_LE or U8S)

for DATE:
04-05: YEAR (U16_LE)
06: MONTH
07: DAY-OF-MONTH

for TIME:
04: HOURS
05: MINUTES
06: SECONDS
07: 0x00
*/


#endif //AWLCANMESSAGEDEF_H
