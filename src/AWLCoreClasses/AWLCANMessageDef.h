#ifndef AWLCANMESSAGEDEF_H
#define AWLCANMESSAGEDEF_H

// CAN Frame
typedef struct {
    unsigned long id;        // Message id
    unsigned long timestamp; // timestamp in milliseconds	
    unsigned char flags;     // [extended_id|1][RTR:1][reserver:6]
    unsigned char len;       // Frame size (0.8)
    unsigned char data[8]; // Databytes 0..7
} AWLCANMessage;



// id
#define AWLCANMSG_ID_COMMANDMESSAGE 80



#endif //AWLCANMESSAGEDEF_H
