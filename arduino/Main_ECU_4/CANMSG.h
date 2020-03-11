#ifndef CANMSG_H
#define CANMSG_H

#include "TTMsg.h"

class CANMSG {
private:
    void recieveMsg(const CAN_message_t &msgIn);
    void writeMsg(const CAN_message_t &msgOut);
    TTMsg **ReadTTMessages;
    TTMsg **WriteTTMessages;

public:
    void setIO(TTMsg *messageReads[MSGREADS], TTMsg *messageWrites[MSGWRITES]);
    void begin(uint32_t baudRate = 500000);
    void run();
};
extern CANMSG Messenger;
#endif