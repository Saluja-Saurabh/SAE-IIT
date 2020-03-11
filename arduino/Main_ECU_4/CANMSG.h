#ifndef CANMSG_H
#define CANMSG_H

#include "TTMsg.h"

class CANMSG {
private:
    void flagScan(const byte &flagByte, flagReader funcTbl[8]);
    void readTTMsg(TTMsg *msg, const byte buf[8]);
    void recieveMsg(const CAN_message_t &msgIn);
    TTMsg **ReadTTMessages;
    TTMsg **WriteTTMessages;

public:
    void setIO(TTMsg *messageReads[MSGREADS], TTMsg *messageWrites[MSGWRITES]);
    void begin(uint32_t baudRate = 500000);
    void writeMsg(const CAN_message_t &msgOut);
};
extern CANMSG Messenger;
#endif