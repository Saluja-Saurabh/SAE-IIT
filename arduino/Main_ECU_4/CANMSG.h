#ifndef CANMSG_H
#define CANMSG_H

#include "ECU4.h"
#include "MsgMaster.h"

class CANMSG {
private:
    void flagScan(const byte &flagByte, flagReader funcTbl[8]);
    void readTTMsg(TTMsg &msg, const byte buf[8]);
    static void recieveMsg(const CAN_message_t &msgIn);

public:
    // void setIO(TTMsg *messageReads[MSGREADS], TTMsg *messageWrites[MSGWRITES]); //IMPROVE: only pass id and buf of messages
    void begin(uint32_t baudRate = 500000);
    void writeMsg(const CAN_message_t &msgOut);
};

// Debug Funcs
void printMsg(TTMsg const &msg);

extern CANMSG Messenger;

#endif