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
    void writeMsg(const CAN_message_t *msgOut);
};

extern CANMSG Messenger;

// Debug Funcs
void printMsg(TTMsg const &msg) { // Print out can msg buffer w/ ID
    Serial.print(msg.id, HEX);
    Serial.println(" = ");
    Serial.println(msg.buf[0]);
    Serial.println(msg.buf[1]);
    Serial.println(msg.buf[2]);
    Serial.println(msg.buf[3]);
    Serial.println(msg.buf[4]);
    Serial.println(msg.buf[5]);
    Serial.println(msg.buf[6]);
    Serial.println(msg.buf[7]);
}
#endif