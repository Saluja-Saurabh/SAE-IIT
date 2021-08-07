#include "CANMSG.h"

// void CANMSG::setIO(TTMsg *messageReads[MSGREADS], TTMsg *messageWrites[MSGWRITES]) {
//     ReadTTMessages = messageReads;
//     WriteTTMessages = messageWrites;
// };

void CANMSG::flagScan(const byte &flagByte, flagReader funcTbl[8]) { // Only used by read messages
    if (flagByte) {                                                  // check if flag has any true bits
        for (byte bit = 0; bit < 8; ++bit) {                         // iterate though flag bits
            if (funcTbl[bit]) {                                      // check that we can do something if the bit is true
                funcTbl[bit]((flagByte >> bit) & 1);                 // call function based off bit pos
            }
        }
    }
}

void CANMSG::readTTMsg(TTMsg &msg, const byte buf[8]) {
    byte stop = 8;
    if (msg.containsFlag) {              // Readflags if they are expected
        flagScan(buf[7], msg.flagFuncs); // Only checking byte 0
        msg.buf[7] = buf[7];             // Store byte 0 which are flags
        msg.buf[6] = buf[6];             // also stores byte 1 for completion sake
        stop = 6;                        // Skip flag bytes
    }
    for (byte i = 0; i < stop; i += 2) {
        if (msg.packets[i]) {                                     // are we expecting data on this packet?
            msg.data[i / 2] = decodeLilEdian(buf[i], buf[i + 1]); // decode and store
            // TTMsg buf values not modified as only data array is needed for the read only TTMsgs
        }
    }
}

// Iterate through defined TTMsgs and check if the address is one of theirs
void CANMSG::recieveMsg(const CAN_message_t &msgIn) {
    for (uint8_t i = 0; i < MSGREADS; i++) { // IMPROVE: memoize this
        TTMsg *msg = Master.ReadTTMessages[i];
        if (msg->id == msgIn.id) {
            Messenger.readTTMsg(*msg, msgIn.buf); // id matches; interpret data based off matching msg structure
            break;
        };
    }
}

void CANMSG::writeMsg(const CAN_message_t &msgOut) {
    Can1.write(msgOut); // does this duplicate the msg?
};

void CANMSG::begin(uint32_t baudRate) {
    Can1.setBaudRate(baudRate);           // Speeed
    Can1.enableFIFO();                    // FirstInFirstOut
    Can1.onReceive(Messenger.recieveMsg); // may need to derefrence a pointer to a function
};

CANMSG Messenger;

// Debug

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