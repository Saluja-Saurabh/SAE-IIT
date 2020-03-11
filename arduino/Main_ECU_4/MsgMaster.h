#ifndef MSGMSTR_H
#define MSGMSTR_H

#include "CANMSG.h"

class MsgMaster {
private:
    // IMPROVE: big arrays are kinda wasteful, no?
    uint8_t READNo = 0; // counters for how many messages we have for read & write
    uint8_t WRITENo = 0;
    uint8_t memoFlag[MAXVALIDDATA + 1];  // check if data is flag and what pos it is in | NOTE: pos is stored starting at 1
    int16_t *memoData[MAXVALIDDATA + 1]; // pointers to actual data values // add one to MAXVALIDDATA bc pins start at 0
    TTMsg *ReadTTMessages[MSGREADS];     // Msgs to be read
    TTMsg *WriteTTMessages[MSGWRITES];   // Msgs to be written
    int16_t *memoize(validData lookup, bool &isFlag);
    void insertMsg(TTMsg &msg, bool isReadMsg);
    void finalize(); // populate memo table and ensure messages don't conflict

public:
    bool newMsg(uint32 i, bool isReadMsg, uint32 off = 0);
    bool newMsg(uint32 i, msgHandle h, bool isReadMsg, uint32 off = 0);
    bool newMsg(uint32 i, const validData (&p)[4], bool isReadMsg, uint32 off = 0);
    bool newMsg(uint32 i, const validData (&p)[4], const flagReader (&fF)[8], const validData (&fV)[8], bool isReadMsg, uint32 off = 0);
    bool newMsg(TTMsg msg, bool isReadMsg, uint32 off = 0);
    bool newMsg(uint32 i, validData p[4], flagReader fF[8], validData fV[8], msgHandle h, bool isReadMsg, uint32 off = 0);
    int16_t getData(validData lookup);
    bool setData(validData lookup, int16_t value);
    void begin();
    void run();
};

extern MsgMaster Master;

#endif
