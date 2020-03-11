#include "MsgMaster.h"

bool MsgMaster::newMsg(uint32 i, bool isReadMsg, uint32 off = 0) { // blank msg
    TTMsg msg;
    msg.id = i;
    msg.offset = off;
    insertMsg(msg, isReadMsg);
}
bool MsgMaster::newMsg(uint32 i, msgHandle h, bool isReadMsg, uint32 off = 0) { // purely handled by separate functions
    TTMsg msg;
    msg.id = i;
    msg.handle = h;
    msg.offset = off;
    insertMsg(msg, isReadMsg);
}
bool MsgMaster::newMsg(uint32 i, const validData (&p)[4], bool isReadMsg, uint32 off = 0) { // only stores data
    TTMsg msg;
    msg.id = i;
    msg.offset = off;
    insertMsg(msg, isReadMsg);
}
bool MsgMaster::newMsg(uint32 i, const validData (&p)[4], const flagReader (&fF)[8], const validData (&fV)[8], bool isReadMsg, uint32 off = 0) { // data storage for packets and reactive flags
    TTMsg msg;
    msg.id = i;
    msg.offset = off;
    insertMsg(msg, isReadMsg);
}
bool MsgMaster::newMsg(TTMsg msg, bool isReadMsg, uint32 off = 0) { // explicit duping
    TTMsg msg;
    msg.id = msg.id + msg.offset;
    msg.handle = msg.handle;
    insertMsg(msg, isReadMsg);
}
bool MsgMaster::newMsg(uint32 i, validData p[4], flagReader fF[8], validData fV[8], msgHandle h, bool isReadMsg, uint32 off = 0) { // for duplication purposes
    TTMsg msg;
    msg.id = i;
    memcpy(msg.packets, p, 16);     // copy values to TTMsg using memcpy
    memcpy(msg.flagFuncs, fF, 16);  // bc c++ can't assign after
    memcpy(msg.flagValues, fV, 16); // initalization or somthin lik dat
    msg.handle = h;
    msg.offset = off;
    insertMsg(msg, isReadMsg);
}

void MsgMaster::insertMsg(TTMsg &msg, bool isReadMsg) {
    if (isReadMsg && READNo < MSGMASTERREADS - 1) {
        ReadTTMessages[READNo++] = &msg;
    } else if (WRITENo < MSGMASTERWRITES - 1) {
        WriteTTMessages[WRITENo++] = &msg;
    }
}

int16_t MsgMaster::getData(validData lookup) {
    if (lookup < MAXVALIDDATA) {          // hard coded clamp of max validData
        uint8_t found = memoFlag[lookup]; // get pos of bit if it is a flag
        if (found) {                      // if pos is not 0 then it is a flag
            return bitRead(*memoData[lookup], found - 1);
        }
        // it is not a flag so must be word
        return *memoData[lookup];
    }
    return -1;
}

bool MsgMaster::setData(validData lookup, int16_t value) {
    if (lookup < MAXVALIDDATA) {                                                       // hard coded clamp of max validData
        uint8_t found = memoFlag[lookup];                                              // get pos of bit if it is a flag
        if (found) {                                                                   // if pos is not 0 then it is a flag
            *memoData[lookup] = bitWrite(*memoData[lookup], found - 1, value ? 1 : 0); // get value, set bit, then change value
            return true;
        }
        // it is not a flag so must be word
        *memoData[lookup] = value;
        return true;
    }
    return false;
}

bool MsgMaster::finalize() {
    TTMsg *msg;
    validData dataPoint;
    uint8_t i, j;
    for (i = 0; i < MSGMASTERREADS; i++) {
        msg = ReadTTMessages[i];
        for (j = 0; j < 8; j++) {
            dataPoint = msg->flagValues[i];
            if (dataPoint) {
                msg->containsFlag = true;
                memoFlag[dataPoint] = j + 1;         // store the bit position; also confirming this value is a flag | shift by one because 0 == nil
                memoData[dataPoint] = &msg->data[3]; // make sure this works!
            }
        }

        for (j = 0; j < 4; j++) {
            dataPoint = msg->packets[i];
            if (dataPoint) {
                memoData[dataPoint] = &msg->data[i]; // make sure this works! | Gets refrence of actual data entry on TTMsg
            }
        }

        if (msg->containsFlag && msg->packets[3]) { // if a flags exist and so do all four data slots then this is a problem
            Serial.print("WARNING: FLAG AND MESSAGE CONFLICT! ID: ");
            Serial.println(msg->id, HEX);
        } // TODO: find a way to display errors
    }

    for (i = 0; i < MSGMASTERWRITES; i++) {
        msg = WriteTTMessages[i];
        for (j = 0; j < 8; j++) {
            dataPoint = msg->flagValues[i];
            if (dataPoint) {
                msg->containsFlag = true;
                memoFlag[dataPoint] = j + 1;         // store the bit position; also confirming this value is a flag | shift by one because 0 == nil
                memoData[dataPoint] = &msg->data[3]; // make sure this works!
            }
        }

        for (j = 0; j < 4; j++) {
            dataPoint = msg->packets[i];
            if (dataPoint) {
                memoData[dataPoint] = &msg->data[i]; // make sure this works! | Gets refrence of actual data entry on TTMsg
            }
        }

        if (msg->containsFlag && msg->packets[3]) { // if a flags exist and so do all four data slots then this is a problem
            Serial.print("WARNING: FLAG AND MESSAGE CONFLICT! ID: ");
            Serial.println(msg->id, HEX);
        } // TODO: find a way to display errors
    }
}