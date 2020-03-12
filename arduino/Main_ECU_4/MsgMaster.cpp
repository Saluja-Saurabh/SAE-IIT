#include "MsgMaster.h"

bool MsgMaster::newMsg(uint32_t i, bool isReadMsg, uint32_t off = 0) { // blank msg
    TTMsg msg;
    msg.id = i;
    if (off) {
        offsetMsg(msg, off, isReadMsg);
    }
    insertMsg(msg, isReadMsg);
}

bool MsgMaster::newMsg(uint32_t i, msgHandle h, bool isReadMsg, uint32_t off = 0) { // purely handled by separate functions
    TTMsg msg;
    msg.id = i;
    msg.handle = h;
    if (off) {
        offsetMsg(msg, off, isReadMsg);
    }
    insertMsg(msg, isReadMsg);
}

bool MsgMaster::newMsg(uint32_t i, const validData (&p)[4], bool isReadMsg, uint32_t off = 0) { // only stores data
    TTMsg msg;
    msg.id = i;
    if (off) {
        offsetMsg(msg, off, isReadMsg);
    }
    insertMsg(msg, isReadMsg);
}

bool MsgMaster::newMsg(uint32_t i, const validData (&p)[4], const flagReader (&fF)[8], const validData (&fV)[8], bool isReadMsg, uint32_t off = 0) { // data storage for packets and reactive flags
    TTMsg msg;
    msg.id = i;
    if (off) {
        offsetMsg(msg, off, isReadMsg);
    }
    insertMsg(msg, isReadMsg);
}

void MsgMaster::offsetMsg(TTMsg &msg, uint32_t off, bool isReadMsg) { // duplicates message blocks; allows the offset block to have seperate read/write data
    // Master.newMsg(msg->id + msg->offset, msg->packets, msg->flagFuncs, msg->flagValues, msg->handle, isReadMsg);
    TTMsg msgDup;
    msgDup.id = msg.id + off;
    memcpy(msgDup.packets, msg.packets, 16);       // copy values to TTMsg using memcpy
    memcpy(msgDup.flagFuncs, msg.flagFuncs, 16);   // bc c++ can't assign after
    memcpy(msgDup.flagValues, msg.flagValues, 16); // initalization or somthin lik dat
    msgDup.handle = msg.handle;
    msgDup.isOffset = true; // show that this msg is a "mirror" of another
    insertMsg(msgDup, isReadMsg);
}

void MsgMaster::begin() {
    finalize();
    Messenger.setIO(ReadTTMessages, WriteTTMessages);
    Messenger.begin();
}

int16_t MsgMaster::getDataLookup(uint32_t address, uint8_t dataPacket, bool isOffset = 0) {
    
}

void flagRead(TTMsg *msg) { // read pins that map to flag variables
    // TODO: only last flag byte (the important one) is getting checked
    // for (byte i = 7; i <= 6; i--) {                            // capped to last two bytes
    int i = 7;                                                    // eventually replace with for loop
    msg->buf[i] = 0;                                              // clear flags
    for (byte bit = 0; bit < 8; ++bit) {                          // iterate through byte bits
        if (msg->flagValues[bit]) {                               // check if there is a flag defined
            msg->buf[i] |= digitalReadFast(msg->flagValues[bit]); // store flag
        }                                                         //
        msg->buf[i] = msg->buf[i] << 1;                           // shift bits
    }
    // }
}

void updateData(TTMsg *msg) {
    if (msg->handle && !(msg->handle)(msg)) { // if the handle exists and returns true upon calling continue execution
        return;
    }
    byte stop = 8;
    if (msg->containsFlag) { // Readflags if they are expected
        flagRead(msg);       // Check and store flag bytes
        stop = 6;            // Skip flag bytes
    }
    for (byte i = 0; i < stop; i += 2) {
        if (msg->packets[i / 2]) {                     // If we have a sensor for this packet read and store it
            int val = analogRead(msg->packets[i / 2]); // TODO: Some sensors are digital not just analog!
            msg->data[i / 2] = val;                    // store the raw value
            msg->buf[i] = val % 256;
            msg->buf[i + 1] = val / 256;
        }
    }
    Messenger.writeMsg(msg);
}

void MsgMaster::run() {
    for (TTMsg *msg : WriteTTMessages) { // Iterate through defined TTMsgs and push their data
        updateData(msg);
    }
}

void MsgMaster::insertMsg(TTMsg &msg, bool isReadMsg) {
    if (isReadMsg && READNo < MSGREADS - 1) {
        ReadTTMessages[READNo++] = &msg;
    } else if (WRITENo < MSGWRITES - 1) {
        WriteTTMessages[WRITENo++] = &msg;
    }
}

int16_t MsgMaster::getData(validData lookup, bool isOffset = 0) {
    if (lookup < MAXVALIDDATA) {          // hard coded clamp of max validData
        uint8_t found = memoFlag[lookup]; // get pos of bit if it is a flag
        if (found) {                      // if pos is not 0 then it is a flag
            return bitRead(isOffset ? *memoDataOff[lookup] : *memoData[lookup], found - 1);
        }
        // it is not a flag so must be word
        return isOffset ? *memoDataOff[lookup] : *memoData[lookup];
    }
    return -1;
}

bool MsgMaster::setData(validData lookup, int16_t value, bool isOffset = 0) {
    if (lookup < MAXVALIDDATA) {                                                              // hard coded clamp of max validData
        uint8_t found = memoFlag[lookup];                                                     // get pos of bit if it is a flag
        if (found) {                                                                          // if pos is not 0 then it is a flag
            if (isOffset) {                                                                   // if requesting offset version look up offsetData
                *memoDataOff[lookup] = bitWrite(*memoData[lookup], found - 1, value ? 1 : 0); // get value, set bit, then change value
                return true;
            }
            *memoData[lookup] = bitWrite(*memoData[lookup], found - 1, value ? 1 : 0); // get value, set bit, then change value
            return true;
        }
        // it is not a flag so must be word
        if (isOffset) {
            *memoDataOff[lookup] = value;
            return true;
        }

        *memoData[lookup] = value;
        return true;
    }
    return false;
}

void MsgMaster::finalize() {
    validData dataPoint;
    uint8_t i, j;
    for (TTMsg *msg : ReadTTMessages) {
        for (j = 0; j < 8; j++) {
            dataPoint = msg->flagValues[i];
            if (dataPoint) {
                msg->containsFlag = true;
                memoFlag[dataPoint] = j + 1;                                                  // store the bit position; also confirming this value is a flag | shift by one because 0 == nil
                msg->isOffset ? memoDataOff[dataPoint] : memoData[dataPoint] = &msg->data[3]; // make sure this works!
            }
        }

        for (j = 0; j < 4; j++) {
            dataPoint = msg->packets[i];
            if (dataPoint) {
                msg->isOffset ? memoDataOff[dataPoint] : memoData[dataPoint] = &msg->data[i]; // make sure this works! | Gets refrence of actual data entry on TTMsg
            }
        }

        if (msg->containsFlag && msg->packets[3]) { // if a flags exist and so do all four data slots then this is a problem
            Serial.print("WARNING: FLAG AND MESSAGE CONFLICT! ID: ");
            Serial.println(msg->id, HEX);
        } // TODO: find a way to display errors
    }

    for (TTMsg *msg : WriteTTMessages) {
        for (j = 0; j < 8; j++) {
            dataPoint = msg->flagValues[i];
            if (dataPoint) {
                msg->containsFlag = true;
                memoFlag[dataPoint] = j + 1;                                                  // store the bit position; also confirming this value is a flag | shift by one because 0 == nil
                msg->isOffset ? memoDataOff[dataPoint] : memoData[dataPoint] = &msg->data[3]; // make sure this works!
            }
        }

        for (j = 0; j < 4; j++) {
            dataPoint = msg->packets[i];
            if (dataPoint) {
                msg->isOffset ? memoDataOff[dataPoint] : memoData[dataPoint] = &msg->data[i]; // make sure this works! | Gets refrence of actual data entry on TTMsg
            }
        }

        if (msg->containsFlag && msg->packets[3]) { // if a flags exist and so do all four data slots then this is a problem
            Serial.print("WARNING: FLAG AND MESSAGE CONFLICT! ID: ");
            Serial.println(msg->id, HEX);
        } // TODO: find a way to display errors
    }

    // TODO: offsets of messages
    // for (auto msg : WriteTTMessages) {
    //     initalizeMsg(msg);
    //     if (msg.offset) {
    //         initalizeMsg(offsetMsg(msg));
    //     }
    // }

    // for (auto msg : ReadTTMessages) {
    //     initalizeMsg(msg);
    //     if (msg.offset) {
    //         initalizeMsg(offsetMsg(msg));
    //     }
    // }
}