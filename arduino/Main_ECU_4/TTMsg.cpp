#include "TTMsg.h"

TTMsg::TTMsg(uint32 i, uint32 off = 0) { // blank msg
    id = i;
    offset = off;
}
TTMsg::TTMsg(uint32 i, msgHandle h, uint32 off = 0) { // purely handled by separate functions
    id = i;
    handle = h;
    offset = off;
}
TTMsg::TTMsg(uint32 i, const validData (&p)[4], uint32 off = 0) { // only stores data
    id = i;
    offset = off;
}
TTMsg::TTMsg(uint32 i, const validData (&p)[4], const flagReader (&fF)[8], const validData (&fV)[8], uint32 off = 0) { // data storage for packets and reactive flags
    id = i;
    offset = off;
}
TTMsg::TTMsg(TTMsg msg, uint32 off) { // explicit duping
    id = msg.id + msg.offset;
    handle = msg.handle;
}
TTMsg::TTMsg(uint32 i, validData p[4], flagReader fF[8], validData fV[8], msgHandle h, uint32 off = 0) { // for duplication purposes
    id = i;
    memcpy(packets, p, 16);     // copy values to TTMsg using memcpy
    memcpy(flagFuncs, fF, 16);  // bc c++ can't assign after
    memcpy(flagValues, fV, 16); // initalization or somthin lik dat
    handle = h;
    offset = off;
}

void TTMsg::finalize() {
}

int *TTMsg::memoize(validData lookup) { // lookup if data is remembered and return

    return nullptr;
}

int TTMsg::getData(validData lookup) {
    int *found = memoize(lookup);
    if (found != nullptr) { // if something is found proceed
        return *found;      // return actual value
    } else {
        for (uint8_t i = 0; i < 12; i++) { // check all values including flags
            if (lookTbl[i] == lookup) {
                if (i < 3) { // must be a full 4 byte value
                    return *memo[i];
                } else if (!containsFlag) { // again must be a full 4 byte value
                    return *memo[3];
                } else {                                       // must be a flag value
                    return (*memo[3] & (2 ^ (i - 3))) ? 1 : 0; // if value is not 0 return 1
                }
            }
        }
    }

    return -42069; // l0lz
}