#include "TTMsg.h"

TTMsg::TTMsg(uint32 i, uint32 off = 0) { // blank msg
    id = i;
    packets = {0};
    flagFuncs = {0};
    flagValues = {0};
    offset = off;
}
TTMsg::TTMsg(uint32 i, msgHandle h, uint32 off = 0) { // purely handled by separate functions
    id = i;
    packets = {0};
    flagFuncs = {0};
    flagValues = {0};
    handle = h;
    offset = off;
}
TTMsg::TTMsg(uint32 i, const validData (&p)[4], uint32 off = 0) { // only stores data
    id = i;
    packets = {0};
    flagFuncs = {0};
    flagValues = {0};
    offset = off;
}
TTMsg::TTMsg(uint32 i, const validData (&p)[4], const flagReader (&fF)[8], const validData (&fV)[8], uint32 off = 0) { // data storage for packets and reactive flags
    id = i;
    packets = {0};
    flagFuncs = {0};
    flagValues = {0};
    offset = off;
}
TTMsg::TTMsg(TTMsg msg, uint32 off) { // explicit duping
    id = msg.id + msg.offset;
    packets = {0};
    flagFuncs = {0};
    flagValues = {0};
    handle = msg.handle;
}
TTMsg::TTMsg(uint32 i, validData p[4], flagReader fF[8], validData fV[8], msgHandle h, uint32 off = 0) { // for duplication purposes
    id = i;
    packets = p;
    flagFuncs = fF;
    flagValues = fV;
    handle = h;
    offset = off;
}