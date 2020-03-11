#ifndef FUNCS_H
#define FUNCS_H
#include <arduino.h>

template <typename T = int>
T swapDbl(T *tblA[2], T *tblB[2]) { // swap places of length 2 arrays in mem
    T *tempTbl[2];
    memcpy(tempTbl, tblA, 8);
    memcpy(tblA, tblB, 8);
    memcpy(tblB, tempTbl, 8);
}

#endif