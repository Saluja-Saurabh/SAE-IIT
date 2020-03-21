#include "Funcs.h"

void toggleLED() {
    digitalWriteFast(13, loopSwitch);
    loopSwitch = !loopSwitch;
}

bool loopSwitch = false;

void LEDBlink() { // how does the teensy check for start button when it should be off when pressed???
    // Should Blink Twice per call
    toggleLED();
    delay(250);
    toggleLED();
    delay(125);
    toggleLED();
    delay(250);
    toggleLED();
    delay(125);
}

template <typename T = int>
T swapDbl(T *tblA[2], T *tblB[2]) { // swap places of length 2 arrays in mem
    T *tempTbl[2];
    memcpy(tempTbl, tblA, 8);
    memcpy(tblA, tblB, 8);
    memcpy(tblB, tempTbl, 8);
}

// IMPROVE: anyway to make this take advantage of the compiler?
int16_t decodeLilEdian(const byte low, const byte high) {
    int16_t value = 0;
    int16_t full_data = high * 255 + low;
    if (high < 128) { // positive
        value = full_data;
    } else if (high > 128) { //neg
        value = map(full_data, 65280, 32640, 0, -32640);
    }
    return value;
}