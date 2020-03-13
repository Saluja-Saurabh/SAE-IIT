#ifndef FUNCS_H
#define FUNCS_H
#include <arduino.h>

// Led blinkery stuff
bool loopSwitch = false;
void toggleLED();
void LEDBlink();

template <typename T = int>
T swapDbl(T *tblA[2], T *tblB[2]);

// IMPROVE: anyway to make this take advantage of the compiler?
int16_t decodeLilEdian(const byte low, const byte high);

#endif