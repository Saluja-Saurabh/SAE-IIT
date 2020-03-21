#ifndef FUNCS_H
#define FUNCS_H
#include <arduino.h>

// Led blinkery stuff
extern bool loopSwitch;
void toggleLED();
void LEDBlink(int times = 2);

template <typename T = int>
T swapDbl(T *tblA[2], T *tblB[2]);

// IMPROVE: anyway to make this take advantage of the compiler?
int16_t decodeLilEdian(const byte low, const byte high);

#endif