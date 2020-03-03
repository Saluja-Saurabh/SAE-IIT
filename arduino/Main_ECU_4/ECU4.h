#ifndef ECU4_H
#define ECU4_H

#include "TTMsg.h"

// Led blinkery stuff
bool loopSwitch = false;
void toggleLED() {
    digitalWriteFast(boardLed, loopSwitch);
    loopSwitch = !loopSwitch;
}

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

#endif

// seperation of teensies
#ifdef TEENSYFRONT

#endif

#ifdef TEENSYBACK

#endif