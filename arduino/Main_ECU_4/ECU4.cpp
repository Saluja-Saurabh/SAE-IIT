#include "ECU4.h"

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

// seperation of teensies
#ifdef TEENSYFRONT

#endif

#ifdef TEENSYBACK

#endif