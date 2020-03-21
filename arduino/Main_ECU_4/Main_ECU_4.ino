/*
    SAE Teensyduino
    2019 - 2020
    Teensy 3.6 ECU x2
    Version 4 Rev 2
*/

// #define TEENSYFRONT // hmmmmmm
#define TEENSYBACK
#include "ECU4.h"
#include "MsgMaster.h"
#include "TTMsg.h"

void setup() {
    delay(3000);
    Serial.begin(19200);
    ECU.begin();
}

void loop() {
    ECU.run();
}