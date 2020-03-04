#include <IFCT.h>
int boardLed = 13;
// Led blinkery stuff
bool loopSwitch = false;

void toggleLED() {
    digitalWriteFast(boardLed, loopSwitch);
    loopSwitch = !loopSwitch;
}

void LEDBlink() {
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

CAN_message_t dataIn, msg; // Can data in obj

void setup() {
    pinMode(boardLed, OUTPUT);
    pinMode(25, OUTPUT);
    pinMode(26, OUTPUT);
    pinMode(2, INPUT);
    pinMode(14, INPUT);
    pinMode(15, INPUT);
    pinMode(16, INPUT);
    pinMode(18, INPUT);
    pinMode(20, INPUT);
    pinMode(21, INPUT);

    analogWrite(26, 512);
    analogWrite(25, 512);

    Serial.begin(9600);
    // delay(3000);
    LEDBlink();
    digitalWriteFast(boardLed, LOW);

    Can1.setBaudRate(500000); // Speeed
    Can1.enableFIFO();        // FirstInFirstOut
    msg.id = 42;
    msg.ext = 0;
    msg.len = 8;
    msg.buf[0] = 0;
    msg.buf[1] = 0;
    msg.buf[2] = 0;
    msg.buf[3] = 0;
    msg.buf[4] = 0;
    msg.buf[5] = 0;
    msg.buf[6] = 0;
    msg.buf[7] = 3;
}

void printMsg(CAN_message_t msg) {
    Serial.print("  LEN: ");
    Serial.print(msg.len);
    Serial.print(" EXT: ");
    Serial.print(msg.flags.extended);
    Serial.print(" REMOTE: ");
    Serial.print(msg.rtr);
    Serial.print(" TS: ");
    Serial.print(msg.timestamp);
    Serial.print(" ID: ");
    Serial.print(msg.id);
    Serial.print(" Buffer: ");
    for (uint8_t i = 0; i < msg.len; i++) {
        Serial.print(msg.buf[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

void loop() {
    if (Can1.read(dataIn)) {
        toggleLED();
        printMsg(dataIn);
        //     // Serial.print(dataIn.buf[7]);
        //     // Serial.print(",");
    }
    Can1.write(msg);

    // Serial.print(digitalRead(2) * 512); // startbutton
    // Serial.print(",");
    // Serial.print(analogRead(14));
    // Serial.print(",");
    // Serial.print(analogRead(15));
    // Serial.print(",");
    // Serial.print(analogRead(16));
    // Serial.print(",");
    // Serial.print(analogRead(18));
    // Serial.print(",");
    // Serial.print(analogRead(20));
    // Serial.print(",");
    // Serial.print(analogRead(21));
    // Serial.print(",");
    // Serial.print(1024);
    // Serial.print(",");
    // Serial.print(0);
    // Serial.println();
    // delay(8); // arduino ide is poopoo
}
