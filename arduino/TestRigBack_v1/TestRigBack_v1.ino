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

void analogWriteVolt(uint8_t pin, double volt) {
    int val = constrain(map((long)volt * 10, 0, 33, 0, 4096), 0, 4096);
    analogWrite(pin, val);
}

void setup() {
    analogWriteResolution(12);
    pinMode(boardLed, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
    pinMode(27, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(0, OUTPUT);
    pinMode(23, OUTPUT);
    pinMode(22, OUTPUT);
    pinMode(21, OUTPUT);
    pinMode(20, OUTPUT);
    pinMode(25, INPUT);
    pinMode(19, INPUT);
    pinMode(18, INPUT);

    analogWriteVolt(5, 3.3);
    analogWriteVolt(6, 3.3);
    analogWriteVolt(7, 3.3);
    analogWriteVolt(8, 3.3);
    analogWriteVolt(27, 3.3);
    analogWriteVolt(9, 3.3);
    analogWriteVolt(10, 3.3);
    analogWriteVolt(0, 3.3);
    analogWriteVolt(23, 3.3);
    analogWriteVolt(22, 3.3);
    analogWriteVolt(21, 3.3);
    analogWriteVolt(20, 3.3);
    analogWriteVolt(A21, 3.3); // pump not on digital pin

    Serial.begin(9600);
    // delay(3000);
    LEDBlink();
    digitalWriteFast(boardLed, LOW);

    Can1.setBaudRate(500000); // Speeed
    Can1.enableFIFO();        // FirstInFirstOut
    msg.id = 22;
    msg.ext = 0;
    msg.len = 8;
    msg.buf[0] = 0;
    msg.buf[1] = 0;
    msg.buf[2] = 0;
    msg.buf[3] = 0;
    msg.buf[4] = 0;
    msg.buf[5] = 0;
    msg.buf[6] = 0;
    msg.buf[7] = 69;
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
        // Serial.print(dataIn.buf[7]);
        // Serial.print(",");
    }
    Can1.write(msg);

    // analogWrite(21, 500);
    // Serial.print(digitalRead(25));
    // Serial.print(",");
    // Serial.print(digitalRead(19) * 2);
    // Serial.print(",");
    // Serial.print(digitalRead(18) * 4);
    // Serial.println();
    // delay(8);
}
