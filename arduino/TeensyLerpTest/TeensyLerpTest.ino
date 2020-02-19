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

void setup() {
    pinMode(boardLed, OUTPUT);
    Serial.begin(9600);
    delay(3000);
    LEDBlink();
    digitalWriteFast(boardLed, LOW);
}

void loop() {
    toggleLED();
    Serial.println();
    if (Serial.available()) {
        int r = Serial.read();
        Serial.println(r, DEC);
    }
}
