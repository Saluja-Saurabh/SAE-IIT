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

int target = 0; //0 - 1024
uint64_t time = 0;
int current = 0;

void newTrgt() {
    target = random(0, 1024);
}

bool checkTime() {
    int x = millis() % random(10000, 12000);
    Serial.println(x);
    return x <= 0;
}

void loop() {
    if (checkTime())
        toggleLED();
    // if (Serial.available()) {
    //     int r = Serial.read();
    //     Serial.println(r, DEC);
    // }
}
