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

    analogWrite(5, 1024);
    analogWrite(6, 1024);
    analogWrite(7, 1024);
    analogWrite(8, 1024);
    analogWrite(27, 1024);
    analogWrite(9, 1024);
    analogWrite(10, 1024);
    analogWrite(0, 1024);
    analogWrite(23, 1024);
    analogWrite(22, 1024);
    analogWrite(21, 1024);
    analogWrite(20, 1024);
    // pump not on digital pin

    Serial.begin(9600);
    delay(3000);
    LEDBlink();
    digitalWriteFast(boardLed, LOW);
}

void loop() {
    analogWrite(21, 500);
    Serial.print(digitalRead(25));
    Serial.print(",");
    Serial.print(digitalRead(19) * 2);
    Serial.print(",");
    Serial.print(digitalRead(18) * 4);
    Serial.println();
    delay(8);
}
