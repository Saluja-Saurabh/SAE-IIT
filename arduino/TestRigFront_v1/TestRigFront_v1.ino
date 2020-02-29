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
    // pump not on digital pin
    Serial.begin(9600);
    delay(3000);
    LEDBlink();
    digitalWriteFast(boardLed, LOW);
}

void loop() {
    delay(8);                           // arduino ide is poopoo
    Serial.print(digitalRead(2) * 512); // startbutton
    Serial.print(",");
    Serial.print(analogRead(14));
    Serial.print(",");
    Serial.print(analogRead(15));
    Serial.print(",");
    Serial.print(analogRead(16));
    Serial.print(",");
    Serial.print(analogRead(18));
    Serial.print(",");
    Serial.print(analogRead(20));
    Serial.print(",");
    Serial.print(analogRead(21));
    Serial.print(",");
    Serial.print(1024);
    Serial.print(",");
    Serial.print(0);
    Serial.println();
}
