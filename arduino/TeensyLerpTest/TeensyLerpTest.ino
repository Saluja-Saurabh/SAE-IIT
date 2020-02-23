#include <AutoPID.h>
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

// Lerping vars
unsigned long lastUpdate;
double target = 0; //0 - 1024
double current = 0;

// FakeUser
long cool = 1;
long time = 1;

void newTrgt() {
    target = random(0, 1024);
}

bool checkTime() {
    if (cool) {
        if (millis() - cool > 1000) {
            cool = 0;
            // time = random(3000, 6000);
            time = random(200, 1000);
        }
        // Serial.print(-100);
    } else if (millis() % time <= 500) {
        cool = millis();
        // Serial.print(1024);
        return true;
    } else {
        // Serial.print(0);
    }
    return false;
}

double lerp(double t, double i, double f, double d) {
    // quad
    t = t / (d * 2);
    if (t < 1) {
        return f / 2 * t * t + i;
    }
    return -f / 2 * ((t - 1) * (t - 3) - 1) + i;

    //circular
    // t = t / (d * 2);
    // if (t < 1) {
    //     return -f / 2 * (sqrt(1 - t * t) - 1) + i;
    // }
    // t = t - 2;
    // return f / 2 * (sqrt(1 - t * t) + 1) + i;

    // quad rev
    // if (t < d / 2) {
    //     return -f / 2 * (pow(t * 2 / d - 1, 4) - 1) + i;
    // }
    // return (f / 2) * pow(((t * 2) - d) / d, 4) + (i + f / 2);
}

//input/output variables passed by reference, so they are updated automatically
AutoPID myPID(&current, &target, &current, 0, 1024, 0.25, 4, 10);

void setup() {
    pinMode(boardLed, OUTPUT);
    Serial.begin(9600);
    delay(3000);
    LEDBlink();
    digitalWriteFast(boardLed, LOW);

    myPID.setBangBang(900);
    myPID.setTimeStep(1);
}

void loop() {
    if (checkTime()) {
        newTrgt();
        toggleLED();
    }
    // myPID.run(); //call every loop, updates automatically at certain time interval
    // Serial.print(",");
    Serial.print(target);
    Serial.print(",");
    Serial.print(current);
    Serial.print(",");
    Serial.print(1024);
    Serial.print(",");
    Serial.print(-100);
    Serial.println();
    // delay(8);
    // current = lerp(10, current, (target - current), 15);
    current = lerp(10, current, (target - current) / 1024, 15);
    // if (Serial.available()) {
    //     int r = Serial.read();
    //     Serial.println(r, DEC);
    // }
}
