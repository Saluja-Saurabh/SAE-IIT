#include "ECU4.h"
#define TEENSYFRONT
#define TEENSYBACK

void ECU4::begin() {
    int time = millis();
    pinMode(boardLed, OUTPUT);
#ifdef TEENSYFRONT
    LEDBlink(2); // front will blink twice
    Serial.println("Initalizing ECU4 Teensy 3.6 FRONT CONFIG");
#endif
#ifdef TEENSYBACK
    LEDBlink(3); // back will blink thrice
    Serial.println("Initalizing ECU4 Teensy 3.6 BACK CONFIG");
#endif
    setMasterMessages();
    Master.begin();
    Serial.println("Initalization Finished in: " + millis() - time);
    LEDBlink();
    digitalWriteFast(boardLed, LOW);
}

void ECU4::run() {
    Master.run();
#ifdef TEENSYFRONT
    pushT2A(); // Teensy to andriod
#endif
}

/*
 *  ███████╗ ██╗   ██╗ ███╗   ██╗  ██████╗ ████████╗ ██╗  ██████╗  ███╗   ██╗ ███████╗
 *  ██╔════╝ ██║   ██║ ████╗  ██║ ██╔════╝ ╚══██╔══╝ ██║ ██╔═══██╗ ████╗  ██║ ██╔════╝
 *  █████╗   ██║   ██║ ██╔██╗ ██║ ██║         ██║    ██║ ██║   ██║ ██╔██╗ ██║ ███████╗
 *  ██╔══╝   ██║   ██║ ██║╚██╗██║ ██║         ██║    ██║ ██║   ██║ ██║╚██╗██║ ╚════██║
 *  ██║      ╚██████╔╝ ██║ ╚████║ ╚██████╗    ██║    ██║ ╚██████╔╝ ██║ ╚████║ ███████║
 *  ╚═╝       ╚═════╝  ╚═╝  ╚═══╝  ╚═════╝    ╚═╝    ╚═╝  ╚═════╝  ╚═╝  ╚═══╝ ╚══════╝                                                               
*/

#ifdef TEENSYFRONT
// TODO: Calculate average speed and do what with it?
// TODO: Active aero?

bool pruneFaults() {                                                       // TODO: Readded IMD and AMS faults?
    int final = Master.getData(MCFAULT2, 0) | Master.getData(MCFAULT2, 1); // bitwise OR faults to check both MC controllers
    final = final >> 3;                                                    // remove "reserved" bits
    final = final << 1;                                                    // for IMD fault
    final |= Master.getData(IMDReadLight);                                 // IMD Fault ?
    final = final << 1;                                                    // for BMS fault
    final |= Master.getData(AMSReadLight);                                 // AMS Fault ?
    T2AMsg[10] = final;
    return false;
}

void pushT2A() { // final push to tablet | arraysize: Teensy2SerialArrSize array: Teensy2SerialArr
    String s = "S ";
    T2AMsg[0] = Master.getData(BMSVolt);
    T2AMsg[1] = Master.getData(BMSTemp);
    T2AMsg[2] = 0; // avgSpeed go here
    T2AMsg[3] = Master.getData(MotorLTemp);
    T2AMsg[4] = Master.getData(MotorRTemp);
    T2AMsg[5] = (Master.getData(ControlBoard, 0) + Master.getData(ControlBoard, 1)) / 2; // avg of temps?
    T2AMsg[6] = 0;                                                                       // do both mc!
    T2AMsg[7] = 0;                                                                       // aero
    T2AMsg[8] = Master.getData(BMSSOC);
    T2AMsg[9] = Master.getData(BMSCurrent);
    pruneFaults();
    // T2AMsg[10] = buildFaultList(); //gets updated by fault handler
    for (int i = 0; i < 11; i++) {
        s += T2AMsg[i] + " ";
    }
    Serial.println(s);
}

// TODO: what are these suppose to do again?
void chargerSet() { // Run in loop
    bool chargerState = digitalReadFast(sig_shutdownState);
    digitalWriteFast(sig_charger, chargerState);
}

void setPump(int voltage = 0) { // Run in loop
    analogWriteDAC0(voltage);   // Test values on ocilli and check if it works
}

void brakeLights() { // Run in loop
    digitalWriteFast(sig_brakeLight, Master.getData(sig_brakePress) > 50);
}

// are fault messages all just flags?
// TODO: when a fault is detected send T2A and check serial

void accelCheck() { // read accel numbrs and sync with T2T line
    int a1 = analogRead(sig_accel1);
    int a2 = analogRead(sig_accel2);
    if (a1 < 5 || a2 < 5) { // To check and clean if the value is jumping around
        a1 = 0;
        a2 = 0;
        // Fault to tablet
    }
    float errorcheck = abs(a1 - a2) / a1; // Percent error
    if (errorcheck <= 0.1) {              // if the error is less than 10%
        Master.setData(avgAccel, (a1 + a2) / 2);
    } else {
        Serial.println("Error accelerator reading"); // ERROR?
    }
}
#endif

#ifdef TEENSYBACK
bool MCResetFunc(TTMsg &msg) { // MC Fault reseter thing
    msg.ext = 0;
    msg.len = 8;
    msg.buf[0] = 20;
    msg.buf[1] = 0;
    msg.buf[2] = 1;
    msg.buf[3] = 0;
    msg.buf[4] = 0;
    msg.buf[5] = 0;
    msg.buf[6] = 0;
    msg.buf[7] = 0;
    Messenger.writeMsg(msg);
    return true;
} // IMPROVE: There may be reliability issues with only sending one?

void motorWriteSpeed(TTMsg &msg, byte offset, bool direction, int speed) { // speed is value 0 - 860
    int percent_speed = constrain(map(speed, 0, 1024, 0, 400), 0, 400);    // seprate func for negative vals (regen)
    // Serial.println(percent_speed);
    //Calculations value = (high_byte x 256) + low_byte
    byte low_byte = percent_speed % 256;
    byte high_byte = percent_speed / 256;
    msg.id = SPEEDWRITE_ADD + offset - MOTOR_STATIC_OFFSET;
    // Serial.println(msg->id);
    msg.ext = 0;
    msg.len = 8;
    msg.buf[0] = low_byte; // NM
    msg.buf[1] = high_byte;
    msg.buf[2] = 0; // Speed
    msg.buf[3] = 0;
    msg.buf[4] = direction;           // Direction
    msg.buf[5] = START_BUTTON_PUSHED; // Inverter enable byte
    msg.buf[6] = 0;                   // Last two are the maximum torque values || if 0 then defualt values are set
    msg.buf[7] = 0;
    Messenger.writeMsg(msg);
}
#endif

/**
 *  ██╗  ██╗  █████╗  ███╗   ██╗ ██████╗  ██╗      ███████╗ ██████╗  ███████╗
 *  ██║  ██║ ██╔══██╗ ████╗  ██║ ██╔══██╗ ██║      ██╔════╝ ██╔══██╗ ██╔════╝
 *  ███████║ ███████║ ██╔██╗ ██║ ██║  ██║ ██║      █████╗   ██████╔╝ ███████╗
 *  ██╔══██║ ██╔══██║ ██║╚██╗██║ ██║  ██║ ██║      ██╔══╝   ██╔══██╗ ╚════██║
 *  ██║  ██║ ██║  ██║ ██║ ╚████║ ██████╔╝ ███████╗ ███████╗ ██║  ██║ ███████║
 *  ╚═╝  ╚═╝ ╚═╝  ╚═╝ ╚═╝  ╚═══╝ ╚═════╝  ╚══════╝ ╚══════╝ ╚═╝  ╚═╝ ╚══════╝
*/

#ifdef TEENSYFRONT
// flag handlers
void setPedalState(bool bit) {
    PEDAL_ERROR = bit;
}

void setCarMode(bool bit) {
    // TODO: ensure car is not moving
    if (Master.getData(angle, 0) < 5) { // rpm = motor controller rpm only for first MC
        CAR_MODE = bit;
    }
}

#endif
#ifdef TEENSYBACK
// flag handlers
void initalizeCar(bool bit) {
    START_BUTTON_PUSHED = true;
    Serial.println("MOTORS UNLOCKED");
}

// packet handlers
// TODO: test what the accelerators are outputting
bool motorPushSpeed(TTMsg &msg) {
    // "the value of the tquore needs to be a power of 10 of the actual tourqe;" by dominck
    // call ECEdata to accelerator value
    // TODO: torque vector function thing? probably goes here
    // also uses var: avgSpeed for the accelerator val
    int speed0 = analogRead(23); // speed of motor 0
    int speed1 = analogRead(23); // speed of motor 1
    // Serial.println(speed0);
    motorWriteSpeed(msg, 0, 0, speed0);
    motorWriteSpeed(msg, MOTOR_OFFSET, 1, speed1);
    return false; // Don't continue normal TTMsg proccessing
}

bool prechargeFunc(TTMsg &msg) {              // BROKEN: is this suppose to be a handler? should only be called in beginning? put in ecu class?
    if (digitalReadFast(sig_shutdownState)) { // if airs have no power, then precharge must be checked
        DO_PRECHARGE = 1;                     // its basiaclly a fault
        START_BUTTON_PUSHED = false;
    }
    // Todo was this meant to be just DCBusV?
    float MC_voltage = max(abs(Master.getData(DCBusV, 0)), abs(Master.getData(DCBusV, 1))) / 10; //Returns in power of 10s
    if (!digitalReadFast(sig_shutdownState) && DO_PRECHARGE) {                                   //if airs have no power but had before, then begin precharge circuit
        digitalWriteFast(sig_prechargeAir, LOW);                                                 //Keep air open
        digitalWriteFast(sig_precharge, HIGH);                                                   //precharge is closed

        if (Master.getData(BMSVolt) >= 150 && (Master.getData(BMSVolt) * 0.9) <= MC_voltage) { // BMS voltage is a global
            DO_PRECHARGE = 0;
            START_BUTTON_READY = true;
        }
    } else { // Can use any MCs voltage, will be the same, must be greater than 270V (0.9 * 300V)
        // Should return to normal state
        digitalWriteFast(sig_prechargeAir, HIGH); //close air
        digitalWriteFast(sig_precharge, LOW);     //precharge is off
    }
    return false;
}
#endif

/**
 *  ███╗   ███╗ ███████╗ ███████╗ ███████╗  █████╗   ██████╗  ███████╗ ███████╗
 *  ████╗ ████║ ██╔════╝ ██╔════╝ ██╔════╝ ██╔══██╗ ██╔════╝  ██╔════╝ ██╔════╝
 *  ██╔████╔██║ █████╗   ███████╗ ███████╗ ███████║ ██║  ███╗ █████╗   ███████╗
 *  ██║╚██╔╝██║ ██╔══╝   ╚════██║ ╚════██║ ██╔══██║ ██║   ██║ ██╔══╝   ╚════██║
 *  ██║ ╚═╝ ██║ ███████╗ ███████║ ███████║ ██║  ██║ ╚██████╔╝ ███████╗ ███████║
 *  ╚═╝     ╚═╝ ╚══════╝ ╚══════╝ ╚══════╝ ╚═╝  ╚═╝  ╚═════╝  ╚══════╝ ╚══════╝                                                                                                 
*/

void setMasterMessages() { // BROKEN: validDatas are not mapping to the right addresses!
#ifdef TEENSYFRONT
    //Both MCs with "mirror" messages
    Master.newMsg(FAULT_ADD - MOTOR_STATIC_OFFSET, {MCFAULT0, MCFAULT1, MCFAULT2, MCFAULT3}, MsgRead, MCOFFSET);
    // Others
    Master.newMsg(BMS_STATS_ADD, {BMSTemp, BMSVolt, BMSSOC, BMSCurrent}, MsgRead); // BROKEN: BMSCurrent not set on BMS?
    // I don't like this concurrency issues may arise and system may not allow it rn
    Master.newMsg(T2T_ADD, {avgAccel, sig_brakePress, steeringAng}, {/*NULL, NULL, NULL, initalizeCar, NULL*/}, {IMDReadLight, AMSReadLight, carMode, sig_startButton, pedalAir}, MsgWrite);
    Master.newMsg(T2T2_ADD, {rpmLWheel, rpmRWheel}, MsgWrite);
    Master.newMsg(T2T_ADD, {avgAccel, sig_brakePress, steeringAng}, {/*NULL, NULL, NULL, initalizeCar, NULL*/}, {IMDReadLight, AMSReadLight, carMode, sig_startButton, pedalAir}, MsgRead);
    Master.newMsg(T2T2_ADD, {rpmLWheel, rpmRWheel}, MsgRead);
#endif
#ifdef TEENSYBACK
    //Both MCs with "mirror" messages
    Master.newMsg(SPEEDWRITE_ADD - MOTOR_STATIC_OFFSET, motorPushSpeed, MsgWrite);
    Master.newMsg(RESETMC_ADD - MOTOR_STATIC_OFFSET, MCResetFunc, MsgWrite, MCOFFSET);
    Master.newMsg(TEMP2_ADD - MOTOR_STATIC_OFFSET, MCResetFunc, MsgRead, MCOFFSET);
    Master.newMsg(MOTORPOS_ADD - MOTOR_STATIC_OFFSET, {angle}, MsgRead, MCOFFSET);
    Master.newMsg(VOLTAGE_ADD - MOTOR_STATIC_OFFSET, prechargeFunc, MsgRead, MCOFFSET);
    // I don't like this concurrency issues may arise and system may not allow it rn
    Master.newMsg(T2T_ADD, {avgAccel, sig_brakePress, steeringAng}, {NULL, NULL, NULL, initalizeCar, NULL}, {IMDReadLight, AMSReadLight, carMode, sig_startButton, pedalAir}, MsgWrite);
    Master.newMsg(T2T2_ADD, {rpmLWheel, rpmRWheel}, MsgWrite);
    Master.newMsg(T2T_ADD, {avgAccel, sig_brakePress, steeringAng}, {NULL, NULL, NULL, initalizeCar, NULL}, {IMDReadLight, AMSReadLight, carMode, sig_startButton, pedalAir}, MsgRead);
    Master.newMsg(T2T2_ADD, {rpmLWheel, rpmRWheel}, MsgRead);
#endif
}

ECU4 ECU;