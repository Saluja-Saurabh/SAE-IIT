/*
    SAE - 2019
    Teensy 3.6 ECU x2
    Version 4
*/

#include "ECU4.h"
#include "MsgMaster.h"
#include "TTMsg.h"

/* Buzzer 
 * pre charge func
 * 
 * START:
 * StartButtonPushed false
 * Push button
 * Switch on the buzzer for 3 secs
 * StartButtonPushed true
 * 
 * if fault goto START
 * */

// faults to check

/* 
    ----- SRT ECU specific data -----  
                                        */

// Globals!?
uint32_t MOTOR_OFFSET = 0xe0;         // offset for motor ids // is this actually just for the MCs?
uint32_t MOTOR_STATIC_OFFSET = 0x0A0; // IMPROVE: auto set this global offset to addresses
bool DO_PRECHARGE = true;             // Precharge latching variable
bool START_BUTTON_READY = false;      // when to buzz
bool START_BUTTON_PUSHED = false;     // MC enable bit state
bool PEDAL_ERROR = false;
bool CAR_MODE = false; // true: RaceMode false: EcoMode
bool FAULT = false;    // ams, bms mc faults and DOPRECHARGE == 1 from precharge circuit
// CAN_message_t dataIn;  // 1 data in obj
const byte Teensy2SerialArrSize = 12;
int Teensy2SerialArr[Teensy2SerialArrSize];
int T2AMsg[11];

// void checkFaults() {
// }

// // Flag Handles
void initalizeCar(bool bit) {
    START_BUTTON_PUSHED = true;
    Serial.println("MOTORS UNLOCKED");
}

// // handles
// void setPedalState(bool bit) {
//     PEDAL_ERROR = bit;
// }

bool MCResetFunc(TTMsg *msg) { // MC Fault reseter thing
    msg->ext = 0;
    msg->len = 8;
    msg->buf[0] = 20;
    msg->buf[1] = 0;
    msg->buf[2] = 1;
    msg->buf[3] = 0;
    msg->buf[4] = 0;
    msg->buf[5] = 0;
    msg->buf[6] = 0;
    msg->buf[7] = 0;
    Messenger.writeMsg(msg);
    return true;
} // IMPROVE: There may be reliability issues with only sending one?

// void setCarMode(bool bit) {
//     // TODO: ensure car is not moving
//     // if (*ECUData.MCMotorAng0 < 5) {
//     if (Master.getData(); < 5) { // rpm = motor controller rpm only for first MC
//         CAR_MODE = bit;
//     }
// }

//Both MCs
// struct TTMsg WriteSpeed = TTMsg(SPEEDWRITE_ADD, motorPushSpeed);
// struct TTMsg MCReset0 = TTMsg(RESETMC_ADD - MOTOR_STATIC_OFFSET, MCResetFunc);
// struct TTMsg MCReset1 = TTMsg(MCReset0, MCOFFSET);
// struct TTMsg MCTempRead0 = TTMsg(RESETMC_ADD - MOTOR_STATIC_OFFSET, MCResetFunc);
// struct TTMsg MCTempRead1 = TTMsg(MCTempRead0, MCOFFSET);
// struct TTMsg MCMotorPos0 = TTMsg(MOTORPOS_ADD - MOTOR_STATIC_OFFSET, {angle});
// struct TTMsg MCMotorPos1 = TTMsg(MCMotorPos0, MCOFFSET);
// struct TTMsg MCFaults0 = TTMsg(FAULT_ADD);
// struct TTMsg MCFaults1 = TTMsg(MCFaults0, MCOFFSET);
// struct TTMsg MCVolt0 = TTMsg(VOLTAGE_ADD - MOTOR_STATIC_OFFSET, prechargeFunc);
// struct TTMsg MCVolt1 = TTMsg(MCVolt0, MCOFFSET);
// // Others
// struct TTMsg bmsStat = TTMsg(BMS_STATS_ADD, {BMSTemp, BMSVolt, BMSSOC});
// struct TTMsg motorL = TTMsg(MOTORL_ADD, {MotorLTemp});
// struct TTMsg motorR = TTMsg(MOTORR_ADD, {MotorRTemp});
// struct TTMsg T2TData = TTMsg(T2T_ADD, {avgAccel, sig_brakePress, steeringAng}, {NULL, NULL, NULL, initalizeCar, NULL}, {IMDReadLight, AMSReadLight, carMode, sig_startButton, pedalAir});
// struct TTMsg T2T2Data = TTMsg(T2T2_ADD, {rpmLWheel, rpmRWheel});

bool prechargeFunc(TTMsg *msg) {              // BROKEN: is this suppose to be a handler?
    if (digitalReadFast(sig_shutdownState)) { // if airs have no power, then precharge must be checked
        DO_PRECHARGE = 1;                     // its basiaclly a fault
        START_BUTTON_PUSHED = false;
    }
    // MC average?
    float MC_voltage = max(abs(decodeLilEdian(*ECUData.MCVOLT_P01, *ECUData.MCVOLT_P02)), abs(decodeLilEdian(*ECUData.MCVOLT_P11, *ECUData.MCVOLT_P12))) / 10; //Returns in power of 10s
    if (!digitalReadFast(sig_shutdownState) && DO_PRECHARGE) {                                                                                                 //if airs have no power but had before, then begin precharge circuit
        digitalWriteFast(sig_prechargeAir, LOW);                                                                                                               //Keep air open
        digitalWriteFast(sig_precharge, HIGH);                                                                                                                 //precharge is closed

        if (*ECUData.BMSVolt_p >= 150 && (*ECUData.BMSVolt_p * 0.9) <= MC_voltage) { // BMS voltage is a global
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

// Initalize messages
void setMasterMessages() { // BROKEN: validDatas are not mapping to the right addresses!
    //Both MCs with "mirror" messages
    Master.newMsg(SPEEDWRITE_ADD, motorPushSpeed, MsgWrite);
    Master.newMsg(RESETMC_ADD - MOTOR_STATIC_OFFSET, MCResetFunc, MsgWrite, MCOFFSET);
    Master.newMsg(TEMP2_ADD - MOTOR_STATIC_OFFSET, MCResetFunc, MsgRead, MCOFFSET);
    Master.newMsg(MOTORPOS_ADD - MOTOR_STATIC_OFFSET, {angle}, MsgRead, MCOFFSET);
    Master.newMsg(FAULT_ADD, MsgRead, MCOFFSET);
    Master.newMsg(VOLTAGE_ADD - MOTOR_STATIC_OFFSET, prechargeFunc, MsgRead, MCOFFSET);
    // Others
    Master.newMsg(BMS_STATS_ADD, {BMSTemp, BMSVolt, BMSSOC}, MsgRead);
    Master.newMsg(MOTORL_ADD, {MotorLTemp}, MsgRead);
    Master.newMsg(MOTORR_ADD, {MotorRTemp}, MsgRead);
    // I don't like this concurrency issues may arise and system does not allow it rn
    Master.newMsg(T2T_ADD, {avgAccel, sig_brakePress, steeringAng}, {NULL, NULL, NULL, initalizeCar, NULL}, {IMDReadLight, AMSReadLight, carMode, sig_startButton, pedalAir}, MsgWrite);
    Master.newMsg(T2T2_ADD, {rpmLWheel, rpmRWheel}, MsgWrite);
}

// initECUPointers // after all is declared set the appropriate pointers
// anything that says MC, motor controller, needs to be doubled
// pointers to data that will be used within teensy itself | ex. BMS_VOLTAGE
// ECUData.BMSVolt_p = &bmsStat.data[1]; // we must keep track of what is being stored where in the packet setup
// ECUData.BMSTEMP_p = &bmsStat.data[0];
// ECUData.LMTEMP_P = &motorL.data[0];
// ECUData.RMTEMP_P = &motorR.data[0];
// ECUData.BMSSOC_P = &bmsStat.data[2];        // state of charge
// ECUData.BMSBUSCURRENT_P = &bmsStat.data[2]; // bus current
// ECUData.T2TACCEL_P = &T2TData.data[0];      // avgAccel
// ECUData.T2TBRAKE_P = &T2T2Data.data[1];     // break pressure
// // MCs //IMPROVE: maybe put both MCs in individual sub tables?
// ECUData.MCMotorAng0 = &MCMotorPos0.data[0];
// ECUData.MCMotorAng1 = &MCMotorPos1.data[0];
// ECUData.MCTEMP_P0 = &MCTempRead0.data[0];
// ECUData.MCTEMP_P1 = &MCTempRead1.data[0];
// ECUData.MCVOLT_P01 = &MCVolt0.data[0];
// ECUData.MCVOLT_P02 = &MCVolt0.data[1];
// ECUData.MCVOLT_P11 = &MCVolt1.data[0];
// ECUData.MCVOLT_P12 = &MCVolt1.data[1];
// ECUData.MCFAULT_P00 = &MCFaults0.buf[4];
// ECUData.MCFAULT_P01 = &MCFaults0.buf[5];
// ECUData.MCFAULT_P10 = &MCFaults1.buf[4];
// ECUData.MCFAULT_P11 = &MCFaults1.buf[5];
// // Faults
// ECUData.T2TFlags = &T2TData.buf[7];

// TODO: calculate average speed
// TODO: Active aero

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
    for (int i; i < 11; i++) {
        s += T2AMsg[i] + " ";
    }
    Serial.println(s);
}

//TODO: redo IMD and AMS faults
bool pruneFaults() { // figure which bits go where
    int final = 0;
    final = Master.getDataLookup(FAULT_ADD, 4, 0) | Master.getDataLookup(FAULT_ADD, 4, 1); // or faults to check both
    final = final << 8;
    final |= Master.getDataLookup(FAULT_ADD, 5, 0) | Master.getDataLookup(FAULT_ADD, 5, 1);
    T2AMsg[10] = final;
    return false;
}

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

// load messages as read or write
// TTMsg ReadTTMessages[]{
//     MCVolt0,
//     MCVolt1,
//     bmsStat,
//     T2TData,
// };

// TTMsg WriteTTMessages[]{
//     WriteSpeed,
//     MCReset0,
//     MCReset1,
//     T2TData,
// };

/* 
    ----- END ECU specific data -----  
                                        */

// are fault messages all just flags?
// TODO: when a fault is detected read serial

void setup() {
    delay(3000);
    Serial.begin(19200);
    pinMode(boardLed, OUTPUT);
    Serial.println("Hello!");
    LEDBlink();
    pinMode(2, OUTPUT); // Fusion Tech's Dual CAN-Bus R pin switch
    digitalWriteFast(2, LOW);
    LEDBlink();
    digitalWriteFast(boardLed, LOW);
}

void loop() {
    // if (Can1.read(dataIn)) {
    //     teensyRead(dataIn);
    // }
    // for (TTMsg msg : WriteTTMessages) { // Iterate through defined TTMsgs and push their data
    //     updateData(msg);
    // }
    // pushT2A(); // Teensy to andriod
}

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

// motor functions

// TODO: test what the accelerators are outputting
bool motorPushSpeed(TTMsg *msg) {
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

void motorWriteSpeed(TTMsg *msg, byte offset, bool direction, int speed) { // speed is value 0 - 860
    int percent_speed = constrain(map(speed, 0, 1024, 0, 400), 0, 400);    // seprate func for negative vals (regen)
    // Serial.println(percent_speed);
    //Calculations value = (high_byte x 256) + low_byte
    byte low_byte = percent_speed % 256;
    byte high_byte = percent_speed / 256;
    msg->id = SPEEDWRITE_ADD + offset - MOTOR_STATIC_OFFSET;
    // Serial.println(msg->id);
    msg->ext = 0;
    msg->len = 8;
    msg->buf[0] = low_byte; // NM
    msg->buf[1] = high_byte;
    msg->buf[2] = 0; // Speed
    msg->buf[3] = 0;
    msg->buf[4] = direction;           // Direction
    msg->buf[5] = START_BUTTON_PUSHED; // Inverter enable byte
    msg->buf[6] = 0;                   // Last two are the maximum torque values || if 0 then defualt values are set
    msg->buf[7] = 0;
    Messenger.writeMsg(msg);
}