/*
    SAE - 2019
    Teensy 3.6 ECU x2
    Version 4
*/

#include <IFCT.h> // ImprovedFLexCanLibrary using only Can0
struct TTMsg;
typedef uint32_t uint32;          // clean it up a lil
typedef bool (*msgHandle)(TTMsg); // for message specialization such as a message block with only flags
typedef void (*flagReader)(bool); // functions that are called when flag bits are true
 
// TODO: decide on addresses for all the sensors and bms
enum CanADR : uint32 {
    // motor
    SPEEDWRITE_ADD = 0x0C0,
    RESETMC_ADD = 0x0C1,
    TEMP1_ADD = 0x0A0,
    TEMP2_ADD = 0x0A1,
    TEMP3_ADD = 0x0A2,
    ANLIV_ADD = 0x0A3,
    // DIGIS_ADD = 0x0A4,
    MOTORPOS_ADD = 0x0A5,
    CURRENT_ADD = 0x0A6,
    VOLTAGE_ADD = 0x0A7,
    FAULT_ADD = 0x0AB,
    // IDK
    INFO_ADD = 0x1A5,
    // BMS
    BMS_STATS_ADD = 0x250,
    MOTORL_ADD = 0xE0,
    MOTORR_ADD = 0x00,
    // TEENSY2TEENSY
    T2T_ADD = 0xFFF,
    T2T2_ADD = 0xFFF,
};

// Used to identify what data goes into what message
enum validData : byte {
    NIL = 0, // Pin0 must be sacrificed to the gods to have a nil value

    // Motors
    MotorLTemp = true,
    MotorRTemp = true,
    rpmLWheel = true, // Is this from MCs or motors themselves?
    rpmRWheel = true,

    // Motor controller
    // TEMP1
    PhaseATemp = true,
    PhaseBTemp = true,
    PhaseCTemp = true,
    DriverBoard = true,
    // TEMP2
    ControlBoard = true,
    RTD1 = true,
    RTD2 = true,
    RTD3 = true,
    // TEMP3
    RTD4 = true,
    RTD5 = true,
    motor = true,
    torqueShudder = true,
    //MOTORPOS
    angle = true,
    anglrVel = true,
    electricalFreq = true,
    deltaResolver = true, // Unused
    //CURRENT
    PhaseACur = true,
    PhaseBCur = true,
    PhaseCCur = true,
    DCBusCur = true,
    //VOLTAGE
    DCBusV = true,
    OutputV = true,
    VAB_Vd = true,
    VBC_Vq = true,
    //BMS
    BMSVolt = true,
    BMSTemp = true,
    BMSSOC = true,

    // accelerator
    avgAccel = true, // Manually set

    //T2T //TODO: Do these valid data belong here?
    breakPress = 99,
    steeringAng = 99,
    carMode = true,
    pedalAir = true,

    // Both Teensy's
    boardLed = 13,
    lightsIMDPin = 18,

    // Teensy One    0x400-0x405
    // Send
    // button
    startButton = 14,
    IMDReadLight = 99,
    AMSReadLight = 99,
    // range
    steeringPin = 15,
    brakepressurePin = 16,
    accelerator_1 = 21,
    accelerator_2 = 21, // for double checking
                        // Receive
                        // buttons
    lightsBMSPin = 18,

    // Teensy Two       0x406-0x40A
    // Send
    // button
    // special
    // IMD = 15, // ????????????????????????
    IMDLight = 99,
    AMSLight = 99,
    PrechargeairPin = 99,
    brakeLight = 17,
    PrechargeRelayPin = 99,
    dischargeactive_pin = 99,
    pump = 18, // don't worry ;;;;;)))))

};

/*
    Byte # (map)| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | // byte index in CAN message
    H&L 'packet'        | H | L |   =   |  PKT  | // L and H bytes must be next to each other, referred as packets(PKT)
    PKT pos     |  PKT  |  PKT  |  PKT  |  PKT  | // Valid PKT placement NOTE: is set as seprate H & L in actual buffer
    data        |   | S3|   | S2|   | S1|   | S0| // Where data funcs are called (based off pos in TTMsg table | Byte#/2)
    FLAG pos    | F1| F0|                         // Flag bytes if they exist NOTE: only byte 0 will call functions
    FLAG bits   |--F1--|0|1|3|4|5|6|7|--F0--|0|1|2|3|4|5|6|7| // how the flags are stored
*/

// TODO: actually initalize TTMsg and CAN_message_t values in constructor instead of depending on just the default values!
struct TTMsg : public CAN_message_t {                                                          // Teensy to Teensy message definition/structure
    validData *packets;                                                                        // data that have data in this message; position in table sets where PKT goes (see ^) // points to table of 4
    uint32 offset = 0;                                                                         // now any data can have an offset for duplicates
    flagReader *flagFuncs;                                                                     // functions that are called when a flag bit is true | limits callbacks to flag byte 0 // points to table of 8
    validData *flagValues;                                                                     // sensor pins to read and push onto the flag byte | only flag byte 0 // points to table of 8
    msgHandle handle = 0;                                                                      // function that can handle the message instead | for specialization of messages
    bool containsFlag = 0;                                                                     // used for memoization
    int data[4] = {0};                                                                         // store decoded or pin data for later use
    void Base(uint32 i, validData p[4] = {0}, flagReader fF[8] = {0}, validData fV[8] = {0}) { // TODO: test if it actually initalizes correctly
        id = i;
        packets = p;
        flagFuncs = fF;
        flagValues = fV;
    }
    TTMsg(uint32 i) { // blank msg
        Base(i);
    };
    TTMsg(uint32 i, msgHandle h) { // purely handled by separate functions
        Base(i);
        handle = h;
    }
    TTMsg(uint32 i, const validData (&p)[4]) { // only stores data
        Base(i, (validData *)(&p));
    }
    TTMsg(uint32 i, const validData (&p)[4], const flagReader (&fF)[8], const validData (&fV)[8]) { // data storage for packets and reactive flags
        Base(i, (validData *)(&p), (flagReader *)(&fF), (validData *)(&fV));
    }
    TTMsg(uint32 i, validData p[4], flagReader fF[8], validData fV[8], msgHandle h) { // for duplication purposes
        Base(i, p, fF, fV);
        handle = h;
    }
}

; // IMPROVE: Flags can be extended to handle two bytes if it is really neccessary

/* 
    ----- SRT ECU specific data -----  
                                        */

// Globals!?
uint32 MOTOR_OFFSET = 0xe0;         // offset for motor ids
uint32 MOTOR_STATIC_OFFSET = 0x0A0; // IMPROVE: auto set this global offset to addresses
bool DO_PRECHARGE = true;           // Precharge latching variable
bool START_BUTTON_PUSHED = false;   // MC enable bit state
bool PEDAL_ERROR = false;
bool CAR_MODE = false; // true: RaceMode false: EcoMode
const byte Teensy2SerialArrSize = 12;
int Teensy2SerialArr[Teensy2SerialArrSize];

// Flag Handles
void initalizeCar(bool bit) {
    START_BUTTON_PUSHED = true;
    Serial.println("MOTORS UNLOCKED");
}

void setPedalState(bool bit) {
    PEDAL_ERROR = bit;
}

void IMDSetLight(bool bit) {
    digitalWriteFast(IMDLight, bit);
}

void AMDSetLight(bool bit) {
    digitalWriteFast(AMSLight, bit);
}

void setCarMode(bool bit) {
    // TODO: ensure car is not moving
    if (*ECUData.MCMotorAng < 5) { // rpm = motor controller rpm
        CAR_MODE = bit;
    }
}

//handles
void Fan() {
  23/A9 - FAN1_PWM //fan pins
  22/A8 - FAN2_PWM
  21/A7 - FAN3_PWM
  20/A6 - FAN4_PWM

  8 - SIG_SERVO1_PWM //motor pins
  27 - SIG_SERVO2_PWM

  24 - SIG_FANS_ON/OFF //didn't know if you need this in the fan function.
  A21/DAC0 - SIG_PUMP_ANALOG

      int FanSpeed;
      int MotorSpeed;
      int AvgMotorSpeed //whatever the average motor speed is

      pinMode(fan, OUTPUT);
      pinMode(motorPin, INPUT);

      if( AvgMotorSpeed <0 ) {//some number close to zero
        FanSpeed = 0;
    }
    else( AvgMotorSpeed > 0){//some number close to zero
        FanSpeed = //whatever Fan Speed .
        analogWrite(fan, FanSpeed); //actually spins fan at the FanSpeed.
      }

    }

bool MCResetFunc(TTMsg msg) { // MC Fault reseter thing
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
    writeTTMsg(msg);
    return true;
} // IMPROVE: There may be reliability issues with only sending one?

bool prechargeFunc(TTMsg msg) {
    if (digitalRead(dischargeactive_pin)) { // if airs have no power, then precharge must be checked
        DO_PRECHARGE = 1;
    }
    float MC_voltage = abs(decodeLilEdian(msg.buf[0], msg.buf[1])) / 10; //Returns in power of 10s
    if (!digitalRead(dischargeactive_pin) && DO_PRECHARGE) {             //if airs have no power but had before, then begin precharge circuit
        digitalWrite(PrechargeairPin, LOW);                              //Keep air open
        digitalWrite(PrechargeRelayPin, HIGH);                           //precharge is closed

        if (*ECUData.BMSVolt_p >= 150 && (*ECUData.BMSVolt_p * 0.9) <= MC_voltage) { // BMS voltage is a global
            DO_PRECHARGE = 0;
        }
    } else { // Can use any MCs voltage, will be the same, must be greater than 270V (0.9 * 300V)
        // Should return to normal state
        digitalWrite(PrechargeairPin, HIGH);  //close air
        digitalWrite(PrechargeRelayPin, LOW); //precharge is off
    }
}

// Initalize messages
// TODO: do both MOTOR CONTROLLERS!
TTMsg WriteSpeed = TTMsg(SPEEDWRITE_ADD, motorPushSpeed);
TTMsg MCReset = TTMsg(RESETMC_ADD - MOTOR_STATIC_OFFSET, MCResetFunc);
TTMsg MCTempRead = TTMsg(RESETMC_ADD - MOTOR_STATIC_OFFSET, MCResetFunc);
TTMsg MCMotorPos = TTMsg(MOTORPOS_ADD - MOTOR_STATIC_OFFSET, {angle});
TTMsg MCFaults = TTMsg(FAULT_ADD, pruneFaults);
TTMsg precharge = TTMsg(VOLTAGE_ADD - MOTOR_STATIC_OFFSET, prechargeFunc);
TTMsg bmsStat = TTMsg(BMS_STATS_ADD, {BMSTemp, BMSVolt, BMSSOC});
TTMsg motorL = TTMsg(MOTORL_ADD, {MotorLTemp});
TTMsg motorR = TTMsg(MOTORR_ADD, {MotorRTemp});
TTMsg T2TData = TTMsg(T2T_ADD, {avgAccel, breakPress, steeringAng}, {NULL, NULL, NULL, initalizeCar, NULL}, {IMDReadLight, AMSReadLight, carMode, startButton, pedalAir});
TTMsg T2T2Data = TTMsg(T2T2_ADD, {rpmLWheel, rpmRWheel});

// anything that says MC, motor controller, needs to be doubled
struct ECUData {                       // pointers to data that will be used within teensy itself | ex. BMS_VOLTAGE
    int *BMSVolt_p = &bmsStat.data[1]; // we must keep track of what is being stored where in the packet setup
    int *BMSTEMP_p = &bmsStat.data[0];
    int *LMTEMP_P = &motorL.data[0];
    int *RMTEMP_P = &motorR.data[0];
    int *MCMotorAng = &MCMotorPos.data[0];
    int *MCTEMP_P = &MCTempRead.data[0];
    int *BMSSOC_P = &bmsStat.data[2];        // state of charge
    int *BMSBUSCURRENT_P = &bmsStat.data[2]; // bus current
    int *T2TACCEL_P = &T2TData.data[0];      // avgAccel
} ECUData;                                   // IMPROVE: make more readable way to store data

// TODO: faults! IMD is digital and BMS is seperate address
// TODO: calculate average speed
// TODO: Active aero

int T2AMsg[11];

void pushT2A() { // final push to tablet | arraysize: Teensy2SerialArrSize array: Teensy2SerialArr
    String s = "S ";
    T2AMsg[0] = *ECUData.BMSVolt_p;
    T2AMsg[1] = *ECUData.BMSTEMP_p;
    T2AMsg[2] = 0; // avgSpeed go here
    T2AMsg[3] = *ECUData.LMTEMP_P;
    T2AMsg[4] = *ECUData.RMTEMP_P;
    T2AMsg[5] = *ECUData.MCTEMP_P;
    T2AMsg[6] = 0; // do both mc!
    T2AMsg[7] = 0; // aero
    T2AMsg[8] = *ECUData.BMSSOC_P;
    T2AMsg[9] = *ECUData.BMSBUSCURRENT_P;
    // T2AMsg[10] = buildFaultList(); //gets updated by fault handler
    for (int i; i < 11; i++) {
        s += T2AMsg[i] + " ";
    }
    Serial.println(s);
}

bool pruneFaults(TTMsg msg) { // figure which bits go where
    int final = 0;
    final = msg.buf[4];
    final = final << 8;
    final |= msg.buf[5];
    final = final >> 3; // remove "reserved" bits
    final = final << 1; // for IMD fault
    final |= 1;         // imd fault 1:0
    final = final << 1; // for BMS fault
    final |= 1;         // bms fault 1:0
    T2AMsg[10] = final;
}

// load messages as read or write
TTMsg ReadTTMessages[]{
    precharge,
    bmsStat,
};

TTMsg WriteTTMessages[]{
    WriteSpeed,
    MCReset,
};

/* 
    ----- END ECU specific data -----  
                                        */

void initalizeMsg(TTMsg msg) {
    // TODO: does this ensure no func is set for any flag?
    if (msg.flagFuncs) { // use bool to see if flags are at byte 0
        msg.containsFlag = true;
        if (msg.packets[3]) { // if a flags exist and so do all four data slots then this is a problem
            Serial.print("WARNING: FLAG AND MESSAGE CONFLICT! ID:");
            Serial.println(msg.id, HEX);
        } // TODO: find a way to display errors
    }
}

TTMsg offsetMsg(TTMsg msg) { // duplicates message blocks; allows the offset block to have seperate read/write data
    TTMsg dup = TTMsg(msg.id + msg.offset, msg.packets, msg.flagFuncs, msg.flagValues, msg.handle);
    return dup;
}

// Led blinkery stuff
bool loopSwitch = false;
void toggleLED() {
    digitalWriteFast(boardLed, loopSwitch);
    loopSwitch = !loopSwitch;
}

void LEDBlink() { // how does the teensy check for start button when it should be off when pressed???
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
// are fault messages all just flags?
// TODO: when a fault is detected read serial

void setup() {
    delay(3000);
    Serial.begin(19200);
    pinMode(boardLed, OUTPUT);
    Serial.println("Hello!");
    LEDBlink();

    for (auto msg : WriteTTMessages) {
        initalizeMsg(msg);
        if (msg.offset) {
            initalizeMsg(offsetMsg(msg));
        }
    }

    for (auto msg : ReadTTMessages) {
        initalizeMsg(msg);
        if (msg.offset) {
            initalizeMsg(offsetMsg(msg));
        }
    }

    pinMode(2, OUTPUT); // Fusion Tech's Dual CAN-Bus R pin switch
    digitalWrite(2, LOW);
    Can1.setBaudRate(500000); // Speeed
    Can1.enableFIFO();        // FirstInFirstOut
    LEDBlink();
    digitalWriteFast(boardLed, LOW);
    pushT2A(); // Teensy to andriod
}

CAN_message_t dataIn; // Can data in obj
void loop() {
    if (Can1.read(dataIn)) {
        teensyRead(dataIn);
    }

    for (TTMsg msg : WriteTTMessages) { // Iterate through defined TTMsgs and push their data
        updateData(msg);
    }
}

void accelCheck() { // read accel numbrs and sync with T2T line
    int a1 = analogRead(accelerator_1);
    int a2 = analogRead(accelerator_2);
    if (a1 < 5 || a2 < 5) { // To check and clean if the value is jumping around
        a1 = 0;
        a2 = 0;
    }
    float errorcheck = abs(a1 - a2) / a1;    // Percent error
    if (errorcheck <= 0.1) {                 // if the error is less than 10%
        *ECUData.T2TACCEL_P = (a1 + a2) / 2; // this is how you do it right?
    } else {
        Serial.println("Error accelerator reading"); // ERROR?
    }
}

void updateData(TTMsg msg) {
    if (msg.handle && !(msg.handle)(msg)) { // if the handle exists and returns true upon calling continue execution
        return;
    }
    byte stop = 8;
    if (msg.containsFlag) { // Readflags if they are expected
        flagRead(msg);      // Check bytes
        stop = 6;           // Skip flag bytes
    }
    for (byte i = 0; i < stop; i += 2) {
        if (msg.packets[i / 2]) {                     // If we have a sensor for this packet read and store it
            int val = analogRead(msg.packets[i / 2]); // TODO: Some sensors are digital not just analog!
            msg.data[i / 2] = val;                    // store the raw value
            msg.buf[i] = val % 256;
            msg.buf[i + 1] = val / 256;
        }
    }
    writeTTMsg(msg);
}

// IMPROVE: anyway to make this take advantage of the compiler?
int decodeLilEdian(const byte low, const byte high) {
    int value = 0;
    int full_data = high * 255 + low;
    if (high < 128) { // positive
        value = full_data;
    } else if (high > 128) { //neg
        value = map(full_data, 65280, 32640, 0, -32640);
    }
    return value;
}

void flagRead(TTMsg msg) {                                          // read pins that map to flag variables
    for (byte i = 7; i <= 6; i--) {                                 // capped to first two bytes
        msg.buf[i] = 0;                                             // clear flags
        for (byte bit = 0; bit < 8; ++bit) {                        // iterate through byte bits
            if (msg.flagValues[bit]) {                              // check if there is a flag defined
                msg.buf[i] |= digitalReadFast(msg.flagValues[bit]); // store flag
            }                                                       //
            msg.buf[i] = msg.buf[i] << 1;                           // shift bits
        }
    }
}

void flagScan(const byte &flagByte, flagReader funcTbl[8]) { // Only used by read messages
    if (flagByte) {                                          // check if flag has any true bits
        for (byte bit = 0; bit < 8; ++bit) {                 // iterate though flag bits
            if (funcTbl[bit]) {                              // check that we can do something if the bit is true
                funcTbl[bit]((flagByte >> bit) & 1);         // call function based off bit pos
            }
        }
    }
}

// Push data to andriod using Teensy UART | Eg. Serial1.write();
// TODO: add way to push message blocks to andriod with Serial1.write
// IMPROVE: make andriod decode bytes
void writeTTMsg(TTMsg msg) { // TODO: can't we just get rid of this?
    // Write2Andriod(msg);
    Can1.write(msg);
}

void readTTMsg(TTMsg msg, const byte buf[8]) {
    byte stop = 8;
    if (msg.containsFlag) {              // Readflags if they are expected
        flagScan(buf[7], msg.flagFuncs); // Only checking byte 0
        msg.buf[7] = buf[7];             // Store byte 0 of flags
        msg.buf[6] = buf[6];             // also stores byte 1 for completion sake
        stop = 6;                        // Skip flag bytes
    }
    for (byte i = 0; i < stop; i += 2) {
        if (msg.packets[i]) {                                     // are we expecting data on this packet?
            msg.data[i / 2] = decodeLilEdian(buf[i], buf[i + 1]); // decode and store
            // do we need to store the sepreate bytes? we are now storing the decoded data
            // msg.buf[i] = buf[i];                                  // store lowByte
            // msg.buf[i + 1] = buf[i + 1];                          // store highByte
        }
    }
}

// Iterate through defined TTMsgs and check if the address is one of theirs
void teensyRead(const CAN_message_t &dataIn) {
    for (TTMsg msg : ReadTTMessages) {
        if ((msg).id == dataIn.id) {
            readTTMsg(msg, dataIn.buf); // id matches; interpret data based off matching msg structure
            break;
        };
    }
}

// motor functions

// TODO: test what the accelerators are outputting
bool motorPushSpeed(TTMsg msg) {
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

void motorWriteSpeed(TTMsg msg, byte offset, bool direction, int speed) { // speed is value 0 - 860
    int percent_speed = constrain(map(speed, 0, 1024, 0, 400), 0, 400);   // seprate func for negative vals (regen)
    // Serial.println(percent_speed);
    //Calculations value = (high_byte x 256) + low_byte
    byte low_byte = percent_speed % 256;
    byte high_byte = percent_speed / 256;
    msg.id = SPEEDWRITE_ADD + offset - MOTOR_STATIC_OFFSET;
    // Serial.println(msg.id);
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
    writeTTMsg(msg);
}

// Debug Funcs
void printMsg(TTMsg const &msg) { // Print out can msg buffer w/ ID
    Serial.print(msg.id, HEX);
    Serial.println(" = ");
    Serial.println(msg.buf[0]);
    Serial.println(msg.buf[1]);
    Serial.println(msg.buf[2]);
    Serial.println(msg.buf[3]);
    Serial.println(msg.buf[4]);
    Serial.println(msg.buf[5]);
    Serial.println(msg.buf[6]);
    Serial.println(msg.buf[7]);
}
