/*
    SAE - 2019
    Teensy 3.6 ECU x2
    Version 4
*/

#include <IFCT.h> // ImprovedFLexCanLibrary using only Can0
struct TTMsg;
typedef uint32_t uint32;          // clean it up a lil
typedef bool (*msgHandle)(TTMsg); // for message specialization such as a message block with only flags
typedef void (*flagReader)(void); // functions that are called when flag bits are true

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
    BMS_VOLT_ADD = 0x00,
};

// Used to identify what data goes into what message
enum validData {
    NIL = 0, // Pin0 must be sacrificed to the gods to have a nil value

    // Motor data
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

    // Both Teensy's
    boardLed = 13,

    // Teensy One    0x400-0x405
        // Send
            // button
                startButtonPin = 14,
            // range
                steeringPin = 15,
                brakepressurePin = 16,
                accelerator_1 = 21,
                accelerator_2 = 21, // for double checking
        // Receive
            // buttons
                lightsIMDPin = 18,
                lightsBMSPin = 18,

    // Teensy Two       0x406-0x40A
        // Send
            // button
                lightsIMDPin = 18,
        // special
                // IMD = 15, // ????????????????????????
                PrechargeairPin = 99,
                brakeLight = 17,
                PrechargeRelayPin = 99,
                dischargeactive_pin = 99,
                pump = 18, // don't worry ;)
};

/*
    Byte # (map)| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | // byte index in CAN message
    H&L 'packet'        | H | L |   =   |  PKT  | // L and H bytes must be next to each other, referred as packets(PKT)
    PKT pos     |  PKT  |  PKT  |  PKT  |  PKT  | // Valid PKT placement NOTE: is set as seprate H & L in actual buffer
    data        |   | S3|   | S2|   | S1|   | S0| // Where data funcs are called (based off pos in TTMsg table | Byte#/2)
    FLAG pos    | F1| F0|                         // Flag bytes if they exist NOTE: only byte 0 will call functions
*/

// TODO: actually inistalize TTMsg and CAN_message_t values in constructor instead of depending on just the default values!
struct TTMsg : public CAN_message_t { // Teensy to Teensy message definition/structure
    validData *packets;               // data that have data in this message; position in table sets where PKT goes (see ^) // points to table of 4
    byte offset = 0;                  // now any data can have an offset for duplicates
    flagReader *flagFuncs;            // functions that are called when a flag bit is true | limits callbacks to flag byte 0 // points to table of 8
    validData *flagValues;            // sensor pins to read and push onto the flag byte | only flag byte 0 // points to table of 8
    msgHandle handle = 0;             // function that can handle the message instead | for specialization of messages
    bool containsFlag = 0;            // used for memoization
    int data[4] = {0};                // store decoded or pin data for later use
    TTMsg(validData p[4] = {0}, flagReader fF[8] = {0}, validData fV[8] = {0}) {
        packets = p;
        flagFuncs = fF;
        flagValues = fV;
    };
    TTMsg(uint32 i, validData p[4], byte o, flagReader fF[8], validData fV[8], msgHandle h, byte b[8], bool c) {
        TTMsg(p, fF, fV);
        id = i;
        offset = o;
        handle = h;
        containsFlag = c;
    }
    TTMsg(uint32 i, msgHandle h) { // purely handled by separate functions
        TTMsg();
        id = i;
        handle = h;
    }
    TTMsg(uint32 i, const validData (&p)[4]) { // only stores data
        TTMsg(p);
        id = i;
    }
}; // IMPROVE: Flags can be extended to handle two bytes if it is really neccessary

/* 
    ----- SRT ECU specific data -----  
                                        */

// Globals!?
uint32 MOTOR_OFFSET = 0xe0;         // offset for motor ids
uint32 MOTOR_STATIC_OFFSET = 0x0A0; // IMPROVE: auto set this global offset to addresses
bool DO_PRECHARGE = true;           // Precharge latching variable
bool CAR_UNLOCKED = false;          // MC enable bit state

// Handles
void initalizeCar() {
    CAR_UNLOCKED = true;
    Serial.println("MOTORS UNLOCKED");
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
TTMsg WriteSpeed = TTMsg(SPEEDWRITE_ADD, motorPushSpeed);
TTMsg MCReset = TTMsg(RESETMC_ADD - MOTOR_STATIC_OFFSET, MCResetFunc);
TTMsg precharge = TTMsg(VOLTAGE_ADD - MOTOR_STATIC_OFFSET, prechargeFunc);
TTMsg bmsVolt = TTMsg(BMS_VOLT_ADD, {BMSVolt}); //TODO: does this address only have the bms voltage?

struct ECUData {                       // pointers to data that will be used within teensy itself | ex. BMS_VOLTAGE
    int *BMSVolt_p = &bmsVolt.data[0]; // we must keep track of what is being stored where in the packet setup
} ECUData;                             // IMPROVE: make more readable way to store data

// load messages as read or write
TTMsg ReadTTMessages[]{
    precharge,
    bmsVolt,
};

TTMsg WriteTTMessages[]{
    WriteSpeed,
    MCReset,
};

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
    TTMsg dup;
    dup.id = msg.id + msg.offset;
    *dup.packets = *msg.packets;
    *dup.flagFuncs = *msg.flagFuncs;
    dup.handle = msg.handle;
    return dup;
}

/* 
    ----- END ECU specific data -----  
                                        */

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

void setup() {
    delay(3000);
    Serial.begin(19200);
    pinMode(boardLed, OUTPUT);
    Serial.println("Hello!");
    LEDBlink();

    for (auto msg : WriteTTMessages) {
        initalizeMsg(msg);
        if ((msg).offset) {
            initalizeMsg(offsetMsg(msg));
        }
    }
    for (auto msg : ReadTTMessages) {
        initalizeMsg(msg);
        if ((msg).offset) {
            initalizeMsg(offsetMsg(msg));
        }
    }

    pinMode(2, OUTPUT); // Fusion Tech's Dual CAN-Bus R pin switch
    digitalWrite(2, LOW);
    Can1.setBaudRate(500000); // Speeed
    Can1.enableFIFO();        // FirstInFirstOut
    LEDBlink();
    digitalWriteFast(boardLed, LOW);
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

void updateData(TTMsg msg) {
    if (msg.handle && !(msg.handle)(msg)) { // if the handle exists and returns true upon calling continue execution
        return;
    }
    for (int i = 0; i < 8; i += 2) {
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
    long value = 0;
    long full_data = high * 255 + low;
    if (high < 128) { // positive
        value = full_data;
    } else if (high > 128) { //neg
        value = map(full_data, 65280, 32640, 0, -32640);
        return value;
    }
}

void flagScan(const byte &flag, flagReader funcTbl[8]) { // Only used by read messages
    if (flag) {                                          // check if flag has any true bits
        for (byte bit = 0; bit < 8; ++bit) {             // iterate though flag bits
            if (funcTbl[bit] && (flag >> bit) & 1)       // check that we can do something if the bit is true
                funcTbl[bit]();                          // call function based off bit pos
        }
    }
}

// Push data to andriod using Teensy UART | Eg. Serial1.write();
// TODO: add way to push message blocks to andriod with Serial1.write
// IMPROVE: make andriod decode bytes
void writeTTMsg(const TTMsg msg) { // TODO: can't we just get rid of this?
    Can1.write(msg);
}

void readTTMsg(TTMsg msg, const byte buf[8]) {
    size_t i = 0;
    if (msg.containsFlag) {              // Readflags if they are expected
        flagScan(buf[i], msg.flagFuncs); // Only checking byte 0
        msg.buf[i] = buf[i];             // Store byte 0 of flags
        msg.buf[i + 1] = buf[i + 1];     // also stores byte 1 for completion sake
        i = 2;                           // Skip flag bytes
    }
    for (i = i; i < 8; i += 2) {
        if (msg.packets[i]) {                                     // are we expecting data on this packet?
            msg.data[i / 2] = decodeLilEdian(buf[i], buf[i + 1]); //decode and store
            //do we need to store the sepreate bytes? we are now storing the decoded data
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

    int avgSpeed = 0;
    int accelerator0 = analogRead(accelerator_1); // should be in the ecudata pointer
    int accelerator1 = analogRead(accelerator_2);
    if (accelerator0 < 5 || accelerator1 < 5) { // To check and clean if the value is jumping around
        accelerator0 = 0;
        accelerator1 = 0;
    }

    float errorcheck = abs(accelerator0 - accelerator1) / accelerator1; // Percent error
    if (errorcheck <= 0.1) {                                            // if the error is less than 10%
        avgSpeed = (accelerator0 + accelerator1) / 2;
    } else {
        Serial.println("Error accelerator reading"); // ERROR?
    }
    // "the value of the tquore needs to be a power of 10 of the actual tourqe;" by dominck
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
    msg.buf[4] = direction;    // Direction
    msg.buf[5] = CAR_UNLOCKED; // Inverter enable byte
    msg.buf[6] = 0;            // Last two are the maximum torque values || if 0 then defualt values are set
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
