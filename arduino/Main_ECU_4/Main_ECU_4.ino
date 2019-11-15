/*
    SAE - 2019
    Teensy 3.6 ECU x2
    Version 4
*/

#include <IFCT.h>                      // ImprovedFLexCanLibrary
typedef void (*msgHandle)(TTMsg &msg); // for message specialization such as a message block with only flags
typedef void (*flagReader)(void);      // functions that are called when flag bits are true

// TODO: decide on addresses for all the sensors and bms
enum CanADR : uint32_t {
    // motor
    TEMP1 = 0x0A0,
    TEMP2 = 0x0A1,
    TEMP3 = 0x0A2,
    ANLIV = 0x0A3,
    // DIGIS = 0x0A4,
    MOTORPOS = 0x0A5,
    CURRENT = 0x0A6,
    VOLTAGE = 0x0A7,
    FAULT = 0x0AB,
    // IDK
    INFO = 0x1A5,
};

//TODO: make motorPackets work with TTMsg
struct motorDataPkt {
    uint32_t idOffset = 0;
    int values[8][8];  // not all the tables will be used eg. ANLIV & DIGIS
    bool faults[8][8]; //TODO: what should we do with faults?
} motor0, motor1;

// Push data to andriod using Teensy UART | Eg. Serial1.write();
// TODO: add pushAndriod method to Serial1.write message blocks
// TODO: make teensy to andriod decode bytes
// Used to identify what data goes into what message
enum data {  // both teensies two
    NIL = 0, // Pin0 must be sacrificed to the gods to have a nil value

    // Motor data
    // TEMP1
    PhaseA = true,
    PhaseB = true,
    PhaseC = true,
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
    PhaseA = true,
    PhaseB = true,
    PhaseC = true,
    DCBus = true,
    //VOLTAGE
    DCBus = true,
    Output = true,
    VAB_Vd = true,
    VBC_Vq = true,

    // Teensy One
    startButtonPin = 14,
    steeringPin = 15,
    brakePin = 16,
    pedalPin = 17,
    lightsPin = 18,
    accelerator_1 = 21,
    accelerator_2 = 21,

    // Teensy Two
    TSMP = 14,
    // IMD = 15, // ????????????????????????
    gyro = 16,
    brakeLight = 17,
    pump = 18,
};

/*
    Byte # (map)| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | // byte index in CAN message
    H&L 'packet'        | H | L |   =   |  PKT  | // L and H bytes must be next to each other, referred as packets(PKT)
    PKT pos     |  PKT  |  PKT  |  PKT  |  PKT  | // Valid PKT placement NOTE: is set as seprate H & L in actual buffer
    data        |   | S3|   | S2|   | S1|   | S0| // Where data funcs are called (based off pos in TTMsg table | Byte#/2)
    FLAG pos    | F1| F0|                         // Flag bytes if they exist NOTE: only byte 0 will call functions
*/
typedef struct TTMsg : CAN_message_t { // Teensy to Teensy message definition/structure
    uint32_t id;                       // identifies how the msg should be interpreted using it's address
    data packets[4];                   // data that have data in this message; position in table sets where PKT goes (see ^)
    byte offset;                       // now any data can have an offset for duplicates | the 2 motors in this case
    flagReader flagFuncs[8];           // functions that are called when a flag bit is true | limits callbacks to flag byte 0
    msgHandle handle;                  // function that can handle the message instead | for specialization of messages
    byte values[8];                    // values from bytes that can be pushed after reading
    bool containsFlag;                 // used for memoization
} TTMsg;                               // IMPROVE: Flags can be extended to handle two bytes if it is really neccessary

/* ----- ECU specific data ----- */

byte MOTOROFFSET = 0x30;

bool carUnlocked = false;

void initalizeCar() {   // Start button has been pressed
    carUnlocked = true; // Let the car be able to move
    // TODO: include precharge sequence
}

// this message only checks the flag
TTMsg info{
    INFO,
    {},
    0,
    {initalizeCar},
};

TTMsg motorTemp1{
    TEMP1,
    {
        PhaseA,
        PhaseB,
        PhaseC,
        DriverBoard,
    },
    MOTOROFFSET,
};

TTMsg motorTemp2{
    TEMP2,
    {
        ControlBoard,
        RTD1,
        RTD2,
        RTD3,
    },
    MOTOROFFSET,
};

// load all the ECU specific messages
TTMsg TTMessages[]{
    info,
};

void initalizeMsg(TTMsg &msg) {
    if (*(msg.flagFuncs)) { // use bool to see if flags are at byte 0 | is this better? idk
        msg.containsFlag = true;
        if (msg.packets[4]) { // if a flags exist and so do all four data slots then this is a problem
            Serial.println("WARNING: FLAG AND MESSAGE CONFLICT! ID:" + msg.id);
        } // TODO: find a way to display errors
    }
}

TTMsg offsetMsg(TTMsg &msg) {
    TTMsg dup;
    dup.id = msg.id + msg.offset;
    *dup.packets = *msg.packets;
    *dup.flagFuncs = *msg.flagFuncs;
    dup.handle = msg.handle;
    return dup;
}

/* ----- END ECU specific data ----- */

void setup() {
    // Set motorID address offsets
    motor1.idOffset = 0x30;
    motor0.idOffset = 0x0;
    for (int j = 0; j < 8; ++j) { // load all the zeros for the motor fields
        for (int k = 0; k < 8; ++k) {
            motor0.faults[j][k] = 0;
            motor1.faults[j][k] = 0;
            motor0.values[j][k] = 0;
            motor1.values[j][k] = 0;
        }
    }

    for (size_t i = 0; i < sizeof(TTMessages); i++) {
        initalizeMsg(TTMessages[i]);
        if (TTMessages[i].offset) {
            initalizeMsg(offsetMsg(TTMessages[i]));
        }
    }

    pinMode(2, OUTPUT); // Fusion Tech's Dual CAN-Bus R pin switch
    digitalWrite(2, LOW);
    Can0.setBaudRate(1000000); // Speeed
    Can0.enableFIFO();         // FirstInFirstOut
}

void loop() {
    CAN_message_t dataIn; // Can message obj
    if (Can0.read(dataIn)) {
        if (motorRead(dataIn, motor0)) { //TODO: move motor id matching to sepreate method
        } else if (motorRead(dataIn, motor1)) {
        } else {
            teensyRead(dataIn);
        }
    }
    for (TTMsg &msg : TTMessages) { // Iterate through defined TTMsgs and push their data
        updateData(msg);
        writeTTMsg(msg);
    }
}

void updateData(TTMsg &msg) {
    if (msg.handle) {
        msg.handle(msg);
    } else {
        for (int i = 0; i < 8; i += 2) {
            if (msg.packets[i / 2]) { // If we have a sensor for this packet read and store it
                int val = analogRead(msg.packets[i / 2]);
                msg.values[i] = val / 255;
                msg.values[i + 1] = val % 255;
            }
        }
    }
}

int decodeByte(const byte low, const byte high) { // probably will only be used to push to andriod
    return high * 255 + low;                      // Does c++ cast the return type? seems to work?
}

void flagScan(const byte &flag, flagReader funcTbl[8]) {
    if (flag) {                                    // check if flag has any true bits
        for (byte bit = 0; bit < 8; ++bit) {       // iterate though flag bits
            if (funcTbl[bit] && (flag >> bit) & 1) // check that we can do something if the bit is true
                funcTbl[bit]();                    // call function based off bit pos
        }
    }
}

void writeTTMsg(const TTMsg &msg) { // TODO: can't we just get rid of this?
    Can0.write(msg);
}

//IMPROVE: instead of just storing values and pushing later, why not push as they are recieved? Consult leads!
void readTTMsg(TTMsg &msg, const byte buf[8]) {
    size_t i = 0;
    if (msg.containsFlag) {              // Readflags if they are expected
        flagScan(buf[i], msg.flagFuncs); // Only checking byte 0
        msg.values[i] = buf[i];          // Store byte 0 of flags
        msg.values[i + 1] = buf[i + 1];  // also stores byte 1 for completion sake
        i = 2;                           // Skip flag bytes
    }
    for (i = i; i < 8; i += 2) {
        if (msg.packets[i]) {               // are we expecting data on this packet?
            msg.values[i] = buf[i];         // don't encode as there is no immediate need
            msg.values[i + 1] = buf[i + 1]; // no encode
        }
    }
}

// Iterate through defined TTMsgs and check if the address is one of theirs
void teensyRead(const CAN_message_t &dataIn) {
    for (TTMsg &msg : TTMessages) {
        if (msg.id == dataIn.id) {
            readTTMsg(msg, dataIn.buf); // id matches; interpret data based off matching msg structure
            break;
        };
    }
}

// motor functions

// TODO: test what the accelerators are outputting
void motorPushSpeed(TTMsg &msg) {
    int average = 0;
    int accelerator0 = analogRead(accelerator_1);
    int accelerator1 = analogRead(accelerator_2);
    if (accelerator0 < 5 || accelerator1 < 5) { // To check and clean if the value is jumping around
        accelerator0 = 0;
        accelerator1 = 0;
    }
    float errorcheck = (abs(accelerator0 - accelerator1) / accelerator1); // Percent error
    if (errorcheck <= 0.1) {                                              // if the error is less than 10%
        average = (accelerator0 + accelerator1) / 2;
    } else {
        Serial.println("Error accelerator reading");
    }
}

// Max torque speed is 100 NM || 0 = Clockwise  1 = CounterClockwise
void write_speed(int speed, bool m_direction, bool enable_pin, int id_off) {
    CAN_message_t dataOut;                               // Can message obj
    speed = (speed > 860) ? 860 : 1;                     // IMPROVE: does this not mean %speed is always <= 100 after map?
    uint32_t percent_speed = map(speed, 0, 860, 0, 100); // Converts analog to motor values (NM) || 100NM = 1000 in Code
    if (percent_speed < 1000) {                          // Checks if below 1000
        //Calculations value = (high_byte x 256) + low_byte
        byte low_byte = percent_speed % 256;
        byte high_byte = percent_speed / 256;

        //Setting up sending data parameters
        dataOut.ext = 0;
        dataOut.id = 0x0C0 + id_off; // Command message ID
        dataOut.len = 8;
        dataOut.buf[0] = low_byte; // NM
        dataOut.buf[1] = high_byte;
        dataOut.buf[2] = 0; // Speed
        dataOut.buf[3] = 0;
        dataOut.buf[4] = m_direction; // Direction
        dataOut.buf[5] = carUnlocked; // Inverter enable byte
        dataOut.buf[6] = 0;           // Last two are the maximum torque values || if 0 then defualt values are set
        dataOut.buf[7] = 0;

        Can0.write(dataOut);
    } else {
        Serial.println("Exceeding max torque value within write speed function.");
    }
}

bool motorRead(const CAN_message_t &dataIn, motorDataPkt &packet) {
    uint32_t offst = packet.idOffset;
    // use map to check if id is within motor id range + motor offset
    byte pos = map(dataIn.id, TEMP1 + offst, VOLTAGE + offst, 0, 8);
    if (pos <= 7) {
        motorDecodeData(dataIn, packet.values[pos]);
        return true;
    } else if (dataIn.id == FAULT + offst) {
        motorReadFault(dataIn, packet.faults);
    }
    return false;
}

void motorDecodeData(const CAN_message_t &dataIn, int *valueTbl) {
    for (size_t i = 0; i < 8; i += 2) {
        valueTbl[i] = dataIn.buf[i];
        valueTbl[i + 1] = dataIn.buf[i + 1];
    }
}

void motorReadFault(const CAN_message_t &dataIn, bool faultTbl[8][8]) {
    for (int col = 0; col < 8; ++col) {                        // for each byte
        if (dataIn.buf[col]) {                                 // If the byte has info
            for (int row = 0; row < 8; ++row) {                // for each bit
                if (((dataIn.buf[col] >> row) & 0B00000001)) { // If each bit is true, store value
                    faultTbl[col][row] = 1;
                } else { // No fault
                    faultTbl[col][row] = 0;
                }
            }
        }
    }
}

/*
    "Yeet"
        -Bobamba
                    */