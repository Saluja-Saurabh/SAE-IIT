/*
    SAE - 2019
    Teensy 3.6 ECU x2
    Version 4
*/

#include <IFCT.h> // ImprovedFLexCanLibrary using only Can0
struct TTMsg;
typedef bool (*msgHandle)(TTMsg); // for message specialization such as a message block with only flags
typedef void (*flagReader)(void); // functions that are called when flag bits are true

// bms voltage global, voltage grabbed in function: read_bms_volt_fun
float BMS_VOLTAGE = 0;

// TODO: decide on addresses for all the sensors and bms
enum CanADR : uint32_t {
    // motor
    SPEEDWRITE = 0x0C0,
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
    // BMS
    BMS_VOLTAGE_ID = 0x00,
};

// Used to identify what data goes into what message
enum data {
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

    // Both Teensy's
    boardLed = 13,

    // Teensy One
    startButtonPin = 14,
    steeringPin = 15,
    brakePin = 16,
    pedalPin = 17,
    lightsPin = 18,
    accelerator_1 = 21,
    accelerator_2 = 21, // Should these be the same

    // Teensy Two
    TSMP = 14,
    // IMD = 15, // ????????????????????????
    gyro = 16,
    brakeLight = 17,
    pump = 18,
    PrechargeairPin = 99,
    PrechargeRelayPin = 99,
    dischargeactive_pin = 99,
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
    // uint32_t id = 0;             // identifies how the msg should be interpreted using it's address
    data packets[4] = {NIL};       // data that have data in this message; position in table sets where PKT goes (see ^)
    byte offset = 0;               // now any data can have an offset for duplicates | the 2 motors in this case
    flagReader flagFuncs[8] = {0}; // functions that are called when a flag bit is true | limits callbacks to flag byte 0
    data flagValues[8] = {NIL};    // sensor pins to read and push onto the flag byte | only flag byte 0
    msgHandle handle = 0;          // function that can handle the message instead | for specialization of messages
    bool sideHandle = 0;           // handle is called alongside the acutal proccessing of msg
    // byte buf[8] = {0};             // values from bytes that will be pushed after reading
    bool containsFlag = 0; // used for memoization
    TTMsg(){};
    TTMsg(uint32_t i, data p[4], byte o, flagReader fF[8], data fV[8], msgHandle h, bool s, byte b[8], bool c) {
        id = i;
        // packets = p;
        offset = o;
        // flagFuncs = fF;
        // flagValues = fV;
        handle = h;
        sideHandle = s;
        // buf = {b};
        containsFlag = c;
    }
    TTMsg(uint32_t i, flagReader fF[8]) {
        id = i;
        // flagFuncs = fF;
    }
    TTMsg(uint32_t i, msgHandle h) {
        id = i;
        handle = h;
    }
}; // IMPROVE: Flags can be extended to handle two bytes if it is really neccessary

/* ----- ECU specific data ----- */

uint32_t MOTOROFFSET = 0xe0;
uint32_t MOTORSTATICOFFSET = 0x0A0; // IMPROVE: auto set this global offset to addresses

bool carUnlocked = false;
void initalizeCar() {
    carUnlocked = true;
    Serial.println("MOTORS UNLOCKED");
}

// MC Fault reseter thing
bool MCResetFunc(TTMsg msg) { // IMPROVE: There may be reliability issues with only sending one?
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
}

//global for precharge, latching variable
byte DOPRECHARGE = 1;

bool prechargeFunc(TTMsg msg)
{
  if (digitalRead(dischargeactive_pin)) // if airs have no power, then precharge must be checked 
  {
    DOPRECHARGE = 1;
  }
  int current_id = msg.id;  //save before it gets overwritten
  if (Can1.read(msg))
  {
      if (msg.id == current_id)
      {
        float MC_voltage = abs(decodeliledian(msg.buf[0],msg.buf[1])) / 10; //Returns in power of 10s
        if (!digitalRead(dischargeactive_pin) and DOPRECHARGE) //if airs have no power but had before, then begin precharge circuit
        {
          digitalWrite(PrechargeairPin, LOW);  //Keep air open
          digitalWrite(PrechargeRelayPin, HIGH);  //precharge is closed
          if (BMS_VOLTAGE >= 150 and (BMS_VOLTAGE * 0.9) <= MC_voltage) // BMS voltage is a global 
          {
            DOPRECHARGE = 0;
          }
        }
        else// Can use any MCs voltage, will be the same, must be greater than 270V (0.9 * 300V)
        {
          // Should return to normal state
          digitalWrite(PrechargeairPin, HIGH);  //close air
          digitalWrite(PrechargeRelayPin, LOW);  //precharge is off
        }
      }
  }
}

bool read_bms_volt_fun(TTMsg msg)
{
  int current_id = msg.id;  //save before it gets overwritten
  if (Can1.read(msg))
  {
      if (msg.id == current_id)
      {
        BMS_VOLTAGE = abs(decodeliledian(msg.buf[0],msg.buf[1])) / 10; // First two bytes must be read
      }
  }
}

TTMsg WriteSpeed = TTMsg(SPEEDWRITE, motorPushSpeed);
TTMsg MCReset = TTMsg(0x0C1 - MOTORSTATICOFFSET, MCResetFunc);
TTMsg precharge = TTMsg(VOLTAGE - MOTORSTATICOFFSET, prechargeFunc);
TTMsg bms_volt = TTMsg(BMS_VOLTAGE_ID,read_bms_volt_fun);

// load all the ECU specific messages into the TTMessages array
TTMsg TTMessages[]
{
    WriteSpeed,
    MCReset,
    precharge,
    bms_volt
};

void initalizeMsg(TTMsg msg) {
    if (msg.flagFuncs) { // use bool to see if flags are at byte 0 | is this better? idk
        msg.containsFlag = true;
        if (msg.packets[3]) { // if a flags exist and so do all four data slots then this is a problem
            Serial.print("WARNING: FLAG AND MESSAGE CONFLICT! ID:");
            Serial.println(msg.id, HEX);
        } // TODO: find a way to display errors
    }
}

TTMsg offsetMsg(TTMsg msg) { // duplicates message block as the offset block can have seperate values and flags
    TTMsg dup;
    dup.id = msg.id + msg.offset;
    *dup.packets = *msg.packets;
    *dup.flagFuncs = *msg.flagFuncs;
    dup.handle = msg.handle;
    return dup;
}

/* ----- END ECU specific data ----- */

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

    for (auto msg : TTMessages) {
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

    for (TTMsg msg : TTMessages) { // Iterate through defined TTMsgs and push their data
        updateData(msg);
        // writeTTMsg(*msg); // move to updateData
    }
    // if (!carUnlocked && millis() > 10000) // Simulate car Button press
    //     initalizeCar();
}

void updateData(TTMsg msg) {
    if (msg.handle && !(msg.handle)(msg)) { // if the handle exists and returns true upon calling continue execution
        return;
    }
    for (int i = 0; i < 8; i += 2) {
        if (msg.packets[i / 2]) {                     // If we have a sensor for this packet read and store it
            int val = analogRead(msg.packets[i / 2]); // TODO: Some sensors are digital not just analog!
            msg.buf[i] = val / 255;
            msg.buf[i + 1] = val % 255;
        }
    }
}

int decodeliledian(const byte low, const byte high) { // probably will only be used to push to andriod
    long value = 0;
    long full_data = high * 255 + low;
    if(high < 128) // positive
      {
        value = full_data;
      }
      else if(high > 128) //neg
      {
        value = map(full_data,65280,32640,0,-32640);
      }
    return value;                      // Does c++ cast the return type? seems to work?
}

void flagScan(const byte &flag, flagReader funcTbl[8]) {
    if (flag) {                                    // check if flag has any true bits
        for (byte bit = 0; bit < 8; ++bit) {       // iterate though flag bits
            if (funcTbl[bit] && (flag >> bit) & 1) // check that we can do something if the bit is true
                funcTbl[bit]();                    // call function based off bit pos
        }
    }
}

void printMsg(TTMsg const &msg) { // Print out can msg buffer
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

// Push data to andriod using Teensy UART | Eg. Serial1.write();
// TODO: add way to push message blocks to andriod with Serial1.write
// IMPROVE: make andriod decode bytes
void writeTTMsg(const TTMsg msg) { // TODO: can't we just get rid of this?
    // printMsg(msg);
    // Serial.println(millis());
    Can1.write(msg);
}

//IMPROVE: instead of just storing values and pushing later, why not push as they are recieved? Consult leads!
void readTTMsg(TTMsg msg, const byte buf[8]) {
    size_t i = 0;
    if (msg.containsFlag) {              // Readflags if they are expected
        flagScan(buf[i], msg.flagFuncs); // Only checking byte 0
        msg.buf[i] = buf[i];             // Store byte 0 of flags
        msg.buf[i + 1] = buf[i + 1];     // also stores byte 1 for completion sake
        i = 2;                           // Skip flag bytes
    }
    for (i = i; i < 8; i += 2) {
        if (msg.packets[i]) {            // are we expecting data on this packet?
            msg.buf[i] = buf[i];         // don't encode as there is no immediate need
            msg.buf[i + 1] = buf[i + 1]; // no encode
        }
    }
}

// Iterate through defined TTMsgs and check if the address is one of theirs
void teensyRead(const CAN_message_t &dataIn) {
    for (TTMsg msg : TTMessages) {
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
    int accelerator0 = analogRead(accelerator_1);
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
    motorWriteSpeed(msg, MOTOROFFSET, 1, speed1);

    return false; // Don't continue normal TTMsg proccessing
}

void motorWriteSpeed(TTMsg msg, byte offset, bool direction, int speed) { // speed is value 0 - 860
    int percent_speed = constrain(map(speed, 0, 1024, 0, 400), 0, 400); // seprate func for negative vals (regen)
    // Serial.println(percent_speed);
    //Calculations value = (high_byte x 256) + low_byte
    byte low_byte = percent_speed % 256;
    byte high_byte = percent_speed / 256;
    msg.id = SPEEDWRITE + offset - MOTORSTATICOFFSET;
    // Serial.println(msg.id);
    msg.ext = 0;
    msg.len = 8;
    msg.buf[0] = low_byte; // NM
    msg.buf[1] = high_byte;
    msg.buf[2] = 0; // Speed
    msg.buf[3] = 0;
    msg.buf[4] = direction;   // Direction
    msg.buf[5] = carUnlocked; // Inverter enable byte
    msg.buf[6] = 0;           // Last two are the maximum torque values || if 0 then defualt values are set
    msg.buf[7] = 0;
    writeTTMsg(msg);
}

/*
    "Yeet"
        -Bobamba
                    */
