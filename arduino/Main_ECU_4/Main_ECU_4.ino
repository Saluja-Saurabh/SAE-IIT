#include <IFCT.h>                 // ImprovedFLexCanLibrary
typedef uint_fast8_t uint8;       // Ensures no mixing between unsigned and signed bytes
typedef void (*flagReader)(void); // functions that are called when flags are true
typedef void (*pktIntrpr)(uint8); // functions that are called for msg bytes

/*     Pins    */ //TODO: add pins for individual teensys
// Aero
// Electrical
// Gyro
// Pump

typedef enum pktType : uint8 {
    NIL,
    FLAG,
    LOWBYTE,
    HIGHBYTE,
};
enum CanADR : uint8 { // TODO: layout all the addresses for everything passed by CAN
    // motor
    TEMP1 = 0x0A0,
    TEMP2 = 0x0A1,
    TEMP3 = 0x0A2,
    ANLIV = 0x0A3,
    DIGIS = 0x0A4,
    MOTOR_POS = 0x0A5,
    CURRENT = 0x0A6,
    VOLTAGE = 0x0A7,
    FAULT = 0x0AB,
    // Yes
    FOO = 0x0FF,
    BAR = 0x0BB,
};

//TODO: make motorPackets work with TTPkt
struct motorDataPkt {
    uint8 idOffset = 0;
    long values[8][8]; // not all the tables will be used eg. ANLIV & DIGIS
    bool faults[8][8];
} motor0, motor1;

/*
    Byte # (map)| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | // byte index in CAN message
    H&L 'packet'        | H | L |   =   |  PKT  | // LOWBYTE and HIGHBYTE must be next to each other, referred as packets(PKT)
    PKT pos     |  PKT  |  PKT  |  PKT  |  PKT  | // Valid PKT placement NOTE: must be set as seprate H & L in map array
    FLAG pos    | F | F | F | F | F | F | F | F | // Valid FLAG placement NOTE: currently only one flag is per PKT supported
    pktIntrpr   | I |   | I |   | I |   | I |   | // Where PKT interpreter functions are called
    flagReader  | R | R | R | R | R | R | R | R | // Where FLAG reader functions are called
*/
typedef struct TTPkt { // Teensy to Teensy packet definition
    uint8 address;     // identifies how the msg should be interpreted
    pktType map[8];
    //TODO: see if sensors need specific interpreters, else , if its just a basic analog values then give all TTPkt same func
    pktIntrpr intrprs[4];     // max 4 pkt interpreters per packet
    flagReader *flagFuncs[8]; //limits each packet to one flag byte
    int values[8];            // not all the tables will be used eg. ANLIV & DIGIS
} TTPkt;

TTPkt fooSensor{
    FOO,
    {

    },
    {

    },
};

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
    pinMode(2, OUTPUT); // Fusion Tech's Dual CAN-Bus R pin switch
    digitalWrite(2, LOW);
    Can0.setBaudRate(1000000); // Speeed
    Can0.enableFIFO();         // FirstInFirstOut
}

void loop() {
    CAN_message_t dataIn; // Can message obj
    if (Can0.read(dataIn)) {
        if (motorRead(dataIn, motor0)) {
            // Can0.write(dataOut);
        } else if (motorRead(dataIn, motor1)) {
            // Can0.write(dataOut);
        } else {
            // for (TTPkt &pkt : maps) {
            //     if (pkt.address == dataIn.id) {
            //         readPacket(pkt);
            //         break;
            //     };
            // }
        }
    }
}

void writeCAN(const byte &address, const uint8 &position, const uint8 value[8]) {
    // CAN value conversion : value = (highByte x 256) + lowByte
    CAN_message_t dataOut; // Can message obj
    dataOut.ext = 0;
    dataOut.id = address;
    dataOut.len = 8;
    uint8 c = 0;
    for (uint8 i = 0; i < 8; i += 2) {
        c += i / 2;
        dataOut.buf[i] = value[c] % 256;     // lowByte
        dataOut.buf[i + 1] = value[c] / 256; // highByte
    }
}

void flagScan(const uint8 &flag, flagReader funcTbl[8]) {
    if (flag) {                                             // check if flag has any value
        for (uint8 bit = 0; bit < 8; ++bit) {               // iterate though flag bits
            if (funcTbl[bit] && (flag >> bit) & 0B00000001) // check that we can do somthing if the bit is true
                funcTbl[bit]();                             // call function based off bit pos
        }
    }
}

//TODO: figure out how data will be pushed to andriod

// motor functions
bool motorRead(const CAN_message_t &dataIn, motorDataPkt &packet) {
    uint8 offst = packet.idOffset;
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
    for (size_t i = 0; i < 4; i++) {
        uint8 lowByte = i * 2;
        uint8 highByte = lowByte + 1;
        int full_data = (dataIn.buf[highByte] * 255) + dataIn.buf[lowByte];
        if (dataIn.buf[highByte] < 128) {
            valueTbl[i] = full_data;
        } else if (dataIn.buf[highByte] > 128) {
            valueTbl[i] = map(full_data, 65280, 32640, 0, -32640);
        }
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