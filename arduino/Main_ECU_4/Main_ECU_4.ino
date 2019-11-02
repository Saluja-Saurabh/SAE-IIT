#include <IFCT.h> // ImprovedFLexCanLibrary

/*     Pins    */ //TODO: add all pins for individual teensys
// Aero
// Electrical
// Gyro
// Pump

/*     enums    */
enum dataMapType {
    NIL,
    FLAG,
    CHECK,
    LOWBYTE,
    HIGHBYTE,
};
enum addresses : byte { // TODO: layout all the addresses for everything
    // motor
    TEMP1 = 0x0A0,
    TEMP2 = 0x0A1,
    TEMP3 = 0x0A2,
    // ANLIV = 0x0A3,
    // DIGIS = 0x0A4,
    MOTOR_POS = 0x0A5,
    CURRENT = 0x0A6,
    VOLTAGE = 0x0A7,
    FAULT = 0x0AB,
    // Yes
    FOO = 0x0FF,
    BAR = 0x0BB,
};

/*     structs    */
//TODO: make motorPackets work with general dataPackets
struct motorDataPacket {
    byte idOffset = 0;

    int values[8][8]; // not all the tables will be used eg. ANLIV & DIGIS
    bool faults[8][8];

} motor0, motor1;
/*
    dataMap type helps organize the data layout for
    teensy to teesnsy transfer through CANbus
*/
typedef struct dataMap {
    byte address;
    bool map[8];
    // char *decode[4]; // TODO: figure out how decoding what is what will work, T2T does not need decoding only the address
    // byte offset = 0; // implement if packets need ability offset eg. duplicate sensors
} dataMap;

/*     dataMaps    */ // preferably, LOWBYTE come before a HIGHBYTE
dataMap fooMap = {
    FOO,
    {
        FLAG,
        LOWBYTE,
        HIGHBYTE,
        NIL,
        NIL,
        NIL,
        NIL,
        CHECK,
    },
};
dataMap barMap = {
    BAR,
    {
        FLAG,
        LOWBYTE,
        HIGHBYTE,
        NIL,
        LOWBYTE,
        HIGHBYTE,
        NIL,
        CHECK,
    },
};

dataMap *maps[] = {
    fooMap,
    barMap,
}

void
setup() {
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
        if (readMotor(dataIn, motor0)) {
            // Can0.write(dataOut);
        } else if (readMotor(dataIn, motor1)) {
            // Can0.write(dataOut);
        } else {
            for (dataMap *map : maps) {
                if ((*map).address == dataIn.id) {

                    break;
                };
            }
        }
    }
}

void pushCAN(const byte address, const char position, const uint8_t value[]) {
    // CAN value conversion : value = (highByte x 256) + lowByte
    CAN_message_t dataOut; // Can message obj
    dataOut.ext = 0;
    dataOut.id = address;
    dataOut.len = 8;
    uint8_t c = 0;
    for (uint8_t i = 0; i < 8; i += 2) {
        c += i / 2;
        dataOut.buf[i] = value[c] % 256;     // lowByte
        dataOut.buf[i + 1] = value[c] / 256; // highByte
    }
}

//TODO: figure out how data will be pushed to andriod
void pushSerial(const CAN_message_t &packet) {
    // Serial.write();
}

bool readMotor(const CAN_message_t &dataIn, motorDataPacket &packet) {
    uint_fast8_t offst = packet.idOffset; //weird unsigned comparison issues?
    // use map to check if id is within motor id range
    byte pos = map(dataIn.id, TEMP1 + offst, VOLTAGE + offst, 0, 8);
    if (pos <= 7) {
        readMotorPacket(dataIn, packet.values[pos]);
        return true;
    } else if (dataIn.id == FAULT + offst) {
        readFault(dataIn, packet.faults);
    }
    return false;
}

void readMotorPacket(const CAN_message_t &dataIn, int *valueTbl) {
    for (size_t i = 0; i < 4; i++) {
        byte lowByte = i * 2;
        byte highByte = lowByte + 1;
        long full_data = (dataIn.buf[highByte] * 255) + dataIn.buf[lowByte];
        if (dataIn.buf[highByte] < 128) {
            valueTbl[i] = full_data;
        } else if (dataIn.buf[highByte] > 128) {
            valueTbl[i] = map(full_data, 65280, 32640, 0, -32640);
        }
    }
}

void readFault(const CAN_message_t &dataIn, bool faultTbl[8][8]) {
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

void canSniff(const CAN_message_t &msg) {
    Serial.print("MB ");
    Serial.print(msg.mb);
    Serial.print("  LEN: ");
    Serial.print(msg.len);
    Serial.print(" EXT: ");
    Serial.print(msg.flags.extended);
    Serial.print(" REMOTE: ");
    Serial.print(msg.rtr);
    Serial.print(" TS: ");
    Serial.print(msg.timestamp);
    Serial.print(" ID: ");
    Serial.print(msg.id, HEX);
    Serial.print(" Buffer: ");
    for (uint8_t i = 0; i < msg.len; i++) {
        Serial.print(msg.buf[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}
