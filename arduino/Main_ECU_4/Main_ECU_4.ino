#include <IFCT.h> //ImprovedFLexCanLibrary

CAN_message_t dataIn, dataOut; //Can message obj

/*     Pins    */
//Aero
//Electrical
//Gyro
//Pump

/*     Adresses    */
enum motorID : byte { TEMP1 = 0x0A0,
                      TEMP2 = 0x0A1,
                      TEMP3 = 0x0A2,
                      // ANLIV = 0x0A3,
                      // DIGIS = 0x0A4,
                      MOTOR_POS = 0x0A5,
                      CURRENT = 0x0A6,
                      VOLTAGE = 0x0A7,
                      FAULTS = 0x0AB,
};

struct motorDataPacket {
    byte idOffset = 0;

    int values[8][8];

    //volt
    // 0 = voltageDC
    // 1 = voltageOutput
    // 2 = voltageAB
    // 3 = voltageBC

    //current
    // 4 = currentPA
    // 5 = currentPB
    // 6 = currentPC
    // 7 = currentDC
    //temp
    // 8 = tempPhaseA
    // 9 = tempPhaseB
    // 10 = tempPhaseC
    // 11 = tempDriverBoard
    //tempC
    // 12 = tempMotor
    // 13 = tempControlBoard

    // 14 = sensorAngle
    // 15 = angularVelocity
    // 16 = electricalFrequency

    bool faults[8][8];

} motor0, motor1;

void setup() {
    pinMode(2, OUTPUT);        // Fusion Tech's Dual CAN-Bus R pin switch
    Can0.setBaudRate(1000000); //Speeed
    Can0.enableFIFO();         //FirstInFirstOut
    // Interrupt capability
    // Can0.enableFIFOInterrupt();
    // Can0.onReceive(canSniff);
    // Can0.intervalTimer(); // enable queue system and run callback in background
}

void loop() {
    if (Can0.read(dataIn)) {
        //check if within motor id range
        if (byte pos = constrain(map(dataIn.id, 0x0A0, 0x0A7, 0, 7), 0, 8); pos <= 7) {
            checkMotorPacket(motor0.values[pos]);
            checkMotorPacket(motor1.values[pos]);
        }
    }
}

int checkMotorPacket(motorDataPacket &packet) // Position (0,1,2,3) inside data structure of data
{
    // byte low_byte = position_1 * 2;
    // byte high_byte = low_byte + 1;
    // int value = pre_data; // Value set so the number never changes when ID is not true
    // if (RX_msg.id == ID) // ID of motor array
    // {

    for (int i = 0; i < 17; i++) {
        packet.values[i] =
    }

    long full_data = (RX_msg.buf[high_byte] * 255) + RX_msg.buf[low_byte];
    if (RX_msg.buf[high_byte] < 128) {
        value = full_data;
    } else if (RX_msg.buf[high_byte] > 128) {
        value = map(full_data, 65280, 32640, 0, -32640);
    }
    // }

    return value;
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
