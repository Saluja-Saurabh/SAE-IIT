#include <IFCT.h> //ImprovedFLexCanLibrary

CAN_message_t dataIn, dataOut; //Can message obj

/*     Pins    */
//Aero
//Electrical
//Gyro
//Pump

/*     enums    */
enum motor_ID : byte {
    TEMP1 = 0x0A0,
    TEMP2 = 0x0A1,
    TEMP3 = 0x0A2,
    // ANLIV = 0x0A3,
    // DIGIS = 0x0A4,
    MOTOR_POS = 0x0A5,
    CURRENT = 0x0A6,
    VOLTAGE = 0x0A7,
    FAULT = 0x0AB,
};
byte *motor_msgDef[6];
enum motor_msgEnums : byte {
    //motor_volt
    DCBus = 0,
    Output = 1,
    VAB_Vd = 2,
    VBC_Vq = 3,
    //motor_current
    PhaseA = 0,
    PhaseB = 1,
    PhaseC = 2,
    DCBus = 3,
    //motor_temp1
    PhaseA = 0,
    PhaseB = 1,
    PhaseC = 2,
    DriverBoard = 3,
    //motor_temp2
    ControlBoard = 0,
    //motor_temp3
    motor = 2,
    torqueShudder = 3,
    //motor_pos
    angle = 0,
    anglrVel = 1,
    electricalFreq = 2,
};
// enum motor_volt : byte {
//     DCBus,
//     Output,
//     VAB_Vd,
//     VBC_Vq,
// };
// enum motor_current : byte {
//     PhaseA,
//     PhaseB,
//     PhaseC,
//     DCBus,
// };
// enum motor_temp1 : byte {
//     PhaseA,
//     PhaseB,
//     PhaseC,
//     DriverBoard,
// };
// enum motor_temp2 : byte {
//     ControlBoard
// };
// enum motor_temp3 : byte {
//     motor = 2,
//     torqueShudder = 3,
// };
// enum motor_pos : byte {
//     angle,
//     anglrVel,
//     electricalFreq,
// };
/*     motorFaults    */
char *faults_decoder[8][8] =
    {
        {"Hardware Gate/Desaturation Fault",
         "HW over-current Fault",
         "Accelerator shorted",
         "Accelerator Open",
         "Current sensor Low",
         "Current sensor High",
         "Module Tempature Low",
         "Module Tempature High"},
        {"Control PCB Temperature Low",
         "Control PCB Temperature High",
         "Gate Dive PCB Temperature Low",
         "Gate Dive PCB Temperature High",
         "5V Sense Voltage Low",
         "5V Sense Voltage High",
         "12V Sense Voltage Low",
         "12V Sense Voltage High"},
        {"2.5V Sense Voltage Low",
         "2.5V Sense Voltage High",
         "1.5V Sense Voltage Low",
         "2.5V Sense Voltage High",
         "DC Bus Voltage High",
         "DC Bus Voltage Low",
         "Precharge Timeout",
         "Precharge Voltage Failure"},
        {"EEPROM Checksum Invalid",
         "EEPROM Data Out of Range",
         "EEPROM Update Required",
         "Reserved",
         "Reserved",
         "Reserved",
         "Brake Shorted",
         "Brake Open"},
        {"Motor Over-speed Fault",
         "Over-current Fault",
         "Over-voltage Fault",
         "Inverter Over-temperature Fault",
         "Accelerator Input Shorted Fault",
         "Accelerator Input Open Fault",
         "Direction Command Fault",
         "Inverter Response Time-out Fault"},
        {"Hardware Gate/Desaturation Fault_2",
         "Hardware Over-current Fault_2",
         "Under-voltage Fault",
         "CAN Command Message Lost Fault",
         "Motor Over-temperature Fault",
         "Reserved",
         "Reserved",
         "Reserved"},
        {"Brake Input Shorted Fault",
         "Brake Input Open Fault",
         "Module A Over-temperature Fault7",
         "Module B Over-temperature Fault7",
         "Module C Over-temperature Fault7",
         "PCB Over-temperature Fault7",
         "Gate Drive Board 1 Over-temperature Fault",
         "Gate Drive Board 2 Over-temperature Fault7"},
        {"Gate Drive Board 3 Over-temperature Fault7",
         "Current Sensor Fault",
         "Reserved",
         "Reserved",
         "Reserved",
         "Reserved",
         "Resolver Not Connected",
         "Inverter Discharge Active"}};

struct motorDataPacket {
    byte idOffset = 0;

    int values[8][8]; //not all the tables will be used eg. ANLIV & DIGIS
    bool faults[8][8];

} motor0, motor1;

void setup() {
    //load all the zeros for the motors
    for (int j = 0; j < 8; ++j) {
        for (int k = 0; k < 8; ++k) {
            motor0.faults[j][k] = 0;
            motor1.faults[j][k] = 0;
            motor0.values[j][k] = 0;
            motor1.values[j][k] = 0;
        }
    }
    for (size_t i = 0; i < 6; i++) {
        motor_msgDef[i] = new byte[4];
    }

    pinMode(2, OUTPUT);        // Fusion Tech's Dual CAN-Bus R pin switch
    Can0.setBaudRate(1000000); //Speeed
    Can0.enableFIFO();         //FirstInFirstOut
}

void loop() {
    if (Can0.read(dataIn)) {
        //check if id is within motor info id range
        if (byte pos = map(dataIn.id, TEMP1, VOLTAGE, 0, 8); pos <= 7) { //TODO: ensure ids are always pos
            checkMotorPacket(motor0.values[pos], motor_msgDef[pos]);
            checkMotorPacket(motor1.values[pos], motor_msgDef[pos]);
        } else if (dataIn.id == FAULT) {
            checkFault(motor0.faults);
            checkFault(motor1.faults);
        }
    }
}

void checkMotorPacket(int valueTbl[], byte msgDef[]) {
    // byte low_byte = position_1 * 2;
    // byte high_byte = low_byte + 1;
    // int value = pre_data; // Value set so the number never changes when ID is not true
    // if (RX_msg.id == ID) // ID of motor array
    // {

    for (int i = 0; i < 4; i++) {
        valueTbl[i] = 5;
    }

    // long full_data = (RX_msg.buf[high_byte] * 255) + RX_msg.buf[low_byte];
    // if (RX_msg.buf[high_byte] < 128) {
    //     value = full_data;
    // } else if (RX_msg.buf[high_byte] > 128) {
    //     value = map(full_data, 65280, 32640, 0, -32640);
    // }
    // }
}

void checkFault(bool faultTbl[8][8]) {
    for (int col = 0; col < 8; ++col) // for each byte
    {
        if (dataIn.buf[col]) // If the byte has info
        {
            for (int row = 0; row < 8; ++row) // for each bit
            {
                if (((dataIn.buf[col] >> row) & 0B00000001)) // If each bit is true, store value
                {
                    faultTbl[col][row] = 1;
                } else // No fault
                {
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
