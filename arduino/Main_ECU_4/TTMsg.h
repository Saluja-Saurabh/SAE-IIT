#ifndef TTMSG_H
#define TTMSG_H

#include <IFCT.h>                 // ImprovedFLexCanLibrary
typedef uint32_t uint32;          // clean it up a lil
typedef bool (*msgHandle)(TTMsg); // for message specialization such as a message block with only flags
typedef void (*flagReader)(bool); // functions that are called when flag bits are true

// TODO: decide on addresses for all the sensors and bms
enum CanADR : uint32 {
    // motor controller
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
    MCOFFSET = 0x0FF,
    // IDK
    INFO_ADD = 0x1A5,
    // BMS
    BMS_STATS_ADD = 0x250,
    // motor
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
    steeringAng = 99,
    carMode = true,
    pedalAir = true,

    // Both Teensy's // to keep it consistant duplicate entires will be made for other pins
    boardLed = 13,
    lightsIMDPin = 18,

    /*
            Teensy One      Address: 0x400 - 0x405
    */
    //      button      //
    sig_startButton = 2,
    IMDReadLight = 99,
    AMSReadLight = 99,
    //      range       //
    sig_Wheel2 = 21,
    sig_Wheel1 = 20,
    sig_accel2 = 18,
    sig_accel1 = 16,
    sig_brakePress = 14,
    lightsBMSPin = 18,

    /*
            Teensy Two      Address: 0x406 - 0x40A
    */
    //      send        //
    PWM_servo1 = 8,
    PWM_servo2 = 27, // BROKEN: pin 27 is not PWM?
    //      button      //
    sig_32v = 0,     // ON/OFF // WHAT?
    sig_charger = 5, // ON/OFF
    //      special     //
    sig_precharge = 9,
    sig_prechargeAir = 10,
    sig_brakeLight = 7,
    sig_fans = 24, // ON/OFF
    sig_shutdownState = 25,
    sig_MC = 26, // ON/OFF // WHAT?
    PWM_Fan1 = 23,
    PWM_Fan2 = 22,
    PWM_Fan3 = 21,
    PWM_Fan4 = 20,
    ANL_pump = 21,  // analog only!
    sig_buzzer = 6, //  ON/OFF
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
class TTMsg : public CAN_message_t { // Teensy to Teensy message definition/structure
private:
    validData *memo[4] = {0};
    validData *memoize(validData lookup); // return pointer to element if exists and remember for future if set

public:
    validData *packets;    // data that have data in this message; position in table sets where PKT goes (see ^) // points to table of 4
    uint32 offset = 0;     // now any data can have an offset for duplicates
    flagReader *flagFuncs; // functions that are called when a flag bit is true | limits callbacks to flag byte 0 // points to table of 8
    validData *flagValues; // sensor pins to read and push onto the flag byte | only flag byte 0 // points to table of 8
    msgHandle handle = 0;  // function that can handle the message instead | for specialization of messages
    bool containsFlag = 0; // used for memoization
    int data[4] = {0};     // store decoded or pin data for later use
    TTMsg(uint32 i, uint32 off = 0);
    TTMsg(uint32 i, msgHandle h, uint32 off = 0);
    TTMsg(uint32 i, const validData (&p)[4], uint32 off = 0);
    TTMsg(uint32 i, const validData (&p)[4], const flagReader (&fF)[8], const validData (&fV)[8], uint32 off = 0);
    TTMsg(TTMsg msg, uint32 off);
    TTMsg(uint32 i, validData p[4], flagReader fF[8], validData fV[8], msgHandle h, uint32 off = 0);
    int getDataValue(validData lookup);

}; // IMPROVE: Flags can be extended to handle two bytes if it is really neccessary

#endif

// Example program of the variadic functions
// #include <iostream>
// #include <string>
// using namespace std;

// struct expand_type {
//     template <typename... T>
//     expand_type(T &&...) {}
// };

// void logr(int a){ // force type checking?
//     cout << a;
// }

// template <typename... ArgTypes>
// void print(ArgTypes... args) {
//     expand_type{0, (logr(args),0)...};
// }

// int main()
// {
//   int ya = 4;
//   int yab = 8;
//   int yda = 5;
//   int yga = 98;
//   print(ya, yab, yda, yga);
// }