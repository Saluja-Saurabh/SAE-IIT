#ifndef TTMSG_H
#define TTMSG_H
#define MAXVALIDDATA 81 // highest value of a validData type
// WARNING!!! Due to incredible implementation skills, the size of the array must be manually set; this includes the "mirror" messages for read/write :/
// this can be fixed by making this limit huge and checking for null buuut naah
#define MSGREADS 8  // max # of messages to be read
#define MSGWRITES 8 // max # of messages to be written

#include "Funcs.h"
#include <IFCT.h>                   // ImprovedFLexCanLibrary
typedef bool (*msgHandle)(TTMsg *); // for message specialization such as a message block with only flags
typedef void (*flagReader)(bool);   // functions that are called when flag bits are true

// TODO: decide on addresses for all the sensors and bms
enum CanADR : uint32_t {
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

// validData teensy pin outs are from 0-33
// validData above 33 are arbitrary; used only for id
enum validData : uint8_t { // Used to identify what data goes into what message
    NIL = -1,              // Will this work?; will overflow but is known value

    // Motors
    MotorLTemp = 40,
    MotorRTemp = 41,
    rpmLWheel = 42, // Is this from MCs or motors themselves?
    rpmRWheel = 43,

    // Motor controller
    // TEMP1
    PhaseATemp = 44,
    PhaseBTemp = 45,
    PhaseCTemp = 46,
    DriverBoard = 47,
    // TEMP2
    ControlBoard = 48,
    RTD1 = 49,
    RTD2 = 50,
    RTD3 = 51,
    // TEMP3
    RTD4 = 52,
    RTD5 = 53,
    motor = 54,
    torqueShudder = 55,
    //MOTORPOS
    angle = 56,
    anglrVel = 57,
    electricalFreq = 58,
    deltaResolver = 59, // Unused
    //CURRENT
    PhaseACur = 60,
    PhaseBCur = 61,
    PhaseCCur = 70,
    DCBusCur = 71,
    //VOLTAGE
    DCBusV = 72,
    OutputV = 73,
    VAB_Vd = 74,
    VBC_Vq = 75,
    //BMS
    BMSVolt = 76,
    BMSTemp = 77,
    BMSSOC = 78,

    // accelerator
    avgAccel = 79, // Manually set

    //T2T //TODO: Do these valid data belong here?
    carMode = 79,
    pedalAir = 80,
    steeringAng = 81,

    // Both Teensy's // to keep it consistant duplicate entires will be made for other pins
    boardLed = 13,
    lights_IMD = 18,

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
    FLAG pos    | F1| F0|                         // Flag bytes if they exist NOTE: only byte 7 will call functions
    FLAG bits   |--F1--|0|1|3|4|5|6|7|--F0--|0|1|2|3|4|5|6|7| // how the flags are stored
*/

// TODO: actually initalize TTMsg and CAN_message_t values in constructor instead of depending on just the default values!
struct TTMsg : public CAN_message_t { // Teensy to Teensy message definition/structure
    validData packets[4] = {NIL};     // data that have data in this message; position in table sets where PKT goes (see ^) // points to table of 4
    bool isOffset = 0;                // now any data can have an offset for one "mirror" message
    flagReader flagFuncs[8] = {0};    // functions that are called when a flag bit is true | limits callbacks to flag byte 0 // points to table of 8
    validData flagValues[8] = {NIL};  // sensor pins to read and push onto the flag byte | only flag byte 0 // points to table of 8
    msgHandle handle = 0;             // function that can handle the message instead | for specialization of messages
    bool containsFlag = 0;            // used for memo
    int16_t data[4] = {0};            // store decoded or pin data for later use | NOTE: this value is synced with actual bytes that are read/written
};                                    // IMPROVE: Flags can be extended to handle two bytes if it is really neccessary

#endif