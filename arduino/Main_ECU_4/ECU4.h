#ifndef ECU4_H
#define ECU4_H
#define TEENSYFRONT
#define TEENSYBACK

#include "CANMSG.h"
#include "MsgMaster.h"

#ifdef TEENSYFRONT
// Globals
bool PEDAL_ERROR = false;
bool CAR_MODE = false; // true: RaceMode false: EcoMode
bool FAULT = false;    // ams, bms mc faults and DOPRECHARGE == 1 from precharge circuit
const byte Teensy2SerialArrSize = 12;
int Teensy2SerialArr[Teensy2SerialArrSize];
int T2AMsg[11];

// flag handlers
void setPedalState(bool bit);
void setCarMode(bool bit);
// functions
bool pruneFaults();
void pushT2A();
void chargerSet();
void setPump(int voltage = 0);
void brakeLights();
void accelCheck();

#endif

#ifdef TEENSYBACK

// Globals
bool DO_PRECHARGE = true;         // Precharge latching variable
bool START_BUTTON_READY = false;  // when to buzz
bool START_BUTTON_PUSHED = false; // MC enable bit state
uint32_t MOTOR_OFFSET = 0xe0;     // offset for motor ids // is this actually just for the MCs?
// flag handlers
void initalizeCar(bool bit);
// packet handlers
bool motorPushSpeed(TTMsg &msg);
bool prechargeFunc(TTMsg &msg);
// functions
bool MCResetFunc(TTMsg &msg);
void motorWriteSpeed(TTMsg &msg, byte offset, bool direction, int speed);

#endif

void setMasterMessages();

class ECU4 {
public:
    void run();
    void begin();
};

extern ECU4 ECU;

#endif