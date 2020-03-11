#ifndef SAECAN_H
#define SAECAN_H

#include "TTMsg.h" // ImprovedFLexCanLibrary

void beginCAN(uint32_t baudRate = 500000);
void CANRun();

#endif