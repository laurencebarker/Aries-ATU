/////////////////////////////////////////////////////////////////////////
//
// Aries ATU controller sketch by Laurence Barker G8NJJ
// this sketch controls an L-match ATU network
// with a CAT interface to connect to an HPSDR control program
// copyright (c) Laurence Barker G8NJJ 2019
//
// the code is written for an Arduino Nano 33 IoT module
//
// CAT handler.h
// this file holds the CAT handling code
/////////////////////////////////////////////////////////////////////////
#ifndef __cathandler_h
#define __cathandler_h
#include <Arduino.h>
#include "tiger.h"


//
// accessible global variables
//
extern volatile bool GPTTPressed;                      // true if PTT pressed (ie TX active). Set by interrupt



//
// initialise CATHandler
//
void InitCATHandler(void);


//
// PTT ISR handler
//
void PttISR(void);


//
// generate output messages for local control events
//
void CATHandleVFOEncoder(signed char Clicks);

void CATHandleEncoder(byte Encoder, char Clicks);

void CATHandlePushbutton(byte Button, bool IsPressed, bool IsLongPressed);


//
// write solution for current antenna and frequency to EEPROM
// this is done before we declare TUNE complete
//
void SetTuneResult(bool Successful, byte Inductance, byte Capacitance, bool IsHighZ);


//
// handlers for received CAT commands
//
void HandleCATCommandNumParam(ECATCommands MatchedCAT, int ParsedParam);
void HandleCATCommandNoParam(ECATCommands MatchedCAT);
void HandleCATCommandBoolParam(ECATCommands MatchedCAT, bool ParsedBool);
void HandleCATCommandStringParam(ECATCommands MatchedCAT, char* ParsedParam);

//
// eeprom/PC interface functions
//

//
// write solution for current antenna and frequency to EEPROM
// this is done before we declare TUNE complete
//
void SetTuneResult(bool Successful, byte Inductance, byte Capacitance, bool IsHighZ);

//
// timer tick
//
void CatHandlerTick();

#endif //not defined
