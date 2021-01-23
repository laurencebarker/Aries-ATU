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
extern bool GPCTuneActive;                             // true if TUNE is in progress as signalled by PC (note PTT will be detected first)
extern bool GATUEnabled;                               // true if the ATU is enabled
extern bool GQuickTuneEnabled;                         // true if quick tune allowed
extern bool GValidSolution;                            // true if a valid tune solution found
extern unsigned int GTunedFrequency10;                 // frequency from THETIS, in 10KHz resolution. 0 = DC
extern byte GTXAntenna;                                // selected TX antenna (1-3; 0 if not set)



//
// initialise CATHandler
//
void InitCATHandler(void);


//
// PTT ISR handler
//
void PttISR(void);

//
// tune hardwired input ISR handler
// this triggers on falling edge, to trigger a "tune request"
// we assume it to come from an FPGA source, so no bounce
//
void HWTuneISR(void);


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

// EEEraseSolutionSet: erase an entire set of solutions for one antenna from EEPROM
// this erases a significant block of EEPROM memory, and takes around 3 seconds.
//
void EEEraseSolutionSet(byte Antenna);

//
// function to write, read new LCD display page
//
void EEWritePage(byte Value);
byte EEReadPage();

//
// function to write, read  new display average/peak mode
//
void EEWritePeak(bool Value);
bool EEReadPeak();

//
// function to write, read new ATU enabled/disabled for standalone mode
//
void EEWriteEnabled(bool Value);
bool EEReadEnabled();

//
// function to write, read new ATU quick tune enabled/disabled for standalone mode
//
void EEWriteQuick(bool Value);
bool EEReadQuick();

//
// function to write, read new ATU display scale for standalone mode
//
void EEWriteScale(byte Value);
byte EEReadScale();
//
// write solution for current antenna and frequency to EEPROM
// this is done before we declare TUNE complete
//
void SetTuneResult(bool Successful, byte Inductance, byte Capacitance, bool IsHighZ);

//
// timer tick
//
void CatHandlerTick();

//
// handle ATU on/off CAT message from PC
// or standalone mode: from display click
//
void SetATUOnOff(bool State);


#endif //not defined
