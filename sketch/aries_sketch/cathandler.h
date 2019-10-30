/////////////////////////////////////////////////////////////////////////
//
// Andromeda front panel controller by Laurence Barker G8NJJ
// this sketch provides a knob and switch interface through USB and CAT
// copyright (c) Laurence Barker G8NJJ 2019
//
// CAT handler.h
// this file holds the CAT handling code
/////////////////////////////////////////////////////////////////////////
#ifndef __cathandler_h
#define __cathandler_h
#include <Arduino.h>
#include "tiger.h"



//
// generate output messages for local control events
//
void CATHandleVFOEncoder(signed char Clicks);

void CATHandleEncoder(byte Encoder, char Clicks);

void CATHandlePushbutton(byte Button, bool IsPressed, bool IsLongPressed);




//
// handlers for received CAT commands
//
void HandleCATCommandNumParam(ECATCommands MatchedCAT, int ParsedParam);
void HandleCATCommandNoParam(ECATCommands MatchedCAT);
void HandleCATCommandBoolParam(ECATCommands MatchedCAT, bool ParsedBool);
void HandleCATCommandStringParam(ECATCommands MatchedCAT, char* ParsedParam);


#endif //not defined
