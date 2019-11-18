/////////////////////////////////////////////////////////////////////////
//
// Aries ATU controller sketch by Laurence Barker G8NJJ
// this sketch controls an L-match ATU network
// with a CAT interface to connect to an HPSDR control program
// copyright (c) Laurence Barker G8NJJ 2019
//
// the code is written for an Arduino Nano Every module
//
// hwdriver.h:  drives to mechanical relays
/////////////////////////////////////////////////////////////////////////
#ifndef __hwdriver_h
#define __hwdriver_h

#include <arduino.h>



//
// function to initialise output
//
void InitialiseHardwareDrivers(void);


//
// Hardware driver tick
// read the aDC values
//
void HWDriverTick(void);


//
// functions to get and set inductance and capacitance
//
void SetInductance(byte Value);             // inductance 0-255
void SetCapacitance(byte Value);            // capacitance 0-255
void SetHiLoZ(bool Value);                  // true for low Z (relay=1)
byte GetInductance(void);                   // inductance 0-255
byte GetCapacitance(void);                  // capacitance 0-255
bool GetHiLoZ(void);                        // true for low Z (relay=1)


//
// assert tuning solution to relays
// send settings to I2C
//
void DriveSolution(void);

//
// functions to get the current inductance and capacitance, in nH and pF
//
int GetCapacitanceValue(void);
int GetInductanceValue(void);


// 
// function to return VSWR value
// returns 100*VSWR; clipped to 65535
//
int GetVSWR(void);

#endif
