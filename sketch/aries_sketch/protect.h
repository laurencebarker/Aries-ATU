/////////////////////////////////////////////////////////////////////////
//
// Aries ATU controller sketch by Laurence Barker G8NJJ
// this sketch controls an L-match ATU network
// with a CAT interface to connect to an HPSDR control program
// copyright (c) Laurence Barker G8NJJ 2019
//
// the code is written for an Arduino Nano 33 IoT module
//
// protect.h: amplifier protection code
// this feature is detected by the initialisatino code finding the MCP23017 in thr I2C bus
/////////////////////////////////////////////////////////////////////////
#ifndef __protect_h
#define __protect_h


extern bool GProtectionPresent;            // becomes true of a device detected
extern bool GIsTripped;                    // true if the PA has been tripped
extern byte GTripInputBits;                // input bits with trip state 
                                           // bit0=VSWR; bit1=rev power; bit2=drive power; bit3=temp               



//
// initialise protection
//
void InitProtection(void);


//
// protection trip handler
// triggered by a falling edge on the trip input, signalled by interrupt
//
void TripISR(void);



//
// periodic tick handler
//
void ProtectionTick(void);



//
// function called when display RESET is pressed
//
void TripResetPressed(void);


#endif
