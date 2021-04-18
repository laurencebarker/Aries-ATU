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
// this is an optional feature, enabled by #define AMPLIFIERPROTECTION 1 in globalinclude.h
/////////////////////////////////////////////////////////////////////////


#include "protect.h"



//
// protection trip handler
// triggered by a falling edge on the trip input, signalled by interrupt
//
void TripISR(void)
{
  
}


//
// periodic tick handler
//
void ProtectionTick(void)
{
  
}
