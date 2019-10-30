/////////////////////////////////////////////////////////////////////////
//
// Aries ATU controller sketch by Laurence Barker G8NJJ
// this sketch controls an L-match ATU network
// with a CAT interface to connect to an HPSDR control program
// copyright (c) Laurence Barker G8NJJ 2019
//
// the code is written for an Arduino Nano Every module
//
// iopins.h: I/O pin definitions
/////////////////////////////////////////////////////////////////////////

#ifndef __IOPINS_H
#define __IOPINS_H


#define VPINENCODER1A 2           // encoder 1 - inductance
#define VPINENCODER1B 3

#define VPINENCODER2A 4          // encoder 2 - capacitance
#define VPINENCODER2B 6

#define VPINENCODER1PB 7          // encoder 1 pushbutton
#define VPINPUSHBUTTONLOHIZ 8     // high/low Z pushbutton
#define VPINRELAYLOHIZ 9          // relay for low/high impedance mode: logic 1 selects "low Z"
#define VPINPTT 10                // PTT input. 0 = TX.  !will need to set an interrupt!

#define VPINVSWR_FWD A0           // analogue input
#define VPINVSWR_REV A1           // analogue input

#endif //not defined
