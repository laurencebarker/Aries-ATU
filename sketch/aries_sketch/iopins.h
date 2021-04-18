/////////////////////////////////////////////////////////////////////////
//
// Aries ATU controller sketch by Laurence Barker G8NJJ
// this sketch controls an L-match ATU network
// with a CAT interface to connect to an HPSDR control program
// copyright (c) Laurence Barker G8NJJ 2019
//
// the code is written for an Arduino Nano 33 IoT module
//
// iopins.h: I/O pin definitions
/////////////////////////////////////////////////////////////////////////

#ifndef __IOPINS_H
#define __IOPINS_H

#include "globalinclude.h"


#define VPINENCODER1A A2              // encoder 1 - inductance
#define VPINENCODER1B A3
#define VPINENCODER1PB 4              // encoder 1 pushbutton

#define VPINTR_PTTOUT 3               // PTT asserted output
#define VPINPTT 10                    // PTT input. 0 = TX.  !will need to set an interrupt!
#define VPINSERIALLOAD 6              // latch serial data output 
#define VPINLED 7                     // status LED (note the normal D13 LED pin used for SPI)

#define VPINHWTUNECMD 9               // hardwire TUNE input, if used

// alternate uses for this pin - #defined as required
#define VPINRELAYLOHIZ 8              // relay for low/high impedance mode: logic 1 selects "low Z" (note moved from pin 12)
#define VPINPROTECTIONTRIP 8          // amplifier protection trip input (hw rev % and above)

#define VPINSTANDALONEJUMPER 12       // jumper to enable standalone mode. Standalone = jumper inserted, input = 0
#define VPINSTANDALONEANTA 2          // antenna input in standalone mode
#define VPINSTANDALONEANTB 5          // antenna input in standalone mode

// analogue inputs are swapped if we use the binocular core ferrite in the VSWR bridge
#ifdef VSWR_SWAPVFVR
#define VPINVSWR_FWD A1               // analogue input
#define VPINVSWR_REV A0               // analogue input
#else
#define VPINVSWR_FWD A0               // analogue input
#define VPINVSWR_REV A1               // analogue input
#endif //VSWR_SWAPVFVR

#endif //not defined
