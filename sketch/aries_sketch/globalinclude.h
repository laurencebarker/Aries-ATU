/////////////////////////////////////////////////////////////////////////
//
// Aries ATU controller sketch by Laurence Barker G8NJJ
// this sketch controls an L-match ATU network
// with a CAT interface to connect to an HPSDR control program
// copyright (c) Laurence Barker G8NJJ 2019
//
// the code is written for an Arduino Nano 33 IoT module
//
// globalinclude.h: global include definitions
/////////////////////////////////////////////////////////////////////////
#ifndef __globalinclude_h
#define __globalinclude_h


//
// hardware and software version: send back to console on request
//
#define SWVERSION 7
#define HWVERSION 1
#define PRODUCTID 2                 // Aries

//
// define this variable if if the VSWR bridge uses the binocular core ferrite
//
#define VSWR_SWAPVFVR 1


//
// define this variable if UI with LCD is to be included
//
//#define CONDITIONAL_LCD_UI 1



//
// define this variable if algorithm debug messages are to be printed
//
//#define CONDITIONAL_ALG_DEBUG 1


//
// define this variable if a simulated VSWR value is to be generated for algorithm testing
//
//#define CONDITIONAL_ALG_SIMVSWR 1



//
// define this variable to stream ADC sample readings to the serial port
//
//#define CONDITIONAL_STREAM_ADCREADINGS 1

//
// define this variable to always enable "quick tune" at the start of the algorithm
//
#define CONDITIONAL_ALWAYS_QUICKTUNE 1



#endif      // file sentry
