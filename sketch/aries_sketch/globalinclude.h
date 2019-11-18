/////////////////////////////////////////////////////////////////////////
//
// Aries ATU controller sketch by Laurence Barker G8NJJ
// this sketch controls an L-match ATU network
// with a CAT interface to connect to an HPSDR control program
// copyright (c) Laurence Barker G8NJJ 2019
//
// the code is written for an Arduino Nano Every module
//
// globalinclude.h: global include definitions
/////////////////////////////////////////////////////////////////////////
#ifndef __globalinclude_h
#define __globalinclude_h


//
// hardware and software version: send back to console on request
//
#define SWVERSION 1
#define HWVERSION 1
#define PRODUCTID 2                 // Aries

//
// define this variable if UI with LCD is to be included
//
#define CONDITIONAL_LCD_UI 1

//
// define this variable if algorithm debug messages are to be printed
//
#define CONDITIONAL_ALG_DEBUG 1
//
// define this variable if a simulated VSWR value is to be generated for algorithm testing
//
//#define CONDITIONAL_ALG_SIMVSWR 1


#endif      // file sentry
