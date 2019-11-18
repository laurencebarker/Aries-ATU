/////////////////////////////////////////////////////////////////////////
//
// Aries ATU controller sketch by Laurence Barker G8NJJ
// this sketch controls an L-match ATU network
// with a CAT interface to connect to an HPSDR control program
// copyright (c) Laurence Barker G8NJJ 2019
//
// the code is written for an Arduino Nano Every module
//
// LCD_UI.h: header for temporary LCD user interface
/////////////////////////////////////////////////////////////////////////
#ifndef __lcd_ui_h
#define __lcd_ui_h


//
// initialise the UI and its sub-devices
//
void LCD_UI_Initialise(void);


//
// periodic timer tick
//
void LCD_UI_Tick(void);

//
// set whether tuning or not
// true if tune is in progress
//
void LCU_UI_SetTuning(bool);


//
// set forward power and VSWR
// VSWR is passed as 10*"real" VSWR, to allow insertion of 1 decimal place
//
void SetPwr(int Power);
void SetVSWR(int VSWR);


#endif
