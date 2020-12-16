/////////////////////////////////////////////////////////////////////////
//
// Aries ATU controller sketch by Laurence Barker G8NJJ
// this sketch controls an L-match ATU network
// with a CAT interface to connect to an HPSDR control program
// copyright (c) Laurence Barker G8NJJ 2019
//
// the code is written for an Arduino Nano 33 IoT module
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
// periodic timer tick for encoders
// (this could be called more often than main tick)
//
void LCD_UI_EncoderTick(bool OddEncoder);

//
// set whether tuning or not
// true if tune is in progress
//
void LCD_UI_SetTuning(bool);

//
// force LCD update
//
void SetTuneChanged();

//
// set forward power and VSWR
// VSWR is passed as 10*"real" VSWR, to allow insertion of 1 decimal place
//
void SetPwr(int Power);
void SetVSWR(int VSWR);

// debug
void ShowFrequency(char* FreqString);
void ShowTune(bool IsTune);
void ShowAntenna(int Antenna);
void ShowATUEnabled(bool IsEnabled);

#endif
