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
// this type enumerates the Nextion display pages:
//
enum EDisplayPage
{
  eSplashPage,                              // startup splash page
  eCrossedNeedlePage,                       // crossed needle VSWR page display
  ePowerBargraphPage,                       // linear watts bargraph page display
  eMeterPage,                               // analogue power meter
  eEngineeringPage,                         // engineering page with raw ADC values
  eSetupPage                                // setup page
};



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
// force LCD update
//
void SetTuneChanged();

//
// local version of "sprintf like" function
// Adds a decimal point before last digit if 3rd parameter set
// note integer value is signed and may be negative!
//
unsigned char mysprintf(char *dest, int Value, bool AddDP);


// debug
void ShowFrequency(char* FreqString);
void ShowTune();
void ShowAntenna(int Antenna);
void ShowATUEnabled(bool IsEnabled);


#endif
