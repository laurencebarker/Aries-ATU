/////////////////////////////////////////////////////////////////////////
//
// Aries ATU controller sketch by Laurence Barker G8NJJ
// this sketch controls an L-match ATU network
// with a CAT interface to connect to an HPSDR control program
// copyright (c) Laurence Barker G8NJJ 2019
//
// the code is written for an Arduino Nano 33 IoT module
//
// LCD_UI.cpp: temporary LCD user interface
/////////////////////////////////////////////////////////////////////////

#include "lcd_ui.h"
#include "mechencoder.h"
#include "pushbutton.h"
#include "iopins.h"
#include "hwdriver.h"
#include "algorithm.h"
#include "globalinclude.h"
#include "cathandler.h"
#include <Arduino.h>
#include <Nextion.h>

#ifdef CONDITIONAL_LCD_UI

//
// global variables
//

#define VCOARSESTEP 8                               // L/C steps if coarse setpping
#define VENCODERDIVISOR 2                           // events per mechanical click

bool IsCoarse;
bool GTuneChanged;              // true if a tune operation has happened and display to be updated
int GForwardPower;              // VSWR bridge forward power
int GIntVSWR;                   // VSWR bridge calculated VSWR (1 DP fixed point, so 10 X value needed)
byte GVoltDisplayCount;

//
// display updates, for debug
//
unsigned int GDisplayItem;      // item to display at next tick
byte GDisplayedL;               // value of L posted to display, to check for updates
byte GDisplayedC;               // value of C posted to display, to check for updates
int GDisplayedVSWR;             // VSWR value displayed
int GDisplayedPower;            // power value displayed
int GDisplayedVf;               // forward voltage displayed
int GDisplayedVr;               // reverse voltage displayed
bool GDisplayedPTT;             // PTT value displayed
bool GDisplayedTune;            // tune state displayed
bool GDisplayedZ;               // true if High Z
int GDisplayedFreq;             // frequency displayed



NoClickEncoder Encoder1(VPINENCODER1B, VPINENCODER1A, VENCODERDIVISOR, true);
NoClickEncoder Encoder2(VPINENCODER2B, VPINENCODER2A, VENCODERDIVISOR, true);
PushbuttonDebounce EncoderBtnDebounce(VPINENCODER1PB, 64);          // toggle coarse/fine; longpress to toggle low/high Z
PushbuttonDebounce TuneBtnDebounce(VPINPUSHBUTTONTUNE);             // initiates tune; no longpress



//
// Nextion display object declarations
//

//
// declare pages:
//
NexPage page0 = NexPage(1, 0, "page0");       // creates touch event for the debug page

//
// page 0 objects:
// 
NexText p0LValue = NexText(0, 2, "p0t3");                   // inductance value
NexText p0CValue = NexText(0, 3, "p0t4");                   // capacitance value
NexButton p0LMinusBtn = NexButton(0, 6, "p0b0");            // L- pushbutton
NexButton p0LPlusBtn = NexButton(0, 7, "p0b1");             // L+ pushbutton
NexButton p0CMinusBtn = NexButton(0, 8, "p0b2");            // C- pushbutton
NexButton p0CPlusBtn = NexButton(0, 9, "p0b3");             // C+ pushbutton
NexDSButton p0FineBtn = NexDSButton(0, 17, "p0bt0");        // coarse/fine button
NexDSButton p0HighZBtn = NexDSButton(0, 18, "p0bt1");       // High/LowZ button button
NexText p0VSWRValue = NexText(0, 20, "p0t13");              // VSWR value
NexText p0PowerValue = NexText(0, 22, "p0t15");             // RF Power value
NexText p0FreqValue = NexText(0, 16, "p0t11");             // Frequency value
NexText p0VfValue = NexText(0, 26, "p0t19");                // Vf value
NexText p0VrValue = NexText(0, 27, "p0t20");                // Vr value
NexText p0AntValue = NexText(0, 14, "p0t9");                // Antenna number
NexText p0Enabled = NexText(0, 11, "p0t6");                 // Enabled/disabled
NexText p0PTT = NexText(0, 12, "p0t7");                     // PTT on/off
NexText p0Tune = NexText(0, 13, "p0t8");                    // Tune on/off


//
// declare touch event objects to the touch event list
// this tells the code what touch events too look for
//
NexTouch *nex_listen_list[] = 
{
  &page0,                                     // page change 
  &p0LMinusBtn,                               // L- button press
  &p0LPlusBtn,                                // L+ button press
  &p0CMinusBtn,                               // C- button press
  &p0CPlusBtn,                                // C+ button press
  &p0FineBtn,                                 // coarse/fine button press
  &p0HighZBtn,                                // Low/High Z button
  NULL                                        // terminates the list
};

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// touch event handlers: PAGE change
//

//
// page 0 - splash page callback
//
void page0PushCallback(void *ptr)             // called when page 0 loads (splash page)
{
}

//
// touch event - L-
//
void p0LMinusPushCallback(void *ptr)          // reduce inductance by one step
{
  int Inductance;
  int Increment = -1; 
  if (IsCoarse)
    Increment *= VCOARSESTEP;

  Inductance = GetInductance();
  Inductance = constrain(Inductance + Increment, 0, 255);
  GTuneChanged = true;        // force redraw
  SetInductance(Inductance);
  DriveSolution();
}

//
// touch event - L+
//
void p0LPlusPushCallback(void *ptr)           // increase inductance by one step
{
  int Inductance;
  int Increment = 1; 
  if (IsCoarse)
    Increment *= VCOARSESTEP;
    
  Inductance = GetInductance();
  Inductance = constrain(Inductance + Increment, 0, 255);
  GTuneChanged = true;        // force redraw
  SetInductance(Inductance);
  DriveSolution();
}

//
// touch event - C-
//
void p0CMinusPushCallback(void *ptr)          // reduce capacitance by one step
{
  int Capacitance;
  int Increment = -1; 
  if (IsCoarse)
    Increment *= VCOARSESTEP;
    
  Capacitance = GetCapacitance();
  Capacitance = constrain(Capacitance + Increment, 0, 255);
  GTuneChanged = true;        // force redraw
  SetCapacitance(Capacitance);
  DriveSolution();
}

//
// touch event - C+
//
void p0CPlusPushCallback(void *ptr)           // increase capacitance by one step
{
  int Capacitance;
  int Increment = 1; 
  if (IsCoarse)
    Increment *= VCOARSESTEP;
    
  Capacitance = GetCapacitance();
  Capacitance = constrain(Capacitance + Increment, 0, 255);
  GTuneChanged = true;        // force redraw
  SetCapacitance(Capacitance);
  DriveSolution();
}

//
// touch event - Coarse/Fine
// the display button sets its own text
//
void p0FinePushCallback(void *ptr)            // get new coarse/fine state
{
//  uint32_t ButtonValue = 0;

//  p0FineBtn.getValue(&ButtonValue);
//  if(ButtonValue == 0)
//    IsCoarse = true;
//  else
//    IsCoarse = false;
}

//
// touch event - High/Low Z
//
void p0HighZPushCallback(void *ptr)           // get new High/Low Z state
{
//  uint32_t ButtonValue = 0;
//  p0FineBtn.getValue(&ButtonValue);
//  if(ButtonValue == 0)
//    IsHighZ = false;
//  else
//    IsHighZ = true;
//  SetHiLoZ(IsHighZ);            // set to h/w
//  DriveSolution();
}



//
// initialise the UI and its sub-devices
//
void LCD_UI_Initialise(void)
{
  IsCoarse = true;
  GDisplayItem = 0;
  nexInit(115200);
  page0.attachPush(page0PushCallback);
  p0LMinusBtn.attachPush(p0LMinusPushCallback);
  p0LPlusBtn.attachPush(p0LPlusPushCallback);
  p0CMinusBtn.attachPush(p0CMinusPushCallback);
  p0CPlusBtn.attachPush(p0CPlusPushCallback);
  p0FineBtn.attachPush(p0FinePushCallback);
  p0HighZBtn.attachPush(p0HighZPushCallback);
//
// initialise display variables, then set all elements
//  
  GDisplayedL = 0;               // value of L posted to display, to check for updates
  p0LValue.setText("0");
  GDisplayedC = 0;               // value of C posted to display, to check for updates
  p0CValue.setText("0");
  GDisplayedVSWR = 10;           // VSWR value displayed
  p0VSWRValue.setText("1.0");
  GDisplayedPower = 0;           // power value displayed
  p0PowerValue.setText("0");
  GDisplayedVf = 0;              // forward voltage displayed
  p0VfValue.setText("0");
  GDisplayedVr = 0;              // reverse voltage displayed
  p0VrValue.setText("0");
  GDisplayedPTT = false;         // PTT value displayed
  p0PTT.setText("no PTT");
  GDisplayedTune = false;        // tune state displayed
  p0Tune.setText("No Tune");
  GDisplayedFreq = 0;              // frequency displayed
  p0FreqValue.setText("0");
  p0AntValue.setText("0");
  p0Enabled.setText("Disabled");
  GDisplayedZ = false;
}


//
// force LCD update
//
void SetTuneChanged()
{
  GTuneChanged = true;
}


#define VASCII0 0x30                // zero character in ASCII
//
// local version of "sprintf like" function
// Adds a decimal point before last digit if 3rd parameter set
// note integer value is signed and may be negative!
//
unsigned char mysprintf(char *dest, int Value, bool AddDP)
{
  unsigned char Digit;              // calculated digit
  bool HadADigit = false;           // true when found a non zero digit
  unsigned char DigitCount = 0;     // number of returned digits
  unsigned int Divisor = 10000;     // power of 10 being calculated
//
// deal with negative values first
//
  if (Value < 0)
  {
    *dest++ = '-';    // add to output
    DigitCount++;
    Value = -Value;
  }
//
// now convert the (definitely posirive) number
//
  while (Divisor >= 10)
  {
    Digit = Value / Divisor;        // find digit: integer divide
    if (Digit != 0)
      HadADigit = true;             // flag if non zero so all trailing digits added
    if (HadADigit)                  // if 1st non zero or all subsequent
    {
      *dest++ = Digit + VASCII0;    // add to output
      DigitCount++;
    }
    Value -= (Digit * Divisor);     // get remainder from divide
    Divisor = Divisor / 10;         // ready for next digit
  }
//
// if we need a decimal point, add it now. Also if there hasn't been a preceiding digit
// (i.e. number was like 0.3) add the zero
//
  if (AddDP)
  {
    if (HadADigit == false)
    {
      *dest++ = '0';
      DigitCount++;
    }
    *dest++ = '.';
  DigitCount++;
  }
  *dest++ = Value + VASCII0;
  DigitCount++;
//
// finally terminate with a 0
//
  *dest++ = 0;

  return DigitCount;
}


//
// periodic timer tick for encoders
// (this is called every 2ms, servicing alternate encoders so each is serviced every 4ms)
//
void LCD_UI_EncoderTick(bool OddEncoder)
{

//
// handle touch display events
//  
  nexLoop(nex_listen_list);

  
  if(OddEncoder)
    Encoder1.service();     // update encoder states
  else
    Encoder2.service();
}


//
// periodic timer 16 ms tick
// this is only called if the conditional compilation for it is enabled
//
void LCD_UI_Tick(void)
{
  char LocalStr[20];
  char LocalStr2[20];

  bool ZValue;                                            // current hi/low Z value
  int LValue, CValue;                                     // current inductance and capacitance
  EButtonEvent EncoderPressEvent;
  EButtonEvent TunePressEvent;
  signed char LMovement, CMovement;
  int NewInductance, NewCapacitance;
  unsigned int ADCMean, ADCPeak;

//
// update pushbuttons
//
  EncoderPressEvent = EncoderBtnDebounce.Tick();
  TunePressEvent = TuneBtnDebounce.Tick();
  
  LMovement = Encoder1.getValue();
  CMovement = Encoder2.getValue();


//
// update pushbutton events
//
  if (EncoderPressEvent == ePressed)                    // normal press toggles "coarse"
  {
    IsCoarse = !IsCoarse;
    if(IsCoarse)
    {
      p0FineBtn.setValue(0);
      p0FineBtn.setText("Coarse");
    }
    else
    {
      p0FineBtn.setValue(1);
      p0FineBtn.setText("Fine");
    }
  }

  ZValue = GetHiLoZ();
  if (EncoderPressEvent == eLongPressed)                // encoder longpress toggles "hi/low Z"
  {
    ZValue = !ZValue;
    SetHiLoZ(ZValue);            // set to h/w
    if(ZValue)
    {
      p0HighZBtn.setValue(1);
      p0HighZBtn.setText("High Z");
    }
    else
    {
      p0HighZBtn.setValue(0);
      p0HighZBtn.setText("Low Z");
    }

    DriveSolution();
  }


//
// manually initiate the tuning algorithm if button pressed
//
  if (TunePressEvent == ePressed)
  {
#ifdef CONDITIONAL_ALG_DEBUG
      Serial.println("manual TUNE pressed");                                // debug to confirm state
#endif
    
    InitiateTune(true);
    p0Tune.setText("Tuning");
  }

//
// update encoders
//
  if (IsCoarse)
  {
    LMovement *= VCOARSESTEP;
    CMovement *= VCOARSESTEP;
  }

  LValue = GetInductance();
  CValue = GetCapacitance();
  LValue = constrain(LValue + LMovement, 0, 255);
  CValue = constrain(CValue + CMovement, 0, 255);
  if (LMovement != 0)
  {
    GTuneChanged = true;        // force redraw
    SetInductance(LValue);
    DriveSolution();
  }

  if (CMovement != 0)
  { 
    GTuneChanged = true;        // force redraw
    SetCapacitance(CValue);
    DriveSolution();
  }

//
// update a display element: one per tick
//
  switch(GDisplayItem)
  {
    case 0:                               // L Value
      if(GDisplayedL != LValue)
      {
        mysprintf(LocalStr, LValue, false); 
        GDisplayedL = LValue;
        p0LValue.setText(LocalStr);
      }
      break;

    case 1:                               // C Value
      if(GDisplayedC != CValue)
      {
        mysprintf(LocalStr, CValue, false); 
        GDisplayedC = CValue;
        p0CValue.setText(LocalStr);
      }
      break;

    case 2:                               // VSWR Value
      if (GForwardPower == 0)
      {
        GDisplayedVSWR = 0;               // illegal value, so will always be redrawn
        p0VSWRValue.setText("---");
      }
      else if(GDisplayedVSWR != GIntVSWR)
      {
        mysprintf(LocalStr, GIntVSWR, true);          // 1dp
        GDisplayedVSWR = GIntVSWR;
        p0VSWRValue.setText(LocalStr);
      }
      break;

    case 3:                               // power Value
      if(GDisplayedPower != GForwardPower)
      {
        mysprintf(LocalStr, GForwardPower, false);   
        GDisplayedPower = GForwardPower;
        p0PowerValue.setText(LocalStr);
      }
      break;

    case 4:                               // Vf Value
      GetADCMeanAndPeak(true, &ADCMean, &ADCPeak);
      mysprintf(LocalStr, ADCMean, false); 
      mysprintf(LocalStr2, ADCPeak, false);
      strcat(LocalStr, " ");
      strcat(LocalStr, LocalStr2);
      p0VfValue.setText(LocalStr);
      break;

    case 5:                               // Vr Value
      GetADCMeanAndPeak(false, &ADCMean, &ADCPeak);
      mysprintf(LocalStr, ADCMean, false); 
      mysprintf(LocalStr2, ADCPeak, false);
      strcat(LocalStr, " ");
      strcat(LocalStr, LocalStr2);
      p0VrValue.setText(LocalStr);
      break;

    case 6:                               // PTT
      if(GDisplayedPTT != GPTTPressed)
      {
        GDisplayedPTT = GPTTPressed;
        if(GPTTPressed == true)
          p0PTT.setText("PTT");
        else
          p0PTT.setText("no PTT");
      }
      break;

    case 7:                               // Tune
      if(GDisplayedTune != GPCTuneActive)
      {
        GDisplayedTune = GPCTuneActive;
        if(GPCTuneActive == true)
          p0Tune.setText("TUNE");
        else
          p0Tune.setText("no Tune");
      }
      break;

    case 8:                               // High/Low z
      if(GDisplayedZ != ZValue)
      {
        GDisplayedZ = ZValue;
        if(ZValue)
        {
          p0HighZBtn.setValue(1);
          p0HighZBtn.setText("High Z");
        }
        else
        {
          p0HighZBtn.setValue(0);
          p0HighZBtn.setText("Low Z");
        }
      }
      break;
  }
  if(GDisplayItem >= 8)
    GDisplayItem = 0;
  else
    GDisplayItem++;
}


//
// set whether tuning or not
// true if tune is in progress
//
void LCD_UI_SetTuning(bool IsTuning)
{
#ifdef CONDITIONAL_LCD_UI
  
  if (!IsTuning)
  {
    p0Tune.setText("No Tune");
    GTuneChanged = true;              // set flag to cause redraw of display
  }
#endif
}


//
// set forward power and VSWR
// VSWR is passed as 10*"real" VSWR, to allow insertion of 1 decimal place
//
void SetPwr(int Power)
{
#ifdef CONDITIONAL_LCD_UI
  GForwardPower = Power;
#endif
}


void SetVSWR(int VSWR)
{
#ifdef CONDITIONAL_LCD_UI
  GIntVSWR = VSWR;
#endif
}


// debug
void ShowFrequency(char* FreqString)
{
  p0FreqValue.setText(FreqString);
}

void ShowTune(bool IsTune)
{
  if(IsTune)
    p0Tune.setText("Tune");
  else
    p0Tune.setText("No Tune");
}

void ShowAntenna(int Antenna)
{
  char LocalStr[20];

  mysprintf(LocalStr, Antenna, false);
  p0AntValue.setText(LocalStr);
}

void ShowATUEnabled(bool IsEnabled)
{
  if(IsEnabled)
    p0Enabled.setText("Enabled");
  else
    p0Enabled.setText("Disbled");
}

#endif
