/////////////////////////////////////////////////////////////////////////
//
// Aries ATU controller sketch by Laurence Barker G8NJJ
// this sketch controls an L-match ATU network
// with a CAT interface to connect to an HPSDR control program
// copyright (c) Laurence Barker G8NJJ 2019
//
// the code is written for an Arduino Nano 33 IoT module
//
// LCD_UI.cpp: LCD user interface for debug and VSWR display
//
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


//
// define colours for  Nextion display
//
#define NEXBLACK 0L
#define NEXWHITE 65535L
#define NEXRED 63488L
#define NEXGREEN 2016L
#define NEXBLUE 31L

#define VCOARSESTEP 8                               // L/C steps if coarse setpping
#define VENCODERDIVISOR 2                           // events per mechanical click



//
// paramters for display
//
#define VNEEDLESIZE 234.0
#define VMINXNEEDLEANGLE 13.5F                // angle for 0W
#define VMAXXNEEDLEANGLE 73.0F                // angle for full scale power
#define VXNEEDLEY1 239                        // y start position (px)
#define VXNEEDLEFWDX1 243                     // X needle start position (px)
#define VXNEEDLEREVX1 35                      // X needle start position (px)
#define VFIVESECONDS 312                      // at 16ms tick
#define VVSWRFULLSCALE 10.0F                  // full scale VSWR indication
//
// global variables
//
bool GIsPeakDisplay;                          // true if we are using peak display
EDisplayPage GDisplayPage;                    // global set to current display page number
int GSplashCountdown;                         // counter for splash page
byte GUpdateItem;                             // display item being updated
byte GCrossedNeedleItem;                      // display item in crossed needle page
int GDisplayedForward, GDisplayedReverse;     // displayed meter angle values, to find if needle has moved
int GReqdAngleForward, GReqdAngleReverse;     // required angles for crossed needle
byte GForwardOverscale, GReverseOverscale;    // set non zero if an overscale detected. =no. ticks to display for
bool GInitialisePage;                         // true if page needs to be initialised
bool GCrossedNeedleRedrawing;                 // true if display is being redrawn
unsigned char GUpdateMeterTicks;              // number of ticks since a meter display updated


bool GEncoderCapacitance;
bool IsCoarse;
bool GTuneChanged;              // true if a tune operation has happened and display to be updated
byte GVoltDisplayCount;

//
// display updates, for engineering display
//
unsigned int GDisplayItem;      // item to display at next tick
byte GDisplayedL;               // value of L posted to display, to check for updates
byte GDisplayedC;               // value of C posted to display, to check for updates
int GDisplayedVSWR;             // VSWR value displayed
int GDisplayedPower;            // power value displayed
int GDisplayedVf;               // forward voltage displayed
int GDisplayedVr;               // reverse voltage displayed
bool GDisplayedPTT;             // PTT value displayed
bool GDisplayedQuick;           // Quick tune value displayed
bool GDisplayedTune;            // tune state displayed
bool GDisplayedZ;               // true if High Z
int GDisplayedFreq;             // frequency displayed
byte GDisplayScale;             // scale in use (0-4)
NoClickEncoder Encoder1(VPINENCODER1B, VPINENCODER1A, VENCODERDIVISOR, true);
PushbuttonDebounce EncoderBtnDebounce(VPINENCODER1PB, 64);          // toggle L/C; longpress to toggle coarse/fine

//
// display full scale values for power graphs
// done this way it is easy to add more!
//
#define VMAXSCALESETTING 4
const unsigned int GFullPowerScale[VMAXSCALESETTING + 1]=
{
  100,
  200,
  500, 
  1000,
  2000
};

//
// picture id values for nextion display bargraphs
// these ID values must match the image ID values in the editor!
//
const unsigned int GPowerForeground[VMAXSCALESETTING + 1] =
{
  16,             // 100W display
  19,             // 200W display
  21,             // 500W display
  23,             // 1kW display
  3               // 2kW display
};

const unsigned int GPowerBackground[VMAXSCALESETTING + 1] =
{
  18,             // 100W display
  17,             // 200W display
  20,             // 500W display
  22,             // 1kW display
  2               // 2kW display
};


// picture ID values for the background image for the meter display
const unsigned int GMeterPicture[VMAXSCALESETTING + 1] =
{
  12,             // 100W display
  13,             // 200W display
  14,             // 500W display
  15,             // 1kW display
  6               // 2kW display
};



// picture ID values for the background image for the crossed needle display
const unsigned int GCrossedNeedlePicture[VMAXSCALESETTING + 1] =
{
  8,              // 100W display
  9,              // 200W display
  10,             // 500W display
  11,             // 1kW display
  7               // 2kW display
};


//
// display scale strings for page 5
//
const char* GDisplayScaleStrings[VMAXSCALESETTING + 1] =
{
  "100W (R12,13 = 2K7)",
  "200W (R12,13 = 4K7)",
  "500W (R12,13 = 8K2)",
  "1000W (R12,13 = 12K)",
  "2000W (R12,13 = 18K)"
};





//
// Nextion display object declarations
//

//
// declare pages:
//
NexPage page0 = NexPage(0, 0, "page0");       // creates touch event for "splash" page
NexPage page1 = NexPage(1, 0, "page1");       // creates touch event for "crossed needle" page
NexPage page2 = NexPage(2, 0, "page2");       // creates touch event for "power bargraph" page
NexPage page3 = NexPage(3, 0, "page3");       // creates touch event for "analogue meter" page
NexPage page4 = NexPage(4, 0, "page4");       // creates touch event for "engineering" page
NexPage page5 = NexPage(5, 0, "page5");       // creates touch event for "setup" page

//
// page 0 objects:
// 
NexText p0SWVersion = NexText(0, 5, "p0t4");                   // software version

//
// page 1 objects:
// 
NexPicture p1Axes = NexPicture(1, 1, "p1p0");                     // graph axes
NexButton p1DisplayBtn = NexButton(1, 2, "p1b0");                 // Display pushbutton
NexButton p1PeakBtn = NexButton(1, 3, "p1b1");                    // normal/peak button
NexButton p1EnableBtn = NexButton(1, 4, "p1b2");                  // Enable pushbutton
NexText p1Status = NexText(1, 5, "p1t0");                         // ready/tuning etc

//
// page 2 objects:
// 
NexButton p2DisplayBtn = NexButton(2, 1, "p2b0");                 // Display pushbutton
NexText p2FwdPower = NexText(2,5, "p2t2");                        // Forward power (W)
NexText p2VSWRTxt = NexText(2,8, "p2t3");                         // VSWR
NexProgressBar p2FwdBar = NexProgressBar(2, 3, "p2j0");           // forward power bar
NexProgressBar p2VSWRBar = NexProgressBar(2, 6, "p2j1");          // VSWR bar
NexButton p2PeakBtn = NexButton(2, 10, "p2b1");                   // normal/peak button
NexButton p2EnableBtn = NexButton(2, 11, "p2b2");                 // Enable pushbutton
NexText p2Status = NexText(2, 12, "p2t0");                        // ready/tuning etc

//
// page 3 objects:
// 
NexButton p3DisplayBtn = NexButton(3, 3, "p3b0");                 // Display pushbutton
NexButton p3PeakBtn = NexButton(3, 4, "p3b1");                    // normal/peak button
NexGauge p3Meter = NexGauge(3, 1, "p3z0");                        // power meter
NexProgressBar p3VSWRBar = NexProgressBar(3, 2, "p3j0");          // VSWR bar
NexButton p3EnableBtn = NexButton(3, 5, "p3b2");                  // Enable pushbutton
NexText p3Status = NexText(3, 6, "p3t0");                         // ready/tuning etc

//
// page 4 objects:
// 
NexText p4LValue = NexText(4, 3, "p4t1");                   // inductance value
NexText p4CValue = NexText(4, 5, "p4t2");                   // capacitance value
NexButton p4LMinusBtn = NexButton(4, 6, "p4b1");            // L- pushbutton
NexButton p4LPlusBtn = NexButton(4, 7, "p4b2");             // L+ pushbutton
NexButton p4CMinusBtn = NexButton(4, 8, "p4b3");            // C- pushbutton
NexButton p4CPlusBtn = NexButton(4, 9, "p4b4");             // C+ pushbutton
NexButton p4FineBtn = NexButton(4, 10, "p4b5");             // coarse/fine button
NexButton p4HighZBtn = NexButton(4, 11, "p4b6");            // High/LowZ button button
NexText p4VSWRValue = NexText(4, 13, "p4t3");               // VSWR value
NexText p4PowerValue = NexText(4, 15, "p4t4");              // RF Power value
NexText p4FreqValue = NexText(4, 22, "p4t7");               // Frequency value
NexText p4VfValue = NexText(4, 18, "p4t5");                 // Vf value
NexText p4VrValue = NexText(4, 20, "p4t6");                 // Vr value
NexText p4AntValue = NexText(4, 24, "p4t8");                // Antenna number
NexText p4Status = NexText(4, 27, "p4t0");                  // ready/tuning etc
NexText p4PTT = NexText(4, 26, "p4t9");                     // PTT on/off
NexText p4Quick = NexText(4, 28, "p4t11");                  // quick tune on/off
NexButton p4EnableBtn = NexButton(4, 25, "p4b7");           // Enable pushbutton
NexButton p4DisplayBtn = NexButton(4, 1, "p4b0");           // Setup page pushbutton
NexButton p4SetupBtn = NexButton(4, 29, "p4b8");            // Setup pushbutton

//
// page 5 objects:
// 
NexText p5EraseTxt = NexText(5, 11, "p5t0");                // erase progress message
NexText p5ScaleTxt = NexText(5, 2, "p5t1");                 // display scale string
NexButton p5Ant1Btn = NexButton(5, 7, "p5b2");              // Ant 1 erase pushbutton
NexButton p5Ant2Btn = NexButton(5, 8, "p5b3");              // Ant 2 erase pushbutton
NexButton p5Ant3Btn = NexButton(5, 9, "p5b4");              // Ant 3 erase pushbutton
NexButton p5Ant4Btn = NexButton(5, 10, "p5b5");             // Ant 4 erase pushbutton
NexButton p5ScaleBtn = NexButton(5, 4, "p5b1");             // change display scale pushbutton
NexButton p5RtnBtn = NexButton(5, 1, "p5b0");               // return display pushbutton
NexButton p5AlgBtn = NexButton(5, 12, "p5b6");               // algorithm "quick" pushbutton


//
// declare touch event objects to the touch event list
// this tells the code what touch events too look for
//
NexTouch *nex_listen_list[] = 
{
  &p1PeakBtn,                                 // average/peak power
  &p1EnableBtn,                               // ATU enable/disable
  &p1DisplayBtn,                              // change display button
  &p2PeakBtn,                                 // average/peak power
  &p2EnableBtn,                               // ATU enable/disable
  &p2DisplayBtn,                              // change display button
  &p3PeakBtn,                                 // average/peak power
  &p3EnableBtn,                               // ATU enable/disable
  &p3DisplayBtn,                              // change display button
  &p4LMinusBtn,                               // L- button press
  &p4LPlusBtn,                                // L+ button press
  &p4CMinusBtn,                               // C- button press
  &p4CPlusBtn,                                // C+ button press
  &p4FineBtn,                                 // coarse/fine button press
  &p4HighZBtn,                                // Low/High Z button
  &p4EnableBtn,                               // ATU enable/disable
  &p4DisplayBtn,                              // change display button
  &p4SetupBtn,                                // setup page button
  &p5Ant1Btn,                                 // Antenna 1 erase
  &p5Ant2Btn,                                 // Antenna 2 erase
  &p5Ant3Btn,                                 // Antenna 3 erase
  &p5Ant4Btn,                                 // Antenna 4 erase
  &p5ScaleBtn,                                // Display scale button
  &p5RtnBtn,                                  // Return display button
  &p5AlgBtn,                                  // algorithm "quick" button
  NULL                                        // terminates the list
};


////////////////////////////////////////////////////////////////////////////////////
// maths for power and VSWR displays

//
// convert from power value to degree angle for the cross needle meter
// return integer angle from 13.5 to 90
//
int GetCrossedNeedleDegrees(bool IsForward, bool IsPeak)
{
  float FullScale;
  unsigned int Power;
  float Degrees;
  
  FullScale = (float)GFullPowerScale[GDisplayScale];
  if(!IsForward)
    FullScale *= 0.2;                                     // reverse scale = a fifth of forward

  if(IsPeak)
    Power = FindPeakPower(IsForward);               // get power in watts
  else
    Power = GetPowerReading(IsForward);
//
// calculate angle. Not full scale ~73 degrees but we allow up to 90 degrees
// not the power reading is in tenths of watts so scale to watts too.
//
  Degrees = VMINXNEEDLEANGLE + (VMAXXNEEDLEANGLE - VMINXNEEDLEANGLE)* (float)Power / FullScale;
  if (Degrees > 90.0)                                    // now clip, and set overscale if needed
    Degrees = 90.0;
  return (int)Degrees;
}




//
// convert from power value to degree angle for meter
// return 0 to 180
//
int GetPowerMeterDegrees(bool IsForward, bool IsPeak)
{
  int FullScale;
  unsigned int Power;
  float Degrees;
  
  FullScale = (float)GFullPowerScale[GDisplayScale];
;
  if(IsPeak)
    Power = FindPeakPower(IsForward);            // get power in watts
  else
    Power = GetPowerReading(IsForward);

  Degrees = 180.0 * (float)Power / (float)FullScale;
  if (Degrees > 180.0)                                    // now clip, and set overscale if needed
    Degrees = 180.0;
  return (int)Degrees;
}



//
// convert from power value to % of full scale
// return 0 to 100
//
int GetPowerPercent(bool IsForward, bool IsPeak)
{
  int FullScale;
  unsigned int Power;
  float Percent;
  
  FullScale = (float)GFullPowerScale[GDisplayScale];
  if(IsPeak)
    Power = FindPeakPower(IsForward);            // get power in watts
  else
    Power = GetPowerReading(IsForward);

  Percent = 100.0 * (float)Power / (float)FullScale;
  if (Percent > 100.0)                                    // now clip, and set overscale if needed
    Percent = 100.0;
  return (int)Percent;
}



//
// convert from VSWR value to % of full scale
// begins with a number that has 1 fixed point
// return 0 to 100
//
int GetVSWRPercent(void)
{
  int Power;
  float Percent;
  int Result;

  Percent = (float)GVSWR * 100.0 / VVSWRFULLSCALE;
  if (Percent > 100.0)
    Result = 100;
  else
    Result = (int)Percent;
  return Result;
}

//////////////////////////////////////////////////////////////////////////////////////

//
// set "high/low Z" button text
//
void SetHighLowZButtonText()
{
  if(GDisplayPage == eEngineeringPage)
  {
    if(GetHiLoZ())
      p4HighZBtn.setText("High Z");
    else
      p4HighZBtn.setText("Low Z");
  }
}


//
// set "coarse/fine" button text
//
void SetCoarseButtonText()
{
  if(GDisplayPage == eEngineeringPage)
  {
    if(IsCoarse)
      p4FineBtn.setText("Coarse");
    else
      p4FineBtn.setText("Fine");
  }
}


//
// set "disabled/enabled" button text
// this has to find the control for the currently enabled page
//
void SetEnabledButtonText()
{
  NexButton* Ptr;

  switch(GDisplayPage)                              // get ptr to the right page's control
  {
    case eSplashPage:                              // startup splash page
      Ptr = NULL;
      break;
    case eCrossedNeedlePage:                       // crossed needle VSWR page display
      Ptr = &p1EnableBtn;
      break;
    case ePowerBargraphPage:                       // linear watts bargraph page display
      Ptr = &p2EnableBtn;
      break;
    case eMeterPage:                               // analogue power meter
      Ptr = &p3EnableBtn;
      break;
    case eEngineeringPage:                          // engineering page with raw ADC values
      Ptr = &p4EnableBtn;
      break;
  }
  if (Ptr != NULL)                                  // if there is a valid control, update it
  {
    if(GATUEnabled)
      Ptr->setText("Enabled");
    else
      Ptr->setText("Disabled");
  }
}




//
// set "peak/average" button text
// this has to find the control for the currently enabled page
//
void SetPeakButtonText()
{
  NexButton* Ptr;

  switch(GDisplayPage)                              // get ptr to the right page's control
  {
    case eSplashPage:                              // startup splash page
    case eEngineeringPage:                          // engineering page with raw ADC values
      Ptr = NULL;
      break;
    case eCrossedNeedlePage:                       // crossed needle VSWR page display
      Ptr = &p1PeakBtn;
      break;
    case ePowerBargraphPage:                       // linear watts bargraph page display
      Ptr = &p2PeakBtn;
      break;
    case eMeterPage:                               // analogue power meter
      Ptr = &p3PeakBtn;
      break;
  }
  if (Ptr != NULL)                                  // if there is a valid control, update it
  {
    if(GIsPeakDisplay)
      Ptr->setText("Peak");
    else
      Ptr->setText("Average");
  }
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// touch event handlers

//
// touch event - page 1 Peak
//
void p1PeakPushCallback(void *ptr)           // change peak/average display
{
  GIsPeakDisplay = !GIsPeakDisplay;
  SetPeakButtonText();
  EEWritePeak(GIsPeakDisplay);
}

//
// touch event - page 1 Enabled
//
void p1EnabledPushCallback(void *ptr)         // enable/disable ATU
{
  SetATUOnOff(!GATUEnabled);
  SetEnabledButtonText();
  EEWriteEnabled(GATUEnabled);
}

//
// touch event - page 1 Display (crossed needle)
//
void p1DisplayPushCallback(void *ptr)         // change display
{
  GDisplayPage = ePowerBargraphPage;
  page2.show();
  EEWritePage(2);
  GInitialisePage = true;
}



//
// touch event - page 2 Peak
//
void p2PeakPushCallback(void *ptr)           // change peak/average display
{
  GIsPeakDisplay = !GIsPeakDisplay;
  SetPeakButtonText();
  EEWritePeak(GIsPeakDisplay);
}

//
// touch event - page 2 Enabled
//
void p2EnabledPushCallback(void *ptr)         // enable/disable ATU
{
  SetATUOnOff(!GATUEnabled);
  SetEnabledButtonText();
  EEWriteEnabled(GATUEnabled);
}

//
// touch event - page 2 Display (power bargraph)
//
void p2DisplayPushCallback(void *ptr)         // change display
{
  if(!GTuneActive)
  {
    GDisplayPage = eMeterPage;
    page3.show();
    EEWritePage(3);
    GInitialisePage = true;
  }
}



//
// touch event - page 3 Peak
//
void p3PeakPushCallback(void *ptr)           // change peak/average display
{
  GIsPeakDisplay = !GIsPeakDisplay;
  SetPeakButtonText();
  EEWritePeak(GIsPeakDisplay);
}

//
// touch event - page 3 Enabled
//
void p3EnabledPushCallback(void *ptr)         // enable/disable ATU
{
  SetATUOnOff(!GATUEnabled);
  SetEnabledButtonText();
  EEWriteEnabled(GATUEnabled);
}

//
// touch event - page 3 Display (analogue meter)
//
void p3DisplayPushCallback(void *ptr)         // change display
{
  if (!GTuneActive)
  {
    GDisplayPage = eEngineeringPage;
    page4.show();
    EEWritePage(4);
    GInitialisePage = true;
  }
}



//
// touch event - page 4 Enabled
//
void p4EnabledPushCallback(void *ptr)         // enable/disable ATU
{
  SetATUOnOff(!GATUEnabled);                  // change state for hardware
  SetEnabledButtonText();
  EEWriteEnabled(GATUEnabled);
}


//
// touch event - page 4 Display (engineering)
//
void p4DisplayPushCallback(void *ptr)         // change display
{
  if (!GTuneActive)
  {
    GDisplayPage = eCrossedNeedlePage;
    page1.show();
    EEWritePage(1);
    GInitialisePage = true;
  }
}


//
// touch event - page 4 Setup
//
void p4SetupPushCallback(void *ptr)         // change display to page 5
{
    GDisplayPage = eSetupPage;
    page5.show();
    GInitialisePage = true;
}



//
// touch event - page 4 L-
//
void p4LMinusPushCallback(void *ptr)          // reduce inductance by one step
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
// touch event - page 4 L+
//
void p4LPlusPushCallback(void *ptr)           // increase inductance by one step
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
// touch event - page 4 C-
//
void p4CMinusPushCallback(void *ptr)          // reduce capacitance by one step
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
// touch event - page 4 C+
//
void p4CPlusPushCallback(void *ptr)           // increase capacitance by one step
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
// touch event - page 4 Coarse/Fine
//
void p4FinePushCallback(void *ptr)            // get new coarse/fine state
{
  IsCoarse = !IsCoarse;
  SetCoarseButtonText();
}

//
// touch event - page 4 High/Low Z
//
void p4HighZPushCallback(void *ptr)           // get new High/Low Z state
{
  bool ZValue;
  ZValue = GetHiLoZ();
  ZValue = !ZValue;
  SetHiLoZ(ZValue);            // set to h/w
// display update is done in the display cycle round values section later
  DriveSolution();
}

//
// touch event - page 5 Antenna 1
//
void p5Ant1PushCallback(void *ptr)         // erase antenna 1
{
  p5EraseTxt.setText("Erasing");
  EEEraseSolutionSet(1);
  p5EraseTxt.setText("Done");
}

//
// touch event - page 5 Antenna 2
//
void p5Ant2PushCallback(void *ptr)         // erase antenna 2
{
  p5EraseTxt.setText("Erasing");
  EEEraseSolutionSet(2);
  p5EraseTxt.setText("Done");
}

//
// touch event - page 5 Antenna 3
//
void p5Ant3PushCallback(void *ptr)         // erase antenna 3
{
  p5EraseTxt.setText("Erasing");
  EEEraseSolutionSet(3);
  p5EraseTxt.setText("Done");
}

//
// touch event - page 5 Antenna 4
//
void p5Ant4PushCallback(void *ptr)         // erase antenna 4
{
  p5EraseTxt.setText("Erasing");
  EEEraseSolutionSet(4);
  p5EraseTxt.setText("Done");
}

//
// touch event - page 5 display scale
//
void p5ScalePushCallback(void *ptr)         // set display scale
{
  if(++GDisplayScale > VMAXSCALESETTING)         // increment, save to eeprom then display
    GDisplayScale = 0;
  EEWriteScale(GDisplayScale);
  p5ScaleTxt.setText(GDisplayScaleStrings[GDisplayScale]);
}

//
// touch event - page 5 Return
//
void p5RtnPushCallback(void *ptr)           // change display
{
    GDisplayPage = eEngineeringPage;
    page4.show();
    GInitialisePage = true;
}

//
// touch event - page 5 Algorithm quick/not quick
//
void p5AlgPushCallback(void *ptr)           // change algorithm setting
{
  GQuickTuneEnabled = !GQuickTuneEnabled;
  EEWriteQuick(GQuickTuneEnabled);
  if(GQuickTuneEnabled)
    p5AlgBtn.setText("Quick");
  else
    p5AlgBtn.setText("Full");
}


// end of event handlers for button presses
///////////////////////////////////////////////////////////////////////////////////////////


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
// set foreground and background of bargraphs to set power scale
//
void SetBargraphImages(void)
{
  byte Image;
  char Str[20];
  char Str2[5];

  Str2[0]=0;                                            // empty the string
  if(GDisplayPage == 2)
  {
    Image = GPowerForeground[GDisplayScale];       // foreground image number
    mysprintf(Str2, Image, false);
    strcpy(Str,"p2j0.ppic=");
    strcat(Str, Str2);
    sendCommand(Str);
    Image = GPowerBackground[GDisplayScale];       // background image number
    mysprintf(Str2, Image, false);
    strcpy(Str,"p2j0.bpic=");
    strcat(Str, Str2);
    sendCommand(Str);
  }
}


//
// set background for meter display to set the power scale
//
void SetMeterImages(void)
{
  byte Image;
  char Str[20];
  char Str2[5];

  Str2[0]=0;                                            // empty the string
  if(GDisplayPage == 3)
  {
    Image = GMeterPicture[GDisplayScale];       // foreground image number
    mysprintf(Str2, Image, false);
    strcpy(Str,"p3z0.picc=");
    strcat(Str, Str2);
    sendCommand(Str);
  }
}



//
// set background for crossed needle display to set the power scale
//
void SetCrossedNeedleImages(void)
{
  byte Image;
  if(GDisplayPage == 1)
  {
    Image = GCrossedNeedlePicture[GDisplayScale];       // foreground image number
    p1Axes.setPic(Image);
  }
}



//
// initialise the UI and its sub-devices
//
void LCD_UI_Initialise(void)
{
  char Str[10];

  GDisplayScale = EEReadScale();                              // get display scale to use
  SetADCScaleFactor(GDisplayScale);
  
  IsCoarse = true;
  GDisplayItem = 0;
  nexInit(115200);
  p1PeakBtn.attachPush(p1PeakPushCallback);
  p1EnableBtn.attachPush(p1EnabledPushCallback);
  p1DisplayBtn.attachPush(p1DisplayPushCallback);
  p2PeakBtn.attachPush(p2PeakPushCallback);
  p2EnableBtn.attachPush(p2EnabledPushCallback);
  p2DisplayBtn.attachPush(p2DisplayPushCallback);
  p3PeakBtn.attachPush(p3PeakPushCallback);
  p3EnableBtn.attachPush(p3EnabledPushCallback);
  p3DisplayBtn.attachPush(p3DisplayPushCallback);
  p4EnableBtn.attachPush(p4EnabledPushCallback);
  p4DisplayBtn.attachPush(p4DisplayPushCallback);
  p4SetupBtn.attachPush(p4SetupPushCallback);
  p4LMinusBtn.attachPush(p4LMinusPushCallback);
  p4LPlusBtn.attachPush(p4LPlusPushCallback);
  p4CMinusBtn.attachPush(p4CMinusPushCallback);
  p4CPlusBtn.attachPush(p4CPlusPushCallback);
  p4FineBtn.attachPush(p4FinePushCallback);
  p4HighZBtn.attachPush(p4HighZPushCallback);
  p5Ant1Btn.attachPush(p5Ant1PushCallback);
  p5Ant2Btn.attachPush(p5Ant2PushCallback);
  p5Ant3Btn.attachPush(p5Ant3PushCallback);
  p5Ant4Btn.attachPush(p5Ant4PushCallback);
  p5ScaleBtn.attachPush(p5ScalePushCallback);
  p5RtnBtn.attachPush(p5RtnPushCallback);
  p5AlgBtn.attachPush(p5AlgPushCallback);
//
// initialise display variables, then set all elements
//  
  GDisplayedL = 0;               // value of L posted to display, to check for updates
  GDisplayedC = 0;               // value of C posted to display, to check for updates
  GDisplayedVSWR = 10;           // VSWR value displayed
  GDisplayedPower = 0;           // power value displayed
  GDisplayedVf = 0;              // forward voltage displayed
  GDisplayedVr = 0;              // reverse voltage displayed
  GDisplayedPTT = false;         // PTT value displayed
  GDisplayedTune = false;        // tune state displayed
  GDisplayedFreq = 0;              // frequency displayed
  GDisplayedZ = false;

  GIsPeakDisplay = EEReadPeak();
  mysprintf(Str, SWVERSION, false);                 // set s/w version on splash page
  p0SWVersion.setText(Str);
  GSplashCountdown = VFIVESECONDS;                  // ticks to stay in splash page
  GUpdateItem = 0;
}


//
// force LCD update
//
void SetTuneChanged()
{
  GTuneChanged = true;
}


//
// set status string, for all display pages
//
void SetStatusString(void)
{
  char StatusString[20];                       // ATU status string
  
  if(!GATUEnabled)
    strcpy(StatusString, "Disabled");
  else if (GTuneActive)
    strcpy(StatusString, "Tuning");
  else if (GValidSolution)
    strcpy(StatusString, "ATU Tuned");
  else
    strcpy(StatusString, "No Tune");

  switch(GDisplayPage)
  {
    case eSplashPage:                              // startup splash page
      break;
    case eCrossedNeedlePage:                       // crossed needle VSWR page display
      p1Status.setText(StatusString);
      break;
    case ePowerBargraphPage:                       // linear watts bargraph page display
      p2Status.setText(StatusString);
      break;
    case eMeterPage:                               // analogue power meter
      p3Status.setText(StatusString);
      break;
    case eEngineeringPage:                          // engineering page with raw ADC values
      p4Status.setText(StatusString);
      break;
  }
}

//
// periodic timer tick for encoders
// (this is called every 2ms, servicing alternate encoders so each is serviced every 4ms)
//
void LCD_UI_EncoderTick(bool OddEncoder)
{
    Encoder1.service();     // update encoder states
}



//
// NextionDisplayTick()
// called from the LCD_UI_Tick() below
// (separated to break the code up a bit)
//
void NextionDisplayTick(void)
{
  char Str[20];
  char Str2[20];
  byte InitialPage;
  unsigned int ADCMean, ADCPeak;
  int Value;
  bool ZValue;  
  int X1, Y1, X2, Y2;
  int Forward, Reverse;
  float X,Y;
  float Angle;

  nexLoop(nex_listen_list);
  switch(GDisplayPage)
  {
    default:
      page4.show();
      GDisplayPage = eEngineeringPage;
      GInitialisePage = true;
      break;

    case  eSplashPage:                              // startup splash - nothing to add to display
      if(GSplashCountdown-- <= 0)
      {
        InitialPage = EEReadPage();
        if(InitialPage == 4)                  // choose the operating page from eeprom stored value
        {
          page4.show();
          GDisplayPage = eEngineeringPage;
          GInitialisePage = true;
        }
        else if(InitialPage == 3)
        {
          page3.show();
          GDisplayPage = eMeterPage;
          GInitialisePage = true;
        }
        else if(InitialPage == 2)
        {
          page2.show();
          GDisplayPage = ePowerBargraphPage;
          GInitialisePage = true;
        }
        else
        {
          page1.show();
          GDisplayPage = eCrossedNeedlePage;
          GInitialisePage = true;
        }
      }
      break;

///////////////////////////////////////////////////

    case  eCrossedNeedlePage:                         // crossed needle VSWR page display
      GUpdateMeterTicks++;                            // update ticks since last updated
      if(GInitialisePage == true)                     // load background pics
      {
        GInitialisePage = false;
        SetCrossedNeedleImages();                     // get correct display scales
        GDisplayedForward = -100;                     // set illegal display angles
        GDisplayedReverse = -100;
        GCrossedNeedleRedrawing = false;
        SetPeakButtonText();
        SetEnabledButtonText();
      }
      else
      {
//
// first see what the angles should be, and if they are different from what's drawn.
// we only trest this is we are NOT already redrawing the display.
// redraw once a second even if no change
//
        if(!GCrossedNeedleRedrawing)
        {
          Forward = GetCrossedNeedleDegrees(true, GIsPeakDisplay);
          Reverse = GetCrossedNeedleDegrees(false, GIsPeakDisplay);
          if((Forward != GDisplayedForward) || (Reverse != GDisplayedReverse) || (GUpdateMeterTicks >= 50))
          {
            GCrossedNeedleRedrawing = true;           // if changed, set need to redraw display and required angles
            GDisplayedForward = Forward;
            GDisplayedReverse = Reverse;
            GUpdateItem = 0;                          // set to do 1st stage
          }
        }

//
// then if we need to update display, get on and draw it in sections
//
        if(GCrossedNeedleRedrawing)
        {
          GUpdateMeterTicks = 0;
          switch(GUpdateItem++)
          {
            case 0:                                     // erase image
              sendCommand("ref 1");
              break;
  
            case 1:                                     // draw reverse power line
              X1 = VXNEEDLEREVX1;
              Y1 = VXNEEDLEY1;
              Angle = (float)GDisplayedReverse * M_PI / 180.0;
              X = X1 + VNEEDLESIZE * cos(Angle);
              X2 = (int)X;
              Y = Y1 - VNEEDLESIZE * sin(Angle);
              Y2 = (int)Y;
// get text commands
              strcpy(Str, "line ");             // line
              mysprintf(Str2, X1, false);
              strcat(Str, Str2);
              strcat(Str, ",");                 // line X1,
              mysprintf(Str2, Y1, false);
              strcat(Str, Str2);
              strcat(Str, ",");                 // line X1,Y1,
              mysprintf(Str2, X2, false);
              strcat(Str, Str2);
              strcat(Str, ",");                 // line X1,Y1,X2
              mysprintf(Str2, Y2, false);
              strcat(Str, Str2);
              strcat(Str, ",BLUE");             // line X1,Y1,X2,Y2,BLUE
              sendCommand(Str);
              break;

            case 2:                                     // draw forward power line
              X1 = VXNEEDLEFWDX1;
              Y1 = VXNEEDLEY1;
              Angle = (float)GDisplayedForward * M_PI / 180.0;
              X = X1 - VNEEDLESIZE * cos(Angle);
              X2 = (int)X;
              Y = Y1 - VNEEDLESIZE * sin(Angle);
              Y2 = (int)Y;
// get text commands
              strcpy(Str, "line ");             // line
              mysprintf(Str2, X1, false);
              strcat(Str, Str2);
              strcat(Str, ",");                 // line X1,
              mysprintf(Str2, Y1, false);
              strcat(Str, Str2);
              strcat(Str, ",");                 // line X1,Y1,
              mysprintf(Str2, X2, false);
              strcat(Str, Str2);
              strcat(Str, ",");                 // line X1,Y1,X2
              mysprintf(Str2, Y2, false);
              strcat(Str, Str2);
              strcat(Str, ",BLUE");             // line X1,Y1,X2,Y2,BLUE
              sendCommand(Str);
              break;
              
            case 6: 
              SetStatusString();
              break;
            case 7:                                     // end of dwell after redraw
              GCrossedNeedleRedrawing = false;
              break;
          }
        }   // switch
      }
      break;
/////////////////////////////////////////////

    case  ePowerBargraphPage:                         // bargraph page display
      if(GInitialisePage)
      {
        SetPeakButtonText();
        SetEnabledButtonText();
        SetBargraphImages();                            // get correct display scales
        GInitialisePage = false;
      }
      else
        switch(GUpdateItem)
        {
          case 0:
            Forward = GetPowerPercent(true, GIsPeakDisplay);
            p2FwdBar.setValue(Forward);
            break;
  
          case 4:
            Forward = GetVSWRPercent();
            p2VSWRBar.setValue(Forward);
            break;
  
          case 8:
            if(GIsPeakDisplay)
              Forward = FindPeakPower(true);                 // get forward peak power, in watts
            else
              Forward = GetPowerReading(true);               // get forward power, in watts
            mysprintf(Str, Forward, false);
            p2FwdPower.setText(Str);
  
            mysprintf(Str, GVSWR*10, true);
            p2VSWRTxt.setText(Str);
            break;

          case 10:
            SetStatusString();
            break;
        }          
      if (GUpdateItem++ >= 10)
        GUpdateItem = 0;
      break;
////////////////////////////////////////////////
    
    case  eMeterPage:                                 // analogue meter page display
      if(GInitialisePage)
      {
        SetPeakButtonText();
        SetEnabledButtonText();
        SetMeterImages();                            // get correct display scales
        GInitialisePage = false;
      }
      else
        switch(GUpdateItem)
        {
          case 0:
            Forward = GetPowerMeterDegrees(true, GIsPeakDisplay);
            p3Meter.setValue(Forward);
            break;
  
          case 10:
            Forward = GetVSWRPercent();
            p3VSWRBar.setValue(Forward);
            break;

          case 14:
            SetStatusString();
            break;

        }          
      if (GUpdateItem++ >= 14)
        GUpdateItem = 0;
      break;
////////////////////////////////////////////////
      
    case  eEngineeringPage:                           // engineering page display

      if(GInitialisePage)                       // initialise the controls not refreshed often
      {
        mysprintf(Str, GTunedFrequency10, false);
        p4FreqValue.setText(Str);
        mysprintf(Str, GTXAntenna, false);
        p4AntValue.setText(Str);
        SetEnabledButtonText();
        SetHighLowZButtonText();
        SetCoarseButtonText();
        GInitialisePage = false;
      }
      else                                      // update a display element: one per tick
      {
        switch(GDisplayItem)
        {
          case 0:                               // L Value
            Value = GetInductance();
            if(GDisplayedL != Value)
            {
              mysprintf(Str, Value, false); 
              GDisplayedL = Value;
              p4LValue.setText(Str);
            }
            break;
      
          case 1:                               // C Value
            Value = GetCapacitance();
            if(GDisplayedC != Value)
            {
              mysprintf(Str, Value, false); 
              GDisplayedC = Value;
              p4CValue.setText(Str);
            }
            break;
      
          case 2:                               // VSWR Value
            Value = (int)(GVSWR*10.0);
            if (GForwardPower == 0)
            {
              GDisplayedVSWR = 0;               // illegal value, so will always be redrawn
              p4VSWRValue.setText("---");
            }
            else if(GDisplayedVSWR != Value)
            {
              mysprintf(Str, Value, true);          // 1dp
              GDisplayedVSWR = Value;
              p4VSWRValue.setText(Str);
            }
            break;
      
          case 3:                               // power Value
            if(GDisplayedPower != GForwardPower)
            {
              mysprintf(Str, GForwardPower, false);   
              GDisplayedPower = GForwardPower;
              p4PowerValue.setText(Str);
            }
            break;
      
          case 4:                               // Vf Value
            GetADCMeanAndPeak(true, &ADCMean, &ADCPeak);
            mysprintf(Str, ADCMean, false); 
            mysprintf(Str2, ADCPeak, false);
            strcat(Str, " ");
            strcat(Str, Str2);
            p4VfValue.setText(Str);
            break;
      
          case 5:                               // Vr Value
            GetADCMeanAndPeak(false, &ADCMean, &ADCPeak);
            mysprintf(Str, ADCMean, false); 
            mysprintf(Str2, ADCPeak, false);
            strcat(Str, " ");
            strcat(Str, Str2);
            p4VrValue.setText(Str);
            break;
      
          case 6:                               // PTT & quick tune (won't change together)
            if(GDisplayedPTT != GPTTPressed)
            {
              GDisplayedPTT = GPTTPressed;
              if(GPTTPressed == true)
                p4PTT.setText("PTT");
              else
                p4PTT.setText("no PTT");
            }
            if(GDisplayedQuick != GQuickTuneEnabled)
            {
              GDisplayedQuick = GQuickTuneEnabled;
              if(GQuickTuneEnabled == true)
                p4Quick.setText("Quick");
              else
                p4Quick.setText("Full");
            }
            break;
      
          case 7:                               // Status
            SetStatusString();
            break;
      
          case 8:                               // High/Low z
            ZValue = GetHiLoZ();
            if(GDisplayedZ != ZValue)
            {
              GDisplayedZ = ZValue;
              SetHighLowZButtonText();
            }
            break;
        }
        if(GDisplayItem >= 8)
          GDisplayItem = 0;
        else
          GDisplayItem++;
      }
      break;


    case eSetupPage:
      if(GInitialisePage)                       // initialise the controls not refreshed often
      {
        if(GDisplayScale > VMAXSCALESETTING)
          GDisplayScale = VMAXSCALESETTING;
        p5ScaleTxt.setText(GDisplayScaleStrings[GDisplayScale]);
        if(GQuickTuneEnabled)
          p5AlgBtn.setText("Quick");
        else
          p5AlgBtn.setText("Full");
        GInitialisePage = false;
      }
      break;
  }
}



//
// periodic timer 16 ms tick
// this is only called if the conditional compilation for it is enabled
//
void LCD_UI_Tick(void)
{
  bool ZValue;                                            // current hi/low Z value
  int LValue, CValue;                                     // current inductance and capacitance
  EButtonEvent EncoderPressEvent;
  signed char LMovement=0, CMovement=0;
  int NewInductance, NewCapacitance;
//
// update pushbutton
//
  EncoderPressEvent = EncoderBtnDebounce.Tick();

//
// update manual tune of inductance/capacitance
//
  if(GEncoderCapacitance)
    CMovement = Encoder1.getValue();
  else
    LMovement = Encoder1.getValue();

//
// update pushbutton events
//
  if (EncoderPressEvent == eLongPressed)                    // normal press toggles "coarse"
  {
    IsCoarse = !IsCoarse;
    SetCoarseButtonText();
  }

  if (EncoderPressEvent == ePressed)                // encoder longpress toggles "hi/low Z"
  {
    GEncoderCapacitance = !GEncoderCapacitance;
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
// now update the display
//
  NextionDisplayTick();
}



// debug
void ShowFrequency(char* FreqString)
{
  if(GDisplayPage == eEngineeringPage)
    p4FreqValue.setText(FreqString);
}

void ShowTune()
{
  SetStatusString();
}

void ShowAntenna(int Antenna)
{
  char LocalStr[20];
  if(GDisplayPage == eEngineeringPage)
  {
    mysprintf(LocalStr, Antenna, false);
    p4AntValue.setText(LocalStr);
  }
}

void ShowATUEnabled(bool IsEnabled)
{
  SetStatusString();
  SetEnabledButtonText();
}
