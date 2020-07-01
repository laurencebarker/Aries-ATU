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

#include <LiquidCrystal_I2C.h>
#include "mechencoder.h"
#include "pushbutton.h"
#include "iopins.h"
#include "hwdriver.h"
#include "algorithm.h"
#include "globalinclude.h"


//
// global variables
//

#define VCOARSESTEP 8                               // L/C steps if coarse setpping
#define VENCODERDIVISOR 2                           // events per mechanical click

byte GLValue, GCValue;          // user L, C settings
bool IsCoarse;
bool IsHighZ;                   // true for High Z
bool GTuneChanged;              // true if a tune operation has happened and display to be updated
int GForwardPower;              // VSWR bridge forward power
int GIntVSWR;                   // VSWR bridge calculated VSWR (1 DP fixed point, so 10 X value needed)
byte GVoltDisplayCount;

NoClickEncoder Encoder1(VPINENCODER1B, VPINENCODER1A, VENCODERDIVISOR, true);
NoClickEncoder Encoder2(VPINENCODER2B, VPINENCODER2A, VENCODERDIVISOR, true);
LiquidCrystal_I2C lcd(0x27, 20, 4); // Addr, columns, rows
PushbuttonDebounce EncoderBtnDebounce(VPINENCODER1PB, 64);          // toggle coarse/fine; longpress to toggle low/high Z
PushbuttonDebounce TuneBtnDebounce(VPINPUSHBUTTONTUNE);             // initiates tune; no longpress



//
// initialise the UI and its sub-devices
//
void LCD_UI_Initialise(void)
{
  IsCoarse = true;
  
  lcd.begin(20,4);
  lcd.backlight();
  lcd.setCursor(0, 0);        //1st column, first row
  lcd.print("L=    ");
  lcd.setCursor(2, 0);        //3rd column, first row
  lcd.print(GLValue);
  lcd.setCursor(0, 1);        //1st column, second row
  lcd.print("C=    ");
  lcd.setCursor(2, 1);        //1st column, second row
  lcd.print(GCValue);
  lcd.setCursor(0,3);
  lcd.print("Coarse");
  lcd.setCursor(7,3);
  lcd.print("Lo Z");
  lcd.setCursor(12,3);
  lcd.print("Pwr    W");
  lcd.setCursor(16,3);
  lcd.print(GForwardPower);
  lcd.setCursor(11,2);
  lcd.print("VSWR ");
  lcd.setCursor(16,2);
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
// (this could be called more often than main tick)
//
void LCD_UI_EncoderTick(bool OddEncoder)
{
  if(OddEncoder)
    Encoder1.service();     // update encoder states
  else
    Encoder2.service();
}


//
// periodic timer tick
//
void LCD_UI_Tick(void)
{
  char LocalStr[20];
  EButtonEvent EncoderPressEvent;
  EButtonEvent TunePressEvent;
  signed char LMovement, CMovement;
  int NewInductance, NewCapacitance;

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
      lcd.setCursor(0,3);
    if(IsCoarse)
      lcd.print("Coarse");
    else
      lcd.print("Fine  ");
  }

  if (EncoderPressEvent == eLongPressed)                // encoder longpress toggles "hi/low Z"
  {
    IsHighZ = !IsHighZ;
    SetHiLoZ(IsHighZ);            // set to h/w
    GTuneChanged = true;          // force redraw   
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
    lcd.setCursor(0, 2);        //1st column, third row
    lcd.print("tuning  ");
  }

//
// update encoders
//
  if (IsCoarse)
  {
    LMovement *= VCOARSESTEP;
    CMovement *= VCOARSESTEP;
  }
  
  GLValue = constrain(GLValue + LMovement, 0, 255);
  GCValue = constrain(GCValue + CMovement, 0, 255);
  if (LMovement != 0)
  {
    GTuneChanged = true;        // force redraw
    SetInductance(GLValue);
    DriveSolution();
  }

  if (CMovement != 0)
  { 
    GTuneChanged = true;        // force redraw
    SetCapacitance(GCValue);
    DriveSolution();
  }

//
// finally redraw the display if there has been a change of tune settings
//  
  if (GTuneChanged)
  {
    lcd.clear();
    lcd.setCursor(7,3);            // low/high Z setting
    if(IsHighZ)
      lcd.print("Hi Z");
    else
      lcd.print("Lo Z");
    
    lcd.setCursor(0, 0);        //1st column, first row for inductance
    lcd.print("L=           ");
    lcd.setCursor(2, 0);        //3rd column, first row
    lcd.print(GLValue);
//    NewInductance=GetInductanceValue();
//    lcd.print(" ");
//    lcd.print(NewInductance);
//    lcd.print("nH");

    lcd.setCursor(0, 1);        //1st column, second row for capacitance
    lcd.print("C=           ");
    lcd.setCursor(2, 1);        //1st column, second row
    lcd.print(GCValue);
//    NewCapacitance=GetCapacitanceValue();
//    lcd.print(" ");
//    lcd.print(NewCapacitance);
//    lcd.print("pF");
  }
  GTuneChanged = false;
//
// finally display forward power and VSWR once per second
//  
  if (--GVoltDisplayCount == 0)
  {
    GVoltDisplayCount = 64;
    lcd.setCursor(16,3);                          // display power
    lcd.print("   ");
    lcd.setCursor(16,3);
    lcd.print(GForwardPower);
    lcd.setCursor(16,2);                          // display VSWR
    lcd.print("    ");
    lcd.setCursor(16,2);
    if (GForwardPower == 0)                       // if VSWR undefined
      strcpy(LocalStr, "-.-");
    else if ((GIntVSWR < 99) && (GIntVSWR > 0))
      mysprintf(LocalStr, GIntVSWR, true);
    else
      strcpy(LocalStr, "Hi");
    lcd.print(LocalStr);
  }
}


//
// set whether tuning or not
// true if tune is in progress
//
void LCU_UI_SetTuning(bool IsTuning)
{
#ifdef CONDITIONAL_LCD_UI
  
  if (!IsTuning)
  {
    lcd.setCursor(0, 2);              //1st column, third row
    lcd.print("finished");
    GLValue = GetInductance();        // get tune settings
    GCValue = GetCapacitance();
    IsHighZ = GetHiLoZ();
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
  lcd.setCursor(5, 0);        //5rd column, first row
  lcd.print(FreqString);
}

void ShowTune(bool IsTune)
{
  lcd.setCursor(14, 0);        //14th column, first row
  if(IsTune)
    lcd.print("T");
  else
    lcd.print("-");
}

void ShowAntenna(int Antenna)
{
  lcd.setCursor(16, 0);        //16th column, first row
  lcd.print(Antenna);
}

void ShowATUEnabled(bool IsEnabled)
{
  lcd.setCursor(18, 0);        //18th column, first row
  if(IsEnabled)
    lcd.print("E");
  else
    lcd.print("-");
}
