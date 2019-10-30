/////////////////////////////////////////////////////////////////////////
//
// Aries ATU controller sketch by Laurence Barker G8NJJ
// this sketch controls an L-match ATU network
// with a CAT interface to connect to an HPSDR control program
// copyright (c) Laurence Barker G8NJJ 2019
//
// the code is written for an Arduino Nano Every module
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

byte GLValue, GCValue;        // user L, C settings
bool IsCoarse;
bool IsHighZ;                  // true for High Z
bool GTuneChanged;              // true if a tune operation has happened and display to be updated

NoClickEncoder Encoder1(VPINENCODER1B, VPINENCODER1A, VENCODERDIVISOR, true);
NoClickEncoder Encoder2(VPINENCODER2B, VPINENCODER2A, VENCODERDIVISOR, true);
LiquidCrystal_I2C lcd(0x27, 20, 4); // Addr, columns, rows
PushbuttonDebounce EncoderBtnDebounce(VPINENCODER1PB, 64);          // longpress to toggle low/high Z
PushbuttonDebounce TuneBtnDebounce(VPINPUSHBUTTONLOHIZ);        // no longpress



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
  lcd.setCursor(10,3);
  lcd.print("Low Z");
}



//
// periodic timer tick
//
void LCD_UI_Tick(void)
{
  EButtonEvent EncoderPressEvent;
  EButtonEvent TunePressEvent;
  signed char LMovement, CMovement;
  int NewInductance, NewCapacitance;
  
//
// update encoders and pushbuttons
//
  Encoder1.service();     // update encoder states
  Encoder2.service();
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
    InitiateQuickTune(true);
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
    lcd.setCursor(10,3);            // low/high Z setting
    if(IsHighZ)
      lcd.print("High Z ");
    else
      lcd.print("Low Z  ");
    
    lcd.setCursor(0, 0);        //1st column, first row for inductance
    lcd.print("L=           ");
    lcd.setCursor(2, 0);        //3rd column, first row
    lcd.print(GLValue);
    NewInductance=GetInductanceValue();
    lcd.print(" ");
    lcd.print(NewInductance);
    lcd.print("nH");

    lcd.setCursor(0, 1);        //1st column, second row for capacitance
    lcd.print("C=           ");
    lcd.setCursor(2, 1);        //1st column, second row
    lcd.print(GCValue);
    NewCapacitance=GetCapacitanceValue();
    lcd.print(" ");
    lcd.print(NewCapacitance);
    lcd.print("pF");
  }
  GTuneChanged = false;
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
