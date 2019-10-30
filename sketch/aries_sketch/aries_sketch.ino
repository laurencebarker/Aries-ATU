/////////////////////////////////////////////////////////////////////////
//
// Aries ATU controller sketch by Laurence Barker G8NJJ
// this sketch controls an L-match ATU network
// with a CAT interface to connect to an HPSDR control program
// copyright (c) Laurence Barker G8NJJ 2019
//
// the code is written for an Arduino Nano Every module
//
// "main" file with setup() and loop()
/////////////////////////////////////////////////////////////////////////
//
#include <Arduino.h>
#include <Wire.h>
#include "globalinclude.h"
#include "iopins.h"
#include "LCD_UI.h"
#include "hwdriver.h"
#include "algorithm.h"
#include "tiger.h"
#include "cathandler.h"


//
// global variables
//
bool GTickTriggered;                  // true if a 16ms tick has been triggered

void SetupTimerForInterrupt(int Milliseconds)
{
  int Count;

  Count = Milliseconds * 250;
  TCB0.CTRLB = TCB_CNTMODE_INT_gc; // Use timer compare mode  
  TCB0.CCMP = Count; // Value to compare with. This is 1/5th of the tick rate, so 5 Hz
  TCB0.INTCTRL = TCB_CAPT_bm; // Enable the interrupt
  TCB0.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm; // Use Timer A as clock, enable timer
}


void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);                 // PC communication
  Wire.begin();                       // I2C
  Wire.setClock(400000);

  delay(1000);
//
// configure I/O pins
//
  ConfigIOPins();

//
// initialise timer to give 2ms tick interrupt
//
  SetupTimerForInterrupt(16);

//
// initialise hardware drivers (relays etc)
//
  InitialiseHardwareDrivers();

//
// initialise UI
//
#ifdef CONDITIONAL_LCD_UI
  LCD_UI_Initialise();
#endif

//
// initialise algorithm
//
  InitialiseAlgorithm();
  FindFreqRow(54);
  FindFreqRow(1);
//
// initialise CAT handler
//
  InitCAT();

}



//
// periodic timer tick handler.
//
ISR(TCB0_INT_vect)
{
  GTickTriggered = true;
   // Clear interrupt flag
   TCB0.INTFLAGS = TCB_CAPT_bm;
}



// for heartbeat LED:
bool ledOn = false;
byte Counter = 0;



//
// 16 ms event loop
// this is triggered by GTickTriggered being set by a timer interrupt
// the loop simply waits until released by the timer handler
void loop()
{
  while (GTickTriggered)
  {
    GTickTriggered = false;
// heartbeat LED
    if (Counter == 0)
    {
      Counter=31;
      ledOn = !ledOn;
      if (ledOn)
        digitalWrite(LED_BUILTIN, HIGH); // Led on, off, on, off...
       else
        digitalWrite(LED_BUILTIN, LOW);
    }
    else
      Counter--;

//
// look for any CAT commands in the serial input buffer and process them
//    
    ScanParseSerial();

//
// algorithm tick
//
    AlgorithmTick();
    
//
// UI tick, if conditionally included
//
#ifdef CONDITIONAL_LCD_UI
    LCD_UI_Tick();
#endif

  
  
  }
}



//
// set pinmode for all I/O pins used
// and write any output initial states
//
void ConfigIOPins(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(VPINENCODER1A, INPUT_PULLUP);                 // normal encoder
  pinMode(VPINENCODER1B, INPUT_PULLUP);                 // normal encoder

  pinMode(VPINENCODER2A, INPUT_PULLUP);                 // normal encoder
  pinMode(VPINENCODER2B, INPUT_PULLUP);                 // normal encoder

  pinMode(VPINENCODER1PB, INPUT_PULLUP);                // normal pushbutton
  pinMode(VPINPUSHBUTTONLOHIZ, INPUT_PULLUP);           // normal pushbutton
  pinMode(VPINRELAYLOHIZ, OUTPUT);                      // relay o/p
  pinMode(VPINPTT, INPUT_PULLUP);                       // PTT input
  digitalWrite(VPINRELAYLOHIZ, LOW);                    // deactivate relay

}
