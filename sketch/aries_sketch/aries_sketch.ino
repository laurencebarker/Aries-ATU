/////////////////////////////////////////////////////////////////////////
//
// Aries ATU controller sketch by Laurence Barker G8NJJ
// this sketch controls an L-match ATU network
// with a CAT interface to connect to an HPSDR control program
// copyright (c) Laurence Barker G8NJJ 2019
//
// the code is written for an Arduino Nano 33 IoT module
//
// "main" file with setup() and loop()
// 2ms timer tick driven for real time operation
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
#include <ZeroTimer.h>


//
// global variables
//
// for heartbeat LED:
bool ledOn = false;                         // true if status LED is lit
byte Counter = 0;                           // tick counter for LED on/off

int GTickCounter;                           // tick counter for 16ms tick

bool GTickTriggered;                        // true if a 16ms tick has been triggered
int GAlgTickCount;

#define VALGTICKSPERSTEP 8
#define VMAINTICKSPERTIMERTICK 8            // 8 counts of 2ms per 16ms main tick


void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);               // PC communication
  while (!Serial)                   // wait for it to be ready 
  {
    ;
  }

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
  TC.startTimer(2000, TickHandler);             // 2ms tick: timer period is in microseconds

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
//
// initialise CAT handler
//
  InitCAT();
  InitCATHandler();
}


//
// ZeroTimer interrupt handler
// called every 2ms; invole main tick code every 8 ticks
// light touch encoder scan every 2ms, and do a different encoder each tick
//
void TickHandler(void)
{
  bool IsOdd = false;                           // call encoder tick with alternating odd/even setting
  if(GTickCounter &1)
    IsOdd = true;
  LCD_UI_EncoderTick(IsOdd);
//
// now count ticks to 16ms tick
//
  if(GTickCounter == 0)
  {
    GTickCounter = VMAINTICKSPERTIMERTICK-1;
    GTickTriggered = true;
  }
  else
    GTickCounter--;
}




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
        digitalWrite(VPINLED, HIGH); // Led on, off, on, off...
       else
        digitalWrite(VPINLED, LOW);
    }
    else
      Counter--;

//
// look for any CAT commands in the serial input buffer and process them
//    
    ScanParseSerial();

//
// read VSWR, then algorithm tick
// make very slow to start with!
//
if (--GAlgTickCount < 0)
{
  GAlgTickCount = VALGTICKSPERSTEP;
  HWDriverTick();
  AlgorithmTick();
}
else
  GAlgTickCount--;

//
// THETIS interface tick
//
  CatHandlerTick();
    
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
  pinMode(VPINPUSHBUTTONTUNE, INPUT_PULLUP);            // normal pushbutton
  pinMode(VPINRELAYLOHIZ, OUTPUT);                      // relay o/p
  pinMode(VPINSERIALLOAD, OUTPUT);                      // serial data latch o/p
  pinMode(VPINPTT, INPUT_PULLUP);                       // PTT input
  pinMode(VPINLED, OUTPUT);                             // status LED (normal D13 ise used for SPI)

  pinMode(VPINHWTUNECMD, INPUT_PULLUP);                 // hardwired TUNE command input
  pinMode(VPINFREQCOUNT, INPUT);                        // frequency count input, from prescaler
  pinMode (VPINCOUNTENABLE, OUTPUT);                    // count enable output
  
  
  digitalWrite(VPINRELAYLOHIZ, LOW);                    // deactivate relay

// PTT interrupt needs to catch both edges so we can send SPI data for T/R relay and RX/TX antenna
  attachInterrupt(VPINPTT, PttISR, CHANGE);
}
