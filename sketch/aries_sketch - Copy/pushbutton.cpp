/////////////////////////////////////////////////////////////////////////
//
// this file provides a pushbutton debounce
// copyright (c) Laurence Barker G8NJJ 2019
//
// the code is written for an Arduino Nano 33 IoT module
//
// pushbutton.cpp: code for PushbuttonDebounce class
/////////////////////////////////////////////////////////////////////////
 
#include "pushbutton.h" 

 
//
// constructor
//
PushbuttonDebounce::PushbuttonDebounce(byte BTN, byte TicksPerLongPress)
{
  pinBTN = BTN; 					// I/O pin
  TicksForLongpress = TicksPerLongPress;				// number of ticks needed for a long press
  TickCount = 0;

//
// check pin state and set initial pressed/unpressed state
//
  if(digitalRead(pinBTN) == LOW)				// pressed
    State = true;
  else
    State = false;
}
 

#define VEDGEMASK 0b00000111                                  // 3 most recent samples 
#define VPRESSPATTERN 0b00000100                              // 2 consecutive presses 
#define VRELEASEPATTERN 0b00000011                            // 2 consecutive releases 


//
// tick update: returns and event generated
// call this from a time with a period typically 10-20ms
//
 
EButtonEvent PushbuttonDebounce::Tick(void)
{ 
  byte PinState;					// input pin state
  EButtonEvent Event = eNone;

//
// read new pin state into the register holiding the last 8 states
//
  PinState = digitalRead(pinBTN);
  StateHistory = (StateHistory << 1);
  if (PinState == HIGH)
    StateHistory |= 1;

  if (State == false)				// button isn't currently pressed
  {
    if ((StateHistory  & VEDGEMASK) == VPRESSPATTERN)
    {
      State = true;
      Event = ePressed;
      if (TicksForLongpress != 0)
        TickCount = TicksForLongpress;
    }
  }
  else						// button is currently pressed
  {
    if ((StateHistory  & VEDGEMASK) == VRELEASEPATTERN)  // see if it is being released
    {
      State = false;
      Event = eReleased;
      TickCount = 0;
    }
//
// finally decrement long press count - declare event if we reach zero
//
    if (TickCount != 0)
      if (--TickCount == 0)
      {
        Event = eLongPressed;
      }
  }
  return Event;
} 
