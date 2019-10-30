/////////////////////////////////////////////////////////////////////////
//
// this file provides a pushbutton debounce
// copyright (c) Laurence Barker G8NJJ 2019
//
// the code is written for an Arduino Nano Every module
//
// pushbutton.h: defines for PushbuttonDebounce class
/////////////////////////////////////////////////////////////////////////
 
#ifndef __PushbuttonDebounce_h
#define __PushbuttonDebounce_h

#include <Arduino.h>

    enum EButtonEvent
    { 
      eNone,
      ePressed, 
      eReleased, 
      eLongPressed 
    }; 

 
class PushbuttonDebounce
{ 
  public: 
  

//
// constructor: given button pin number, and the number of ticks to declare a long press (typically ~1second)
// longpress param = 0 if this is not needed
//
    PushbuttonDebounce(byte BTN, byte TicksPerLongPress = 0); 

//
// tick update: returns and event generated
// call this from a time with a period typically 10-20ms
//
    EButtonEvent Tick(void);   

// return true if button pressed, false if released
    bool GetPushbuttonState(void)
      {return State;}; 
 
  private: 
    byte pinBTN; 					// I/O pin
    byte TicksForLongpress;				// number of ticks needed for a long press
    bool State;						// true if pressed
    byte TickCount;					// current tick count
    byte StateHistory;					// history of states per tick - shifts in at LSB
}; 
 
 
// ---------------------------------------------------------------------------- 
#endif // __PushbuttonDebounce_h
