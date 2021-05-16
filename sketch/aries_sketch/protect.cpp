/////////////////////////////////////////////////////////////////////////
//
// Aries ATU controller sketch by Laurence Barker G8NJJ
// this sketch controls an L-match ATU network
// with a CAT interface to connect to an HPSDR control program
// copyright (c) Laurence Barker G8NJJ 2019
//
// the code is written for an Arduino Nano 33 IoT module
//
// protect.h: amplifier protection code
// this feature is detected by the initialisatino code finding the MCP23017 in thr I2C bus
/////////////////////////////////////////////////////////////////////////


#include <Arduino.h>
#include <Wire.h>
#include "protect.h"
#include "iopins.h"
#include "LCD_UI.h"

bool GProtectionPresent;            // becomes true of a device detected
bool GIsTripped;                    // true if the PA has been tripped
byte GTripInputBits;                // input bits with trip state 
                                    // bit0=VSWR; bit1=rev power; bit2=drive power; bit3=temp               

//
// defines for the MCP23017 address and registers within it
// MCP23017 operated with IOCON.BANK=0
//
#define VMCPADDR 0x20               // MCP23017 address in I2C
#define VGPIOAADDR 0x12             // GPIO A (band bits, flip flop reset, enable)
#define VGPIOBADDR 0x13             // GPIO B (trip conditions, band bit)
#define VOLATAADDR 0x14             // o/p latch A (flip flop reset, enable)
#define VIODIRAADDR 0x0             // direction A
#define VIODIRBADDR 0x1             // direction B
#define IOCONADDR 0x0A              // I/O control
#define VGPPUA 0x0C                 // pullup control for GPIO A
#define VGPPUB 0x0D                 // pullup control for GPIO B


//
// function to read 8 bit input from MCP23017
// returns register value
//
byte ReadMCPRegister(byte ChipAddress, byte Address)
{
  byte Input;                                   // becomes the new bit sequence for an input
  Wire.beginTransmission(ChipAddress);
  Wire.write(Address);                               // point to GPIOB register
  Wire.endTransmission();
  Wire.requestFrom(ChipAddress, 1);                  // read 1 byte
  Input=Wire.read();                                    // GPIOB
  return Input;
}


//
// function to write 8 bit value to MCP23017
// returns register value
//
void WriteMCPRegister(byte ChipAddress, byte Address, byte Value)
{
  Wire.beginTransmission(ChipAddress);
  Wire.write(Address);                                  // point to register
  Wire.write(Value);                                    // write its data
  Wire.endTransmission();
}



//
// function to cycle the FIFO reset pin to 1 then 0.
// note enable PTT will always be off
//
void CycleFlipFlopReset(void)
{
  WriteMCPRegister(VMCPADDR, VGPIOAADDR, 0x00);                 // PTT disable, no flip flop reset
  WriteMCPRegister(VMCPADDR, VGPIOAADDR, 0x01);                 // PTT disable, no flip flop reset
  WriteMCPRegister(VMCPADDR, VGPIOAADDR, 0x00);                 // PTT disable, no flip flop reset
}



//
// enable or disable the PA PTT
//
void SetPAPTTEnable(bool Enabled)
{
  byte State = 0;
  if(Enabled) 
    State = 0x02;
  WriteMCPRegister(VMCPADDR, VGPIOAADDR, State);                // PTT bit, no flip flop reset
}




//
// initialise protection
//
void InitProtection(void)
{
  byte Detect;
  
  Wire.beginTransmission(VMCPADDR);
  Detect = Wire.endTransmission();

  if(Detect == 0)
    GProtectionPresent = true;
  
  if(GProtectionPresent)
  {
    WriteMCPRegister(VMCPADDR, VIODIRAADDR, 0xFC);                // make bits 0, 1 output
    WriteMCPRegister(VMCPADDR, VIODIRBADDR, 0xFF);                // make all input
    WriteMCPRegister(VMCPADDR, VGPIOAADDR, 0x00);                 // PTT disable, no flip flop reset
    WriteMCPRegister(VMCPADDR, VGPPUA, 0xFF);                     // enable all pullups
    WriteMCPRegister(VMCPADDR, VGPPUB, 0xFF);                     // enable all pullups
    
    CycleFlipFlopReset();
    if(digitalRead(VPINPROTECTIONTRIP) == HIGH)                 // if still high, successfully reset
    {
      SetPAPTTEnable(true);                                     // turn PTT enable on
      GIsTripped = false;
    }
    else
      GIsTripped = true;                                        // set trip state
  }
}



//
// periodic tick handler
// red "tripped" input bit, and MCP words
// from the MCP words separate out the 4 trip conditions, and the "band" bits
//
void ProtectionTick(void)
{
  byte MCPByte;

  byte BandBits = 0;
  byte TripBits = 0;
  bool TripInput = false;

  if(digitalRead(VPINPROTECTIONTRIP) == LOW)
    TripInput = true;

  TripBits = ReadMCPRegister(VMCPADDR, VGPIOBADDR);
  BandBits = ReadMCPRegister(VMCPADDR, VGPIOAADDR);
//
// rearrange band bits so bit0= 6M, bit6 = 180M;
// strip the non band bits, then copy top bit from GPA7 into the word.
//
  BandBits &= 0b11111100;
  BandBits = BandBits >> 1;
  
  if(TripBits & 0b10000000)
    BandBits |= 0b1;                                          // copy input A7 to B0
   TripBits &= 0b00001111;
   TripBits ^= 0b00001111;                                    // 4 trip bits are active low; invert.
   BandBits ^= 0b01111111;                                    // 7 band bits are active low; invert.
   GTripInputBits = TripBits;                                 // make available to display
//
// now decide if there are any actions needed
// if tripped: send trip bits to display
// if not tripped state: if tripped input bit, change display, turn PTT off then set "tripped"
//
  if(TripInput)
  {
    if(GIsTripped == false)
    {
      SetPATrippedScreen(true);                               // change display
    }
    GIsTripped = true;                                        // set trip state
    SetPAPTTEnable(false);                                    // turn PTT enable off
  }
  else
  {
    DisplayBandBits(BandBits);
  }
}



//
// function called when display RESET is pressed
// cycle the reset flip flop; if not tripped after cycle, set "not tripped"
//
void TripResetPressed(void)
{
  CycleFlipFlopReset();
  if(digitalRead(VPINPROTECTIONTRIP) == HIGH)                 // if still high, successfully reset
  {
    SetPAPTTEnable(true);                                     // turn PTT enable on
    SetPATrippedScreen(false);                                // change display to normal
    GIsTripped = false;
  }
}
