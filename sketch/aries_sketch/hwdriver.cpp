/////////////////////////////////////////////////////////////////////////
//
// Aries ATU controller sketch by Laurence Barker G8NJJ
// this sketch controls an L-match ATU network
// with a CAT interface to connect to an HPSDR control program
// copyright (c) Laurence Barker G8NJJ 2019
//
// the code is written for an Arduino Nano Every module
//
// hwdriver.cpp:  drives to mechanical relays
/////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <Wire.h>
#include "hwdriver.h"
#include "iopins.h"
#include "globalinclude.h"
#include "LCD_UI.h"


//
// global variables
//
byte StoredLValue;                          // L=0-255
byte StoredCValue;                          // C=0-255
bool StoredHiLoZ;                           // true if Lo impedance
#define VVSWR_HIGH 100.0F                   // to avoid divide by zero
#define VLINEVOLTSCALE 0.1074F              // convert ADC reading to volts
float GVSWR;                                // calculated VSWR value


//
// defines for the MCP23017 address and registers within it
// MCP23017 operated with IOCON.BANK=0
//
#define VMCPRELAYADDR 0x20          // MCP23017 address in I2C
#define VGPIOAADDR 0x12             // GPIO A (column and LED out)
#define VGPIOBADDR 0x13             // GPIO B (row input)
#define VIODIRAADDR 0x0             // direction A
#define VIODIRBADDR 0x1             // direction B
#define VGPPUA 0x0C                 // pullup control for GPIO A
#define VGPPUB 0x0D                 // pullup control for GPIO B


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
// function to initialise output
//
void InitialiseHardwareDrivers(void)
{
  WriteMCPRegister(VMCPRELAYADDR, VIODIRAADDR, 0x00);                // make Direction register A = 00 (all output)
  WriteMCPRegister(VMCPRELAYADDR, VIODIRBADDR, 0x00);                // make Direction register A = 00 (all output)
  DriveSolution();
}


//
// functions to get and set inductance and capacitance
// these just store the value, ready to be sent to the relays on demand
//
void SetInductance(byte Value)              // inductance 0-255
{
  StoredLValue = Value;
}

void SetCapacitance(byte Value)             // capacitance 0-255
{
  StoredCValue = Value;
}

void SetHiLoZ(bool Value)                   // true for high Z (relay=1)
{
  StoredHiLoZ = Value;
}

byte GetInductance(void)                    // inductance 0-255
{
  return StoredLValue;
}

byte GetCapacitance(void)                   // capacitance 0-255
{
  return StoredCValue;
}

bool GetHiLoZ(void)                         // true for low Z (relay=1)
{
  return StoredHiLoZ;
}


//
// Hardware driver tick
// read the aDC values
//
void HWDriverTick(void)
{
  int FwdVoltReading, RevVoltReading;               // raw ADC samples
  float Unit;                                       // converted measurement
  float VFwd, VRev;                                 // forward, reverse line voltages
  int Power, DisplayVSWR;                           // values for display
  
  FwdVoltReading = analogRead(VPINVSWR_FWD);        // read forward power sensor (actually line volts)
  RevVoltReading = analogRead(VPINVSWR_REV);        // read reverse power sensor (actually line volts)
  
//
// convert the raw measurements to "normal" units
//
  VFwd = (float)FwdVoltReading * VLINEVOLTSCALE;    // forward line RMS voltage
  VRev = (float)RevVoltReading * VLINEVOLTSCALE;    // reverse line RMS voltage

  Unit = VFwd * VFwd/50;                            // calculate power in 50 ohm line
  Power = int(Unit);
  SetPwr(Power);
//
// finally calculate VSWR
// GVSWR stored as float
//
  if (VFwd != VRev)
    GVSWR = (VFwd+VRev) / (VFwd - VRev);                 // VSWR
  else
    GVSWR = VVSWR_HIGH;                                  // unvalid result

  if (GVSWR > VVSWR_HIGH)                                // clip at impossibly high value
    GVSWR = VVSWR_HIGH;
    
  DisplayVSWR = (int)(GVSWR * 10.0);                     // 1 decimal place int 
  SetVSWR(DisplayVSWR);
}




// 
// function to return VSWR value
// returns 100*VSWR; clipped to 65535
//
int GetVSWR(void)
{
  int VSWR;

  VSWR =  int(GVSWR * 100.0);                    

//
// finally optional debug code: calculate a simulated VSWR value with a minimum at (VLTARGET, VCTARGET)  
// this can calculate a "noise free" VSWR value in 2 ways.
// if algorithm is searching the desired Low or High Z region, calculated a min at defined L&C value
// if algorithm is searching the wrong region, give a minimum at (0,0) that just keeps getting bigger
// cloose the desired low/high Z region with VHILOTARGET
//
#ifdef CONDITIONAL_ALG_SIMVSWR
#define VSWRMIN 140                               // 2.0
#define VLTARGET 27                               // L value we should get the minimum at
#define VCTARGET 59                               // C value we should get the minimum at
#define VLSLOPE 5
#define VCSLOPE 8
#define VHILOTARGET 1                             // 1 for min VSWR in high Z range; 0 for min VSWR in low Z range

  if((StoredHiLoZ && (VHILOTARGET == 1)) || (!StoredHiLoZ && (VHILOTARGET==0)))                   // if we are looking in the right region 
    VSWR = VSWRMIN + VLSLOPE*abs(StoredLValue - VLTARGET) + VCSLOPE*abs(StoredCValue - VCTARGET);
  else
    VSWR = 10*VSWRMIN+VLSLOPE*StoredLValue + VCSLOPE * StoredCValue;
#endif
  
  return VSWR;
}



//
// tables of inductance and capacitance per relay bit (bit 0 first)
// capacitance in pF; inductance in nH
// these tables are for the AT11
//
int GCapacitorValues[] = 
{
  10, 20, 39, 82, 150, 330, 680, 1360 
};

int GInductorValues[] = 
{
  40, 80, 160, 320, 640, 1280, 2560, 5120
};


//
// functions to get the current inductance and capacitance, in nH and pF
//
int GetCapacitanceValue(void)
{
  byte Setting;                               // current binary value
  byte Bit;                                   // bit counter
  int Result = 0;

  Setting = StoredCValue;                     // get current binary setting
  for(Bit=0; Bit<8; Bit++)
  {
    if(Setting & 1)                           // if bottom bit set, add in table value
      Result += GCapacitorValues[Bit];
    Setting = Setting >> 1;                   // shift right to text next bit
  }
  return Result;  
}

int GetInductanceValue(void)
{
  byte Setting;                               // current binary value
  byte Bit;                                   // bit counter
  int Result = 0;

  Setting = StoredLValue;                     // get current binary setting
  for(Bit=0; Bit<8; Bit++)
  {
    if(Setting & 1)                           // if bottom bit set, add in table value
      Result += GInductorValues[Bit];
    Setting = Setting >> 1;                   // shift right to text next bit
  }
  return Result;  
}






//
// assert tuning solution to relays
// send settings to I2C
// active low relays for ebay relay driver board
//
void DriveSolution(void)
{
  Wire.beginTransmission(VMCPRELAYADDR);
  Wire.write(VGPIOAADDR);                                  // point to GPIOA register
  Wire.write(StoredCValue^0xFF);                                // write its data
  Wire.write(StoredLValue^0xFF);                                // write GPIOB data
  Wire.endTransmission();

  if(StoredHiLoZ == true)
    digitalWrite(VPINRELAYLOHIZ, HIGH);     // high Z
  else
    digitalWrite(VPINRELAYLOHIZ, LOW);     // low Z
}
