/////////////////////////////////////////////////////////////////////////
//
// Aries ATU controller sketch by Laurence Barker G8NJJ
// this sketch controls an L-match ATU network
// with a CAT interface to connect to an HPSDR control program
// copyright (c) Laurence Barker G8NJJ 2019
//
// the code is written for an Arduino Nano 33 IoT module
//
// hwdriver.cpp:  drives to mechanical relays
// using a serial shift register, accessed by SPUI
/////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <SPI.h>
#include "hwdriver.h"
#include "iopins.h"
#include "globalinclude.h"
#include "LCD_UI.h"
#include "cathandler.h"


//
// global variables
//
byte StoredLValue;                          // L=0-255
byte StoredCValue;                          // C=0-255
byte StoredAntennaRXTRValue;                // RX setting: TR (bit 0) ant select (bits 2:1)
byte StoredAntennaTXTRValue;                // TX setting: TR (bit 0) ant select (bits 2:1)
bool StoredHiLoZ;                           // true if Lo impedance
float GVSWR;                                // calculated VSWR value

bool GResendSPI;                            // true if SPI data must be shifted again
bool GSPIShiftInProgress;                   // true if SPI shify is currently happening

#define VVSWR_HIGH 100.0F                   // to avoid divide by zero
#define VLINEVOLTSCALE 0.02057F             // convert ADC reading to volts (3.3V VCC, 12 bit ADC)


//
// ADC averaging
// if enabled, average 16 samples for each reading
#define ENABLEADCAVERAGING 1


//
// function to initialise output
//
void InitialiseHardwareDrivers(void)
{
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
//
// change ADC registers to speed it up
//
  ADC->CTRLB.reg &= 0b1111100011111111;           // clear prescaler bits
  ADC->CTRLB.reg |= ADC_CTRLB_PRESCALER_DIV64;    // CK/64
  ADC->SAMPCTRL.reg = 0x0;                        // no settling time per successive approximation sample

  analogReadResolution(12);
//
// if ADC averaging enabled, make the ADC settings.
// set AVGCTRL.SAMPLENUM to 0x4 (16 samples)
// set CTRLB.RESSEL to 0x1 (averaging)
// set AVGCTRL.ADJRES to 0x4 
// see wiring_analog.c for details of register access
#ifdef ENABLEADCAVERAGING
  ADC->CTRLB.bit.RESSEL = 0x1;
  ADC->AVGCTRL.bit.SAMPLENUM = 0x4;
  ADC->AVGCTRL.bit.ADJRES = 0x4;
#endif
  while (ADC->STATUS.bit.SYNCBUSY == 1)           // same as sync_ADC()
    ;

  DriveSolution();
}




//
// functions to set antenna (numbered 1-3)
// the SPI driver will shift either StoredAntennaRXTR or StoredAntennaTXTR
// for RX antenna, need to drive its setting out if we are in RX mode
// for TX antenna, if already in TX we do NOT set it; gets set on next TX
//
void SetAntennaSPI(int Antenna, bool IsRXAnt)
{

  byte Ant = 0b000;                                         // clear bit 0 for antenna 1
  
  if (Antenna == 2)
    Ant = 0b011;                                            // set bit 1&0 for antenna 2
  else if (Antenna == 3)
    Ant = 0b101;                                            // set bit 2&0 for antenna 3

  if(IsRXAnt)
  {
    StoredAntennaRXTRValue = (StoredAntennaRXTRValue & 0b11111000);       // erase ond ant, and TX bit off
    StoredAntennaRXTRValue |= Ant;                                      // add in new ant
//
// either send new data to SPI, or queue a shift
// (this handles a race condition: just coming out of TX, already sending old RX antenna data when new RX antenna setting arrives)
// if PTT is pressed, RX ant will get set anyway when TX deasserted
//
    if ((GPTTPressed == false) && (!GSPIShiftInProgress))
      DriveSolution();
    else
      GResendSPI = true;
  }
  else
  {
    StoredAntennaTXTRValue = (StoredAntennaTXTRValue & 0b11111000);       // erase old ant, and TX bit off
    StoredAntennaTXTRValue |= Ant;                                      // add in new ant
  }
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
// set a null solution (essentially the same as "disable")
//
void SetNullSolution(void)
{
  StoredLValue = 0;
  StoredCValue = 0;
  StoredHiLoZ = false;
}

//
// Hardware driver tick
// read the ADC values
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

#ifdef CONDITIONAL_STREAM_ADCREADINGS
  if (FwdVoltReading > 200)
  {
    Serial.print("Vf=");
    Serial.print(FwdVoltReading);
    Serial.print(" Vr=");
    Serial.println(RevVoltReading);
  }
#endif
  
  Unit = VFwd * VFwd/50;                            // calculate power in 50 ohm line
  Power = int(Unit);
  SetPwr(Power);
//
// finally calculate VSWR
// GVSWR stored as float
//
  if (VFwd > VRev)
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
#define VHILOTARGET 0                             // 1 for min VSWR in high Z range; 0 for min VSWR in low Z range

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
// declare the data that will transfer by SPI to radio hardware
// byte 0: Antenna select and T/R relay
// byte 1: capacitors
// byte 2: inductors
// remember these get overwritten by incoming RX data whether we want it or not!
//
#define VNUMSHIFTBYTES 3
byte GSPIShiftSettings[VNUMSHIFTBYTES];


//
// assert tuning solution to relays
// send the current TR strobe, antenna, inductor and capacitor settings to SPI
// there is apotential race condition that could lead to a "resend"
// essentially this is just a big unidirectional shift register (we don't need the read reply)
// we want SPI mode 0, MSB first, unknown rate
//
void DriveSolution(void)
{
  bool NotDoneShift = true;                                   // force at least one shift

  GSPIShiftInProgress = true;
  while(NotDoneShift || GResendSPI)
  {
// copy in data
    if (GPTTPressed)
      GSPIShiftSettings[0] = StoredAntennaTXTRValue;
    else
      GSPIShiftSettings[0] = StoredAntennaRXTRValue;
    
    GSPIShiftSettings[1] = StoredCValue;
    GSPIShiftSettings[2] = StoredLValue;
    
    digitalWrite(VPINSERIALLOAD, LOW);         // be ready to give a rising edge after the transfer
    SPI.transfer(GSPIShiftSettings, VNUMSHIFTBYTES);
    digitalWrite(VPINSERIALLOAD, HIGH);         // rising edge to latch data after the transfer
//
// now see if we need to re-do
// on first pass through loop, allow GResendSPI to cause loop to run again
//
    if(NotDoneShift == false)
      GResendSPI = false;
    else
      NotDoneShift = false;
  }
  GSPIShiftInProgress = false;
  if(StoredHiLoZ == true)
    digitalWrite(VPINRELAYLOHIZ, HIGH);     // high Z
  else
    digitalWrite(VPINRELAYLOHIZ, LOW);     // low Z
}
