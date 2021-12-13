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
// using a serial shift register, accessed by SPI
/////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <SPI.h>
#include "hwdriver.h"
#include "iopins.h"
#include "globalinclude.h"
#include "LCD_UI.h"
#include "cathandler.h"
#include "protect.h"


//
// global variables
//
bool GStandaloneMode;                       // true if ATU is in standalone mode
byte StoredLValue;                          // L=0-255
byte StoredCValue;                          // C=0-255
byte StoredAntennaRXTRValue;                // RX setting: TR (bit 0) ant select (bits 2:1)
byte StoredAntennaTXTRValue;                // TX setting: TR (bit 0) ant select (bits 2:1)
bool StoredHiLoZ;                           // true if Lo impedance
float GVSWR;                                // calculated VSWR value
unsigned int GForwardPower;                 // forward power (W)

bool GResendSPI;                            // true if SPI data must be shifted again
bool GSPIShiftInProgress;                   // true if SPI shify is currently happening

#define VVSWR_HIGH 100.0F                   // to avoid divide by zero
float GADCScaleFactor;                        // ADC scaling factor (scales ADC reading to RMS line volts)

unsigned int GVf, GVr;                      // forward and reverse voltages (raw ADC measurements)

// variables for reading average and p-p ADC values
#define VNUMADCAVG 16
unsigned int GVFArray[VNUMADCAVG];          // circular buffer of Vf values
unsigned int GVRArray[VNUMADCAVG];          // circular buffer of Vr values
unsigned int GCircBufferPtr;                // current position to write to
unsigned int GPACurrent;                    // PA current in 100mA units (1 decimal point)


#define VHILOZBIT 0b00001000                // high low Z bit in SPI shift word


//
// scale factors from ADC reading to RMS line volts
// theses assume VSWR bridge has 14 turns
// one per display scale; resistor changes needed for each in h/w
// there is a spreadsheet in documentation folder to calculate!
//
const float GADCScaleValues[] = 
{
  0.019939,                                 // 100W: R12, R13=2K7
  0.028801,                                 // 200W: R12, R13=4K7
  0.044309,                                 // 500W: R12, R13=8K2
  0.061147,                                 // 1000W: R12, R13=12K
  0.087732                                  // 2000W: R12, R13=18K
};

#define VCURRENTSCALEFACTOR 0.051645        // to get current in 1/10A units (1dp)



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
  GCircBufferPtr = 0;
}



//
// function to set the ADC power scaling value depending on the display scale in use
// range 0-4
//
void SetADCScaleFactor(byte DisplayScale)
{
  if(DisplayScale > 4)                                        // clip to just 5 display scales
    DisplayScale = 4;
    
  GADCScaleFactor =  GADCScaleValues[DisplayScale];           // and lookup the value to use
}



//
// functions to set antenna (numbered 1-4, but 4 selects external relay 3)
// the SPI driver will shift either StoredAntennaRXTR or StoredAntennaTXTR
// for RX antenna, need to drive its setting out if we are in RX mode
// for TX antenna, if already in TX we do NOT set it; gets set on next TX
//
void SetAntennaSPI(int Antenna, bool IsRXAnt)
{

  byte Ant = 0b000;                                         // clear bit 0 for antenna 1
  byte OriginalRXAntennaWord;                               // value before call, to detect different
  
  if (Antenna == 2)
    Ant = 0b011;                                            // set bit 1&0 for antenna 2
  else if ((Antenna == 3) || (Antenna == 4))
    Ant = 0b101;                                            // set bit 2&0 for antenna 3 or 4

  if(IsRXAnt)
  {
    OriginalRXAntennaWord = StoredAntennaRXTRValue;
    StoredAntennaRXTRValue = Ant;                                        // add in new ant
    if(StoredAntennaRXTRValue != OriginalRXAntennaWord)                  // update hardware if changed
    {
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
  }
  else      // TX antenna
    StoredAntennaTXTRValue = Ant;                                      // add in new ant
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
// with rev 4 hardware a tiny inductance gave best VSWR across the whole HF band
//
void SetNullSolution(void)
{
  StoredLValue = 1;
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
  int DisplayVSWR;                                  // values for display
  int CurrentReading;
  
  FwdVoltReading = analogRead(VPINVSWR_FWD);        // read forward power sensor (actually line volts)
  RevVoltReading = analogRead(VPINVSWR_REV);        // read reverse power sensor (actually line volts)
  GVf = FwdVoltReading;
  GVr = RevVoltReading;

  if(GProtectionPresent)
  {
    CurrentReading = analogRead(VPINPACURRENT);       // read PA current sensor
    GPACurrent = CurrentReading * VCURRENTSCALEFACTOR;     // 1 dp fixed point
  }
//
// write values to circular buffer
// adjust pointer first
//
  if(++GCircBufferPtr >= VNUMADCAVG)
    GCircBufferPtr = 0;
  GVFArray[GCircBufferPtr] = FwdVoltReading;
  GVRArray[GCircBufferPtr] = RevVoltReading;
  
//
// convert the raw measurements to "normal" units
//
  VFwd = (float)FwdVoltReading * GADCScaleFactor;    // forward line RMS voltage
  VRev = (float)RevVoltReading * GADCScaleFactor;    // reverse line RMS voltage

#ifdef CONDITIONAL_STREAM_ADCREADINGS
  if (FwdVoltReading > 0)
  {
    Serial.print("Vf=");
    Serial.print(FwdVoltReading);
    Serial.print(" Vr=");
    Serial.println(RevVoltReading);
  }
#endif
  
  Unit = VFwd * VFwd/50;                            // calculate power in 50 ohm line
  GForwardPower = int(Unit);

//
// finally calculate VSWR
// GVSWR stored as float
// below 1W, just report 1
//
  if (Unit < 1.0)
    GVSWR = 1.0;
  else if (VFwd > VRev)
    GVSWR = (VFwd+VRev) / (VFwd - VRev);                 // VSWR
  else
    GVSWR = VVSWR_HIGH;                                  // unvalid result

  if (GVSWR > VVSWR_HIGH)                                // clip at impossibly high value
    GVSWR = VVSWR_HIGH;
}



//
// function to return mean and p-p excursion of Vf and Vr
// used to get a display value for the Nextion display
// 
void GetADCMeanAndPeak(bool IsVF, unsigned int* Mean, unsigned int* Peak)
{
  unsigned int* Ptr;                          // pointer to array
  byte Cntr;                                  // loop counter
  unsigned long Total;                        // total value found
  unsigned int Smallest, Largest;             // biggest and smallest found
  unsigned int Reading;

  Smallest = 65535;
  Largest = 0;
  Total = 0;

  if(IsVF)                                    // get array pointer                                     
    Ptr = GVFArray;
  else
    Ptr = GVRArray;

  for(Cntr=0; Cntr < VNUMADCAVG; Cntr++)
  {
    Reading = *Ptr++;
    Smallest = min(Smallest, Reading);
    Largest = max(Largest, Reading);
    Total += Reading;
  }
  *Mean = Total / VNUMADCAVG;                        // get mean
  *Peak = Largest - Smallest;
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
// there is a potential race condition that could lead to a "resend"
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
    if(StoredHiLoZ)
      GSPIShiftSettings[0] |= VHILOZBIT;
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
  
// on rev 4 and below hardware, drive out the high/low Z bit on DIG8
  if(HWVERSION <= 4)
  {
    if(StoredHiLoZ == true)
      digitalWrite(VPINRELAYLOHIZ, HIGH);     // high Z
    else
     digitalWrite(VPINRELAYLOHIZ, LOW);     // low Z
  }
}

//
// find peak power by searching buffer
// returns a power peak value
// parameter true for forward power
//
unsigned int FindPeakPower(bool IsFwdPower)
{
  unsigned int* Ptr;                          // pointer to array
  byte Cntr;                                  // loop counter
  unsigned int Largest = 0;                   // biggest and smallest found
  unsigned int Reading;
  float Volts, Power;

  if(IsFwdPower)                                    // get array pointer                                     
    Ptr = GVFArray;
  else
    Ptr = GVRArray;

  for(Cntr=0; Cntr < VNUMADCAVG; Cntr++)
  {
    Reading = *Ptr++;
    Largest = max(Largest, Reading);
  }
  Volts = (float)Largest * GADCScaleFactor;              // forward line RMS voltage
  Power = Volts * Volts/50;                             // calculate power in 50 ohm line
  return (unsigned int)Power;
}



//
// returns an average power value
// parameter true for forward power
//
unsigned int GetPowerReading(bool IsFwdPower)
{
  unsigned int* Ptr;                          // pointer to array
  byte Cntr;                                  // loop counter
  unsigned long Total = 0;                    // total value found
  unsigned int Reading;
  float Volts, Power;

//
// average the array of voltage readings
//
  if(IsFwdPower)                              // get array pointer                                     
    Ptr = GVFArray;
  else
    Ptr = GVRArray;

  for(Cntr=0; Cntr < VNUMADCAVG; Cntr++)
  {
    Reading = *Ptr++;
    Total += Reading;
  }
  Total = Total/VNUMADCAVG;                   // mean value
  Volts = (float)Total * GADCScaleFactor;    // forward line RMS voltage
  
  Power = Volts * Volts/50;                            // calculate power in 50 ohm line
  return (unsigned int)Power;
}
