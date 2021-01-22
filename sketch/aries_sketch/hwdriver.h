/////////////////////////////////////////////////////////////////////////
//
// Aries ATU controller sketch by Laurence Barker G8NJJ
// this sketch controls an L-match ATU network
// with a CAT interface to connect to an HPSDR control program
// copyright (c) Laurence Barker G8NJJ 2019
//
// the code is written for an Arduino Nano 33 IoT module
//
// hwdriver.h:  drives to mechanical relays
// using a serial shift register, accessed by SPUI
/////////////////////////////////////////////////////////////////////////
#ifndef __hwdriver_h
#define __hwdriver_h

#include <arduino.h>


extern bool GStandaloneMode;                       // true if ATU is in standalone mode
extern bool GResendSPI;                            // true if SPI data must be shifted again
extern bool GSPIShiftInProgress;                   // true if SPI shify is currently happening
extern unsigned int GVf, GVr;                      // forward and reverse voltages
extern float GVSWR;                                // calculated VSWR value
extern unsigned int GForwardPower;                 // forward power (W)



//
// function to initialise output
//
void InitialiseHardwareDrivers(void);


//
// Hardware driver tick
// read the ADC values
//
void HWDriverTick(void);


//
// functions to set antenna (numbered 1-3)
//
void SetAntennaSPI(int Antenna, bool IsRXAnt);


//
// functions to get and set inductance and capacitance
//
void SetInductance(byte Value);             // inductance 0-255
void SetCapacitance(byte Value);            // capacitance 0-255
void SetHiLoZ(bool Value);                  // true for high Z (relay=1)
byte GetInductance(void);                   // inductance 0-255
byte GetCapacitance(void);                  // capacitance 0-255
bool GetHiLoZ(void);                        // true for high Z (relay=1)

//
// set a null solution (essentially the same as "disable")
// this sets the values ready for a PTT press to send to hardware
//
void SetNullSolution(void);


//
// assert tuning solution to relays
// send settings to I2C
//
void DriveSolution(void);




//
// function to return mean and p-p excursion of Vf and Vr
///
// find peak power by searching buffer
// returns a power peak value
// 1st parameter true for forward power
//
unsigned int FindPeakPower(bool IsFwdPower);

//
// returns a power value
// 1st parameter true for forward power
//
unsigned int GetPowerReading(bool IsFwdPower);


//
// used to get a display value for the Nextion display
// 
void GetADCMeanAndPeak(bool IsVF, unsigned int* Mean, unsigned int* Peak);






#endif
