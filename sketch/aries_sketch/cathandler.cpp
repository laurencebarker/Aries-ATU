/////////////////////////////////////////////////////////////////////////
//
// Aries ATU controller sketch by Laurence Barker G8NJJ
// this sketch controls an L-match ATU network
// with a CAT interface to connect to an HPSDR control program
// copyright (c) Laurence Barker G8NJJ 2019
//
// the code is written for an Arduino Nano 33 IoT module
//
// CAT handler.cpp
// this file holds the CAT handling code and THETIS interface
// responds to parsed messages, and initiates message sends
// this is the main body of the program!
/////////////////////////////////////////////////////////////////////////

#include "globalinclude.h"
#include "iopins.h"
#include <Arduino.h>
#include "cathandler.h"
#include <stdlib.h>
#include "LCD_UI.h"
#include "extEEPROM.h"
#include "hwdriver.h"
#include "algorithm.h"


//
// global variables
//
#define VMAXFREQUENCY 6149                      // 61490KHz, 61.49MHz
#define VNUMSOLUTIONS 6150                      // solutions held per antenna
#define VNUMSOLUTIONPAGES 145                   // no. 128 byte pages to hold solutions
#define VSOLUTIONSIZE 3                         // size in EEPROM of one stored solution
#define EEPAGESIZE 128
unsigned int GTunedFrequency10;                 // frequency from THETIS, in 10KHz resolution. 0 = DC
bool GATUEnabled;                               // true if the ATU is enabled
unsigned int GQueuedCATFrequency;               // frequency passed by tHETIS if TX was active.
byte GRXAntenna;                                // selected RX antenna (1-3; 0 if not set)
byte GTXAntenna;                                // selected TX antenna (1-3; 0 if not set)
bool GPCTuneActive;                             // true if TUNE is in progress as signalled by PC (note PTT will be detected first)
volatile bool GPTTPressed;                      // true if PTT pressed (ie TX active). Set by interrupt
unsigned int GSolutionStartFreq;                // start frequency for internal block of stored solutions
bool GTXAllowed;                                // true if ATU solution should be set to ATU when TX asserted
bool GATUIsTuned;                               // true if L/C settings are already set
bool GQueuedFrequencyChange;                    // true if new frequency sent, but tX was in progress

// buffer to hold a set of solutions for all HF frequencies for one antenna
// chosen to be an integer number of EEPROM pages, slightly larger than max size needed
//
byte SolutionBuffer[VNUMSOLUTIONPAGES*EEPAGESIZE];

//
// EEPROM access class
//
extEEPROMFast myEEPROM(kbits_1024, 1, 128, 0x50);         // size, number of EEPROMs, page size, I2C address

//
// variables for local search either side of "ideal" match
//
#define VNUMSERACHSIZE 11
int GLocalSearch[] = 
{0, -1, +1, -2, +2, -3, +3, -4, +4, -5, +5};



//
// initialise CATHandler
// most of this is crude, early debug!
//
void InitCATHandler(void)
{
//  byte WriteStatus, ReadStatus;
//  byte Cntr;
//  byte WriteData[]={255,255, 255, 255, 255};
//byte WriteData[]={0,1,2,3,4};
//  byte ReadData[16];

  
  byte i2cStat = myEEPROM.begin(myEEPROM.twiClock100kHz);
  if ( i2cStat != 0 ) 
  {
    Serial.println(F("I2C Problem"));
  }

//  Serial.println("Starting");
  
//  WriteStatus = myEEPROM.write(65536, WriteData, 5);

//  Serial.println("Reading address 0");
//  ReadStatus = myEEPROM.read(0, ReadData, 10);

//  for (Cntr=0; Cntr < 10; Cntr++)
//  {
//    Serial.print("addr: ");
//    Serial.print(Cntr);
//    Serial.print(" data = ");
//    Serial.println(ReadData[Cntr]);
//  }

//  Serial.println("Reading address 65536");
//  ReadStatus = myEEPROM.read(65536, ReadData, 10);

//  for (Cntr=0; Cntr < 10; Cntr++)
//  {
//    Serial.print("addr: ");
//    Serial.print(Cntr+65536);
//    Serial.print(" data = ");
//    Serial.println(ReadData[Cntr]);
//  }
  
}


//
// clip to numerical limits allowed for a given message type
//
int ClipParameter(int Param, ECATCommands Cmd)
{
  SCATCommands* StructPtr;

  StructPtr = GCATCommands + (int)Cmd;
//
// clip the parameter to the allowed numeric range
//
  if (Param > StructPtr->MaxParamValue)
    Param = StructPtr->MaxParamValue;
  else if (Param < StructPtr->MinParamValue)
    Param = StructPtr->MinParamValue;
  return Param;  
}



/////////////////////////////////  make CAT messages  /////////////////////////

//
// function to send back a software version message
//
void MakeSoftwareVersionMessage(void)
{
  long Version;
  Version = (PRODUCTID * 100000) + (HWVERSION*1000) + SWVERSION;
  
//  Version = SWVERSION;
  MakeCATMessageNumeric(eZZZS,Version);
}


//
// function to send back a tune success message
//
void MakeTuneSuccessMessage(bool Result)
{
  MakeCATMessageBool(eZZOX,Result);
}


//
// function to send back an "erase block" success message
//
void MakeEraseSuccessMessage(bool Result)
{
  MakeCATMessageBool(eZZOZ,Result);
}



//////////////////////// EEPROM access functions //////////////////////////////////////

//
// EEEraseSolutionSet: erase an entire set of solutions for one antenna from EEPROM
// this erases a significant block of EEPROM memory, and takes around 3 seconds.
//
void EEEraseSolutionSet(byte Antenna)
{
  unsigned int Counter;                               // counts blocks of solutions
  unsigned int StartAddress;                          // start address of Erase
  byte Buffer[EEPAGESIZE];                            // temp page buffer
  byte WriteStatus;                                   // error return value

//
// fill page buffer, then set EEPROM start address
//
  for(Counter=0; Counter < EEPAGESIZE; Counter++)
    Buffer[Counter] = 0xFF;                           // set to same as uninitialised EEPROM
    
  if(Antenna == 0)                                    // get start address in EEPROM
    StartAddress = 0;
  else
    StartAddress = 32768*(Antenna-1);

//
// loop through all solution space and erase - process blocks of EEPROM page size
//
  byte i2cStat = myEEPROM.begin(myEEPROM.twiClock400kHz);

  for(Counter=0; Counter < VNUMSOLUTIONPAGES; Counter++)
  {
    WriteStatus = myEEPROM.write(StartAddress, Buffer, EEPAGESIZE);
    StartAddress += EEPAGESIZE;
  }
}


//
// read out a block of solutions from EEPROM into RAM for one antenna
// this actually reads all solutions for that antenna
// can use a single function call
//
void GetSolutionBlock(byte Antenna)
{
  byte ReadStatus;
  unsigned int StartAddress;                          // start address of Erase
//
// get EEPROM start address
//
  if(Antenna == 0)                                    // get start address in EEPROM
    StartAddress = 0;
  else
    StartAddress = 32768*(Antenna-1);
  
  byte i2cStat = myEEPROM.begin(myEEPROM.twiClock400kHz);
  ReadStatus = myEEPROM.read(StartAddress, SolutionBuffer, VNUMSOLUTIONS * VSOLUTIONSIZE);
}



//
// write solution for current antenna and frequency to EEPROM
// this is done before we declare TUNE complete
//
void SetTuneResult(bool Successful, byte Inductance, byte Capacitance, bool IsHighZ)
{
  byte WriteStatus;
  byte WriteBuffer[VSOLUTIONSIZE];                    // data to write  
  unsigned int StartAddress;                          // start address of solution to write back
  int Cntr;

//
// get EEPROM start address
//
  if(GTXAntenna == 0)
    StartAddress = 0;
  else
    StartAddress = 32768*(GTXAntenna-1);
  StartAddress += VSOLUTIONSIZE * GTunedFrequency10;

//
// get write data
//
  if(Successful)
  {
    WriteBuffer[0] = 0b1;                             // data is available
    if(IsHighZ)
      WriteBuffer[0] |= 0x80;                          // sert top bit if high Z
    WriteBuffer[1] = Inductance;
    WriteBuffer[2] = Capacitance;
  }
  else
  {
    for(Cntr=0; Cntr < VSOLUTIONSIZE; Cntr++)
      WriteBuffer[Cntr] = 0xFF;
  }
  byte i2cStat = myEEPROM.begin(myEEPROM.twiClock400kHz);
  WriteStatus = myEEPROM.write(StartAddress, WriteBuffer, VSOLUTIONSIZE);
  MakeTuneSuccessMessage(Successful);               // send message back to PC software
}



///////////////////////////////// process CAT commands ///////////////////////

//
// handle a frequency change message
// search locally (0 to +/- 50KHz) to find a solution
//
void SetupForNewFrequency(void)
{
  byte Solution1, Solution2, Solution3;               // 3 solution bytes  
  int StartAddress;                                   // start address of Erase
  int TestAddress;                                    // address we are testing to see if it has a solution
  int Cntr;
  bool SolutionFound = false;                         // true if we get a hit
  bool IsHighZ = false;
  
//
// get memory start address
// (this is the same as the offset into the EEPROM block 1, 2 or 3)
//
  StartAddress = VSOLUTIONSIZE * GTunedFrequency10;
//
// now look for an exact or near solution
// this starts at the exact frequency then steps outp looking for a solution
//
  for(Cntr=0; Cntr < VNUMSERACHSIZE; Cntr++)
  {
    TestAddress = StartAddress + GLocalSearch[Cntr];
    if((TestAddress >= 0) && (TestAddress < VMAXFREQUENCY))
    {
      Solution1 = SolutionBuffer[TestAddress++];          // read the 3 solution bytres; then see if valid
      Solution2 = SolutionBuffer[TestAddress++];
      Solution3 = SolutionBuffer[TestAddress];
      if((Solution1 & 0b00000001) == 0)                   // if bottom bit set
      {
        SolutionFound = true;                             // we have found a match
        break;
      }
    }
    // if we have a solutino, set it and send success; else set bypass
    if(SolutionFound)
    {
      if(Solution1 & 0b10000000)
        IsHighZ = true;
      SetInductance(Solution2);             // inductance 0-255
      SetCapacitance(Solution3);            // capacitance 0-255
      SetHiLoZ(IsHighZ);                    // true for low Z (relay=1)
    }
    else
      SetNullSolution();
    MakeTuneSuccessMessage(SolutionFound);
  }
}



//
// handle an antenna change message
//
void SetupForNewAntenna(void)
{
  SetNullSolution();                          // temporarily set L, C values for ATU out of circuit
  GetSolutionBlock(GTXAntenna);               // read EEPROM block for whole antenna
  SetupForNewFrequency();                     // then find if we have a solution
}




//////////////////////////////// handle CAT RX messages ///////////////////////

//
// handle TUNE on/off CAT message from PC
//
void SetTuneOnOff(bool State)
{
  GPCTuneActive = State;
  if(State && GPTTPressed)
    InitiateQuickTune(true);
  ShowTune(State);
}



//
// handle ATU on/off CAT message from PC
//
void SetATUOnOff(bool State)
{
  GATUEnabled = State;
  if(GATUEnabled)
    SetupForNewAntenna();                       // get new settings
  else
    SetNullSolution();                          // set L, C values for ATU out of circuit
  ShowATUEnabled(State);                        // debug
}



//
// handle frequency change CAT message from PC
// this arrives as a string, not an int
// strip the last 4 digits then extract value in 10KHz CHUNKS
//
void SetNewFrequency(char* FreqString)
{
  unsigned long GCatFrequency_10KHz;
  byte Length;

  Length=strlen(FreqString);                                              // length of passed string should be 11 (Hz resolution)
  FreqString[Length-4]=0;                                                 // make 3 chars shorter (resolution 10KHz)
  GTunedFrequency10 = atol(FreqString);
//
// now see if we have a solution
//
  if(GPTTPressed)
    GQueuedFrequencyChange=true;
  else
    SetupForNewFrequency();
//
// set the frequency the algorithm should use
//
  FindFreqRow(GTunedFrequency10/100);                                     // set algorithm frequency, in units of 1MHz
}


//
// handle a change of RX antenna CAT command from PC
// simply set and drive it out
//
void HandleRXAntennaChange(int Antenna)
{
  if((Antenna >= 1) && (Antenna <= 3))
  {
      GRXAntenna = Antenna;
      SetAntennaSPI(Antenna, true);                                       // send new antenna setting to SPI
  }
}


//
// handle a change of TX antenna CAT command from PC
// if different, re-load EEPROM then search for solution
//
void HandleTXAntennaChange(int Antenna)
{
  if((Antenna >= 1) && (Antenna <= 3))
  {
    if(Antenna != GTXAntenna)
    {
      GTXAntenna = Antenna;
      SetAntennaSPI(Antenna, false);                                      // send new antenna setting to SPI
      SetupForNewAntenna();                                               // find solution etc
    }
    
  }
#ifdef CONDITIONAL_LCD_UI
    ShowAntenna(Antenna);
#endif
}


//
// handle an erase solution CAT command from PC
// this is non real time and message handling will stall till complete
//
void HandleEraseSolutions(int Antenna)
{
  EEEraseSolutionSet(Antenna);
  if(Antenna == GTXAntenna)                                             // if current antenna, erase RAM too
  {
    SetupForNewAntenna();                                               // find solution etc
  }
  MakeEraseSuccessMessage(true);
}


//
// handle an L/C fine tune CAT command from PC
//
void HandleLCFineTune(int Command)
{
//  Serial.print("L/C fine tune = ");
//  Serial.println(Command);
}



//
// handle CAT commands with numerical parameters
//
void HandleCATCommandNumParam(ECATCommands MatchedCAT, int ParsedParam)
{
  int Device;
  byte Param;
  bool State = false;
  
  switch(MatchedCAT)
  {
    case eZZOA:                                                       // RX Antenna change
      HandleRXAntennaChange(ParsedParam);
      break;

    case eZZOC:                                                       // TX Antenna change
      HandleTXAntennaChange(ParsedParam);
      break;

    case eZZOZ:                                                       // erase tuning solutions
      HandleEraseSolutions(ParsedParam);
      break;

    case eZZZE:                                                       // fine tune L/C
      HandleLCFineTune(ParsedParam);
      break;
  }
}


//
// handle CAT commands with no parameters
//
void HandleCATCommandNoParam(ECATCommands MatchedCAT)
{
  switch(MatchedCAT)
  {
    case eZZZS:                                                       // s/w version reply
      MakeSoftwareVersionMessage();
      break;
  }
}


//
// handle CAT commands with boolean parameter
//
void HandleCATCommandBoolParam(ECATCommands MatchedCAT, bool ParsedBool)
{
  switch(MatchedCAT)
  {
    case eZZTU:                                                       // TUNE on/off
      SetTuneOnOff(ParsedBool);
      break;

    case eZZOV:                                                       // ATU on/off
      SetATUOnOff(ParsedBool);
      break;
  }
}


//
// handle CAT commands with string parameter
//
void HandleCATCommandStringParam(ECATCommands MatchedCAT, char* ParsedParam)
{
  switch(MatchedCAT)
  {
    case eZZTV:                                                       // frequency change message
      SetNewFrequency(ParsedParam);
      break;
  }
}



//
// PTT ISR handler
// this triggers on both edges, so we can set RX and TX antenna as well as ATU tune solution
// we assume PTT to come from an FPGA source, so no bounce
//
void PttISR(void)
{
  if(digitalRead(VPINPTT) == LOW)
    GPTTPressed = true;                                      // got TX/RX state from I/O pin
  else
    GPTTPressed = false;                                     // got TX/RX state from I/O pin
  
//
// either send new data to SPI, or queue a shift
//  
  if(GSPIShiftInProgress)
    GResendSPI = true;
  else
    DriveSolution();                                         // drive SPI. This sets T/R state AND sends L/C data
//
// if Tune command already sent via CAT, initiate tune
// (there is a race condition and it's unclear whether the CAT tune command or PTT would happen first)
//
  if(GPTTPressed)
  {
    if(GPCTuneActive == true)
      InitiateQuickTune(true);
  }
  else
  {
    GPCTuneActive = false;                                    // cancel CAT tune
    if(GTuneActive)                                           // if algorithm still running, cancel it
      CancelAlgorithm();
  }
}



//
// 16ms tick
//
void CatHandlerTick()
{
  // see if we have a queuesd frequency change, while PTT was pressed; handle when not pressed
  if((!GPTTPressed) && (GQueuedFrequencyChange))
  {
    GQueuedFrequencyChange=false;
    SetupForNewFrequency();
  }
  
}
