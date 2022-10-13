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


#define VEEDISPLAYPAGELOC 0x1FFF0L
#define VEEPEAKLOC 0x1FFF1L
#define VEEENABLEDLOC 0x1FFF2L
#define VEEDISPLAYSCALELOC 0x1FFF3L
#define VEEALLOWQUICKLOC 0x1FFF4L




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
volatile bool GTuneHWPressed;                   // true if tune strobe active (ie tune request). Set by interrupt
unsigned int GSolutionStartFreq;                // start frequency for internal block of stored solutions
bool GTXAllowed;                                // true if ATU solution should be set to ATU when TX asserted
bool GATUIsTuned;                               // true if L/C settings are already set
bool GQueuedFrequencyChange;                    // true if new frequency sent, but tX was in progress
byte GPTTReleaseCount;                          // PTT release counter, for debouncing
byte GTuneHWReleaseCount;                       // hardwired tune strobe release counter
bool GValidSolution;                            // true if a valid tune solution found
unsigned int GFreqPollTicks;                    // period in ticks until next frequency poll


#define VFULLTUNEFREQ 1000                      // freq (10KHz units) above which we always full tune
#define VFREQPOLLINTERVAL 312                   // units of ticks. 5s between freq polls

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
#define VNUMSEARCHSIZE 11
int GLocalSearch[] = 
{0, -1, +1, -2, +2, -3, +3, -4, +4, -5, +5};





//
// initialise CATHandler
// most of this is crude, early debug!
//
void InitCATHandler(void)
{
  byte i2cStat = myEEPROM.begin(myEEPROM.twiClock400kHz);
//  if ( i2cStat != 0 ) 
//  {
//    Serial.println(F("I2C Problem"));
//  }

// initialise algorithm operation: select whether quick tune always allowed

  if (GStandaloneMode)                        // for standalone mode, load "enabled" state from EEPROM
  {
    GATUEnabled = EEReadEnabled();
    GQuickTuneEnabled = EEReadQuick();
  }
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
  if(GATUEnabled)
    MakeCATMessageBool(eZZOX,Result);
  GValidSolution = Result;
  if(Result)                                      // set "tuned" LED lit if good
    digitalWrite(VPINTUNELED, HIGH);
  else
    digitalWrite(VPINTUNELED, LOW);


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
  unsigned int RAMStartAddress;                       // start address of solution to write back
  unsigned int EEPROMStartAddress;                    // start address of solution to write back
  int Cntr;

  GPCTuneActive = false;                              // set tune not active
//
// get RAM and EEPROM start address
//
  RAMStartAddress = VSOLUTIONSIZE * GTunedFrequency10;
  if(GTXAntenna == 0)
    EEPROMStartAddress = RAMStartAddress;
  else
    EEPROMStartAddress = RAMStartAddress + 32768*(GTXAntenna-1);

//
// get write data
//
  if(Successful)
  {
    WriteBuffer[0] = 0;                                 // data is available signalled by bottom bit = 0
    if(IsHighZ)
      WriteBuffer[0] |= 0x80;                           // sert top bit if high Z
    WriteBuffer[1] = Inductance;
    WriteBuffer[2] = Capacitance;
  }
  else
  {
    for(Cntr=0; Cntr < VSOLUTIONSIZE; Cntr++)
      WriteBuffer[Cntr] = 0xFF;
  }
//
// now write data to EEPROM and write it to RAM copy in SolutionBuffer
//
  for(Cntr=0; Cntr < VSOLUTIONSIZE; Cntr++)
    SolutionBuffer[Cntr+RAMStartAddress] = WriteBuffer[Cntr];

  byte i2cStat = myEEPROM.begin(myEEPROM.twiClock400kHz);
  WriteStatus = myEEPROM.write(EEPROMStartAddress, WriteBuffer, VSOLUTIONSIZE);

  MakeTuneSuccessMessage(Successful);               // send message back to PC software
}



//
// function to write new LCD display page
//
void EEWritePage(byte Value)
{
  myEEPROM.write(VEEDISPLAYPAGELOC, Value);
}

//
// function to read new LCD display page
//
byte EEReadPage(void)
{
  byte Result;
  Result = myEEPROM.read(VEEDISPLAYPAGELOC);
  if((Result == 0) || (Result > VNUMPAGES))
    Result = VNUMPAGES;
  return (byte)Result;
}


//
// function to write new display average/peak mode
//
void EEWritePeak(bool Value)
{
  byte Data;

  Data = (byte)Value;
  myEEPROM.write(VEEPEAKLOC, Data);
}

//
// function to read new display average/peak mode
//
bool EEReadPeak(void)
{
  byte Result;
  Result = myEEPROM.read(VEEPEAKLOC);
  return (bool)Result;
}


//
// function to write new ATU enbled/disabled for standalone mode
//
void EEWriteEnabled(bool Value)
{
  byte Data;

  Data = (byte)Value;
  myEEPROM.write(VEEENABLEDLOC, Data);
}

//
// function to read new ATU enbled/disabled for standalone mode
//
bool EEReadEnabled(void)
{
  byte Result;
  Result = myEEPROM.read(VEEENABLEDLOC);
  return (bool)Result;
}

//
// function to write, read new ATU display scale for standalone mode
//
void EEWriteScale(byte Value)
{
  myEEPROM.write(VEEDISPLAYSCALELOC, Value);
}


byte EEReadScale(void)
{
  byte Result;
  Result = myEEPROM.read(VEEDISPLAYSCALELOC);
  if(Result > VDISPLAYSCALE)
    Result = VDISPLAYSCALE;
  return (byte)Result;
}

//
// function to write, read new ATU quick tune enabled/disabled for standalone mode
//
void EEWriteQuick(bool Value)
{
  byte Data;
  Data = (byte)Value;
  myEEPROM.write(VEEALLOWQUICKLOC, Data);
}

bool EEReadQuick()
{
  byte Result;
  Result = myEEPROM.read(VEEALLOWQUICKLOC);
  return (bool)Result;
}



///////////////////////////////// process CAT commands ///////////////////////

//
// handle a frequency change message
// search locally (0 to +/- 50KHz) to find a solution
//
void SetupForNewFrequency(void)
{
  byte Solution1, Solution2, Solution3;               // 3 solution bytes  
  int TestAddress;                                    // address we are testing to see if it has a solution
  int TestFrequency;                                  // frequency value we are testing for
  int Cntr;
  bool SolutionFound = false;                         // true if we get a hit
  bool IsHighZ = false;
  

//
// now look for an exact or near solution
// this starts at the exact frequency then steps out looking for a solution
//
  for(Cntr=0; Cntr < VNUMSEARCHSIZE; Cntr++)
  {
    TestFrequency = GTunedFrequency10 + GLocalSearch[Cntr];     // start, +1, -1, +2, -2...
    if((TestFrequency >= 0) && (TestFrequency < VMAXFREQUENCY))
    {
//
// get memory start address
// (this is the same as the offset into the EEPROM block 1, 2 or 3)
//
      TestAddress = VSOLUTIONSIZE * TestFrequency;
      Solution1 = SolutionBuffer[TestAddress++];          // read the 3 solution bytes; then see if valid
      Solution2 = SolutionBuffer[TestAddress++];
      Solution3 = SolutionBuffer[TestAddress];
      if((Solution1 & 0b00000001) == 0)                   // if bottom bit is zero
      {
        SolutionFound = true;                             // we have found a match
        break;
      }
    }
  }
    // if we have a solution, set it and send success; else set bypass
    //(only if enabled!)
  if(GATUEnabled)
  {
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
// if ATU is enabled, initiate tune
//
void SetTuneOnOff(bool State)
{
  if(GATUEnabled)
  {
    GPCTuneActive = State;
    if(State && GPTTPressed)
      InitiateTune(GQuickTuneEnabled);                               // try quick tune
  }
}



//
// handle ATU on/off CAT message from PC
// or standalone mode: from display click
//
void SetATUOnOff(bool State)
{
  if((State != GATUEnabled) && GStandaloneMode)               // if state has changed, save it to EEPROM
    EEWriteEnabled(State);
    
  GATUEnabled = State;
  if(GATUEnabled)
    SetupForNewAntenna();                       // get new settings
  else
  {
    SetNullSolution();                          // set L, C values for ATU out of circuit
    DriveSolution();                            // disable straightaway
  }

  ShowATUEnabled(State);                        // debug
}


//
// handle quick tune on/off message from PC
//
void SetATUQuickTune(bool State)
{
  GQuickTuneEnabled = State;
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
// set the frequency the algorithm should use
//
  FindFreqRow(GTunedFrequency10/100);                                     // set algorithm frequency, in units of 1MHz
//
// now see if we have a solution
//
  if(GATUEnabled)
  {
    if(GPTTPressed)
      GQueuedFrequencyChange=true;
    else
      SetupForNewFrequency();
  }
  ShowFrequency(FreqString);
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
  if((Antenna >= 1) && (Antenna <= 4))
  {
    if(Antenna != GTXAntenna)
    {
      GTXAntenna = Antenna;
      SetAntennaSPI(Antenna, false);                                      // send new antenna setting to SPI
      SetupForNewAntenna();                                               // find solution etc
      ShowAntenna(Antenna);
    }
  }
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
// 01: inductance +; 02: capacitance +; 51: inductance -; 52: capacitance -
//
void HandleLCFineTune(int Command)
{
  int CurrentL;
  int CurrentC;
  byte Knob;
  byte Steps;
  CurrentL = GetInductance();
  CurrentC = GetCapacitance();
  Knob = Command / 10;                                  // encoder number (1,2,51,52)
  Steps = Command % 10;
  if(Knob == 1)
  {
    CurrentL = constrain(CurrentL + Steps, 0, 255);
    SetInductance(CurrentL);
    if (!GTuneActive)
      DriveSolution();                                    // send to shift registers
  }
  else if(Knob == 2)
  {
    CurrentC = constrain(CurrentC + Steps, 0, 255);
    SetCapacitance(CurrentC);
    if (!GTuneActive)
      DriveSolution();                                    // send to shift registers
  }
  else if(Knob == 51)
  {
    CurrentL = constrain(CurrentL - Steps, 0, 255);
    SetInductance(CurrentL);
    if (!GTuneActive)
      DriveSolution();                                    // send to shift registers
  }
  else if(Knob == 52)
  {
    CurrentC = constrain(CurrentC - Steps, 0, 255);
    SetCapacitance(CurrentC);
    if (!GTuneActive)
      DriveSolution();                                    // send to shift registers
  }

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

    case eZZOY:                                                       // ATU quick Tune
      SetATUQuickTune(ParsedBool);
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
    case eZZFT:                                                       // frequency change message
      SetNewFrequency(ParsedParam);
      break;
  }
}



//
// tune hardwired input ISR handler
// this triggers on falling edge, to trigger a "tune request"
// we assume it to come from an FPGA source, so no bounce
//
void HWTuneISR(void)
{
  if ((GTuneHWPressed == false) && (GTuneHWReleaseCount==0))
  {
    GTuneHWPressed = true;                                      // got TX/RX state from I/O pin
    GTuneHWReleaseCount = 2;
#ifdef CONDITIONAL_ALG_DEBUG
    Serial.println("HW TUNE");                                // debug to confirm state
#endif
    if(GATUEnabled)
    {
      GPCTuneActive = true;
      if(GTuneHWPressed && GPTTPressed)
        InitiateTune(GQuickTuneEnabled);                               // try quick tune
    }
  }
}




//
// PTT ISR handler
// this triggers on the falling edge only to initiate a PTT, so we can set RX and TX antenna as well as ATU tune solution
// this may bounce, due to presence of RF on grounds: so set "PTT pressed" and a PTT Tick counter to be polled
// and only handle the interrupt if PTT isn't already pressed and the timeout count expired
// (this prevents PTT being re-asserted by a bounce on its supposed rising edge)
// PTT release by polling code.
//
void PttISR(void)
{
  if ((GPTTPressed == false) && (GPTTReleaseCount==0))
  {
    GPTTPressed = true;                                   // got TX/RX state from I/O pin
    GPTTReleaseCount = 2;
    
#ifdef CONDITIONAL_ALG_DEBUG
    Serial.println("PTT");                                // debug to confirm state
#endif
    digitalWrite(VPINTR_PTTOUT, HIGH);                    // activate T/R output

//
// if Tune command already sent via CAT, initiate tune
// (there is a race condition and it's unclear whether the CAT tune command or PTT would happen first)
//
//    if(GPCTuneActive == true)
//      InitiateTune(GQuickTuneEnabled);                               // try quick tune
//    else
    if(GPCTuneActive == false)
    {
//
// either send new data to SPI, or queue a shift
//  
      if(GSPIShiftInProgress)
        GResendSPI = true;
      else
        DriveSolution();                                         // drive SPI. This sets T/R state AND sends L/C data
    }
  }
}



//
// 16ms tick
//
void CatHandlerTick()
{
  byte AntennaSelect = 0;                                  // ant input for standalone mode 
  
  // see if we have a queuesd frequency change, while PTT was pressed; handle when not pressed
  if((!GPTTPressed) && (GQueuedFrequencyChange))
  {
    GQueuedFrequencyChange=false;
    SetupForNewFrequency();
  }

  // check PTT state. First decrement the debounce count; PTT won't press or release if not zero.
  // then if the debounce count, see if PTT needs releasing.
  if(GPTTReleaseCount != 0)
    GPTTReleaseCount--;
  if((GPTTReleaseCount == 0) && (GPTTPressed == true))
  {
    if(digitalRead(VPINPTT) == HIGH)
    { 
#ifdef CONDITIONAL_ALG_DEBUG
      Serial.println("no PTT");                             // debug to confirm state
#endif
      GPTTReleaseCount = 2;
      digitalWrite(VPINTR_PTTOUT, LOW);                     // deactivate T/R output
      GPTTPressed = false;
      GPCTuneActive = false;                                // cancel CAT tune
      if(GTuneActive)                                       // if algorithm still running, cancel it
        CancelAlgorithm();
      DriveSolution();                                      // this sends RX antenna setting to H/W
    }
  }

  // check Tune strobe state state. First decrement the debounce count; TUNE won't press or release if not zero.
  // then if the debounce count, see if TUNE needs releasing.
  if(GTuneHWReleaseCount != 0)
    GTuneHWReleaseCount--;
  if((GTuneHWReleaseCount == 0) && (GTuneHWPressed == true))
  {
    if(digitalRead(VPINHWTUNECMD) == HIGH)
    { 
#ifdef CONDITIONAL_ALG_DEBUG
      Serial.println("no TUNE");                                // debug to confirm state
#endif
      GTuneHWReleaseCount = 2;
      GTuneHWPressed = false;
    }
  }

//
// in standalone mode, poll for frequency and check antenna select inputs
//
  if(GStandaloneMode)                                     // in standalone mode, poll for frequency
  {
    if(Serial)
    {
      if(GFreqPollTicks == 0)                               // if timed out, reload & send msg
      {
        GFreqPollTicks = VFREQPOLLINTERVAL;
        MakeCATMessageNoParam(eZZFT);                       // TX freq request
      }
      else
        GFreqPollTicks--;
    }

    if(digitalRead(VPINSTANDALONEANTA) == LOW)
      AntennaSelect |= 1;
    if(digitalRead(VPINSTANDALONEANTB) == LOW)
      AntennaSelect |= 2;
    AntennaSelect+= 1;
    if(AntennaSelect != GTXAntenna)
      HandleTXAntennaChange(AntennaSelect);
  }
}
