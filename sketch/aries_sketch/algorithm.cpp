/////////////////////////////////////////////////////////////////////////
//
// Aries ATU controller sketch by Laurence Barker G8NJJ
// this sketch controls an L-match ATU network
// with a CAT interface to connect to an HPSDR control program
// copyright (c) Laurence Barker G8NJJ 2019
//
// the code is written for an Arduino Nano 33 IoT module
//
// algorithm.cpp:  tuning algorithm to find ATU tune solution
/////////////////////////////////////////////////////////////////////////

#include "hwdriver.h"
#include "globalinclude.h"
#include "cathandler.h"
#include "LCD_UI.h"


//
// type definition for sequence state variable
//
enum EAlgorithmState
{ 
  eAlgIdle,
  eLowZCoarseStepC,
  eHighZCoarseStepL,
  eLowZCoarseStepL,
  eLowZMidStepC,
  eLowZMidStepL,
  eLowZFineStepC,
  eLowZFineStepL,
  eHighZCoarseStepC,
  eHighZMidStepL,
  eHighZMidStepC,
  eHighZFineStepL,
  eHighZFineStepC,
  eAlgEEPROMWrite   
}; 


//
// structure for tune parameters
//
struct STuneParams
{
  byte FreqMax;                       // max freq (MHz) covered by this row
  byte LMax;                          // max inductance (0-255)
  byte LCoarseStep;                   // coarse step size
  byte LMidStep;                      // mid step size
  byte CMax;                          // max capacitance (0-255)
  byte CCoarseStep;                   // coarse step size
  byte CMidStep;                      // mid step size
};


//
// array of frequency table records;
// this is indexed by the GFreqRow variable
// 
#define VNUMTUNEROWS 4                // this tells how far to step
STuneParams GTuneParamArray[] = 
{
  {1, 255, 16, 4, 255, 16, 4},        // 1.8MHz band
  {3, 255, 16, 4, 255, 16, 4},        // 3.5 MHz band
  {7, 255, 16, 4, 255, 16, 4},        // 7 MHz band
  {64, 20, 3, 1, 20, 3, 1}            // high frequenncy
};


#define VSUCCESSVSWR 150                  // threshold for "successful" tune 


//
// global variables
//
bool GIsQuickTune;              // true if we are trying a quick tune
bool GTuneActive;               // bool set true when algorithm running. Clear it to terminate.
byte GFreqRow;                  // row number on freq data table
byte GCurrentL;                 // current inductance value
byte GCurrentLStep;             // current L step value
byte GCurrentLMinRange;         // min sweep range for L tune
byte GCurrentLMaxRange;         // max sweep range for L tune
byte GCurrentC;                 // current capacitance value
byte GCurrentCStep;             // current C step value
byte GCurrentCMinRange;         // min sweep range for C tune
byte GCurrentCMaxRange;         // max sweep range for C tune
bool GCurrentZ;                 // current HighZ/LowZ value. True = HighZ
EAlgorithmState GAlgState;      // sequencer state variable
bool GIsEven;
byte GFreqMHz;                  // frequency in MHz

//
// params of min VSWR found in an algorithm state
//
unsigned int GMinVSWRFound;     // VSWR value found
byte MinVSWRLValue;             // L value corresponding to min VSWR
byte MinVSWRCValue;             // C  Value corresponding to min VSWR
unsigned int GMinVSWRFoundLowZ; // min value found in 1st step
byte MinVSWRLValueFoundLowZ;    // L value corresponding to min VSWR
byte MinVSWRCValueFoundLowZ;    // C  Value corresponding to min VSWR
#define VMAXVSWR 65535
unsigned int VSWRToPrint;


//
// function to initialise algorithm code
//
void InitialiseAlgorithm(void)
{
  GTuneActive = false;  
  GAlgState = eAlgIdle;
}



//
// find the frequency row to use
// sets the row variable for tuning parameters to use
// paramters is the required frequency (units of MHz)
// step DOWN through the rows
// 
void FindFreqRow(byte FrequencyMHz)
{
  int Row;               // row value being checked

  GFreqRow = 0;                                         // point to lowest freq by default
  for (Row=VNUMTUNEROWS-1; Row >= 0; Row--)                // step through all rows
  {
    if(FrequencyMHz <= GTuneParamArray[Row].FreqMax)
      GFreqRow = Row;
  }
#ifdef CONDITIONAL_ALG_DEBUG
    Serial.print("F=");
    Serial.print(FrequencyMHz);
    Serial.print(" MHz; row = ");
    Serial.print(GFreqRow);
    Serial.println();
#endif
  
}


//
// find the next L/C step
// sets the next value to use into global variables GCurrentL or GCurrentC
// this looks up the settings to use on a state dependent basis
// returns true if a valid new value, false if we are already at the end (ie time to move on)
//
bool FindNextStep(void)
{
  bool IsL;                           // IsL: true for inductance; false for capacitance
  byte Step;                          // Step: required step value
  bool Result = true;                 // return value
  int NewValue;                       // calculated new value: deliberately use int to trap under or overrange
//
// begin by finding the settings to use on a state dependent basis
//  
  switch(GAlgState)
  {
    case eAlgIdle:                          // shouldn't be called in this state!
    case eAlgEEPROMWrite:
      break;
      
    case eHighZCoarseStepC:
    case eLowZCoarseStepC:
      IsL = false;                                          // we are stepping C
      Step = GTuneParamArray[GFreqRow].CCoarseStep;         // coarse step C value
      break;

    case eHighZCoarseStepL:
      IsL = true;                                           // we are stepping L
      Step = GTuneParamArray[GFreqRow].LCoarseStep;         // coarse step L value
      break;

    case eLowZCoarseStepL:
      IsL = true;                                           // we are stepping L
      Step = GTuneParamArray[GFreqRow].LCoarseStep;         // coarse step L value
      break;

    case eHighZMidStepC:
    case eLowZMidStepC:
      IsL = false;                                          // we are stepping C
      Step = GTuneParamArray[GFreqRow].CMidStep;            // mid step C value
      break;

    case eHighZMidStepL:
    case eLowZMidStepL:
      IsL = true;                                           // we are stepping L
      Step = GTuneParamArray[GFreqRow].LMidStep;            // mid step L value
      break;

    case eHighZFineStepC:
    case eLowZFineStepC:
      IsL = false;                                          // we are stepping C
      Step = 1;                                             // fine step C value
      break;

    case eHighZFineStepL:
    case eLowZFineStepL:
      IsL = true;                                           // we are stepping L
      Step = 1;                                             // fine step L value
      break;
  }
//
// now apply the step
//
  if (IsL)                                                  // if inductance, increment GCurrentL
  {
    if (GCurrentL == GCurrentLMaxRange)                     // if reached end
      Result = false;
    else
    {
      NewValue = (int)GCurrentL;
      NewValue = constrain(NewValue + Step, GCurrentLMinRange, GCurrentLMaxRange);
      GCurrentL = (byte)NewValue;
    }
  }
  else                                                      // else if capacitance, increment GCurrentC
  {
    if (GCurrentC == GCurrentCMaxRange)                     // if reached end
      Result = false;
    else
    {
      NewValue = (int)GCurrentC;
      NewValue = constrain(NewValue + Step, GCurrentCMinRange, GCurrentCMaxRange);
      GCurrentC = (byte)NewValue;
    }
  }

  return Result;                                            // false if end of current step
}


//
// debug - print solution to serial
//
void PrintSolution(void)
{
  String StateName;
//
// find name for sequencer state
//
  switch(GAlgState)
  {
    case eAlgIdle:
      StateName = "Idle: ";
      break;
      
    case eLowZCoarseStepC:
      StateName = "LowZ, coarse step C: ";
      break;

    case eHighZCoarseStepL:
      StateName = "HighZ, coarse step L: ";
      break;

    case eLowZCoarseStepL:
      StateName = "LowZ, coarse step L: ";
      break;

    case eLowZMidStepC:
      StateName = "LowZ, mid step C: ";
      break;

    case eLowZMidStepL:
      StateName = "LowZ, mid step L: ";
      break;

    case eLowZFineStepC:
      StateName = "LowZ, fine step C: ";
      break;

    case eLowZFineStepL:
      StateName = "LowZ, fine step L: ";
      break;

    case eHighZCoarseStepC:
      StateName = "HighZ, coarse step C: ";
      break;

    case eHighZMidStepL:
      StateName = "HighZ, mid step L: ";
      break;

    case eHighZMidStepC:
      StateName = "HighZ, mid step C: ";
      break;

    case eHighZFineStepL:
      StateName = "HighZ, fine step L: ";
      break;

    case eHighZFineStepC:
      StateName = "HighZ, fine step C: ";
      break;

    case eAlgEEPROMWrite:
      StateName = "Write EEPROM: ";
      break;
  }
  Serial.print(StateName);
  if(GCurrentZ)
    Serial.print ("HiZ;");
  else
    Serial.print ("LoZ;");
  
  Serial.print("VSWR=");
  Serial.print(VSWRToPrint);
  Serial.print (" L=");
  Serial.print(GCurrentL);
  Serial.print (" C=");
  Serial.print(GCurrentC);
  Serial.println();
}




//
// send solution to hardware
// uses the global variables for the values set
//
void SendCandidateSolution(void)
{
  SetInductance(GCurrentL);               // inductance 0-255
  SetCapacitance(GCurrentC);              // capacitance 0-255
  SetHiLoZ(GCurrentZ);                    // true for high Z
  DriveSolution();
}


//
// function called to assess tune resut.
// if quick tune, decide whether to do full tune if VSWR found is good enough, cancel tuning and store solution; else go to full tune
// if full tune, send to "store" state
// 
//
void AssessTune(void)
{
  if (!GIsQuickTune)                                            // if this was a full tune
  {
    if (GMinVSWRFound < VSUCCESSVSWR)                           // if successful full tune
    {
      GAlgState = eAlgEEPROMWrite;                // found a solution
      GCurrentC = MinVSWRCValue;                  // assert it to relays (in algorithmtick())
      GCurrentL = MinVSWRLValue;

      SetTuneResult(true, GCurrentL, GCurrentC, GCurrentZ);
#ifdef CONDITIONAL_ALG_DEBUG
      Serial.print("Full tune: successful best found: ");               
      Serial.print("VSWR=");
      Serial.print(GMinVSWRFound);
      Serial.print (" L=");
      Serial.print(MinVSWRLValue);
      Serial.print (" C=");
      Serial.print(MinVSWRCValue);
      Serial.println();
#endif
    }
    else                                          // unsuccessfyl full tune
    {
      SetNullSolution();                          // put ATU into bypass
      DriveSolution();                            // send to hardware
      GAlgState = eAlgIdle;                       // found a solution
      SetTuneResult(false, GCurrentL, GCurrentC, GCurrentZ);
  #ifdef CONDITIONAL_ALG_DEBUG
      Serial.print("Full tune FAIL: unsuccessful best found: ");               
      Serial.print("VSWR=");
      Serial.print(GMinVSWRFound);
      Serial.print (" L=");
      Serial.print(MinVSWRLValue);
      Serial.print (" C=");
      Serial.print(MinVSWRCValue);
      Serial.println();
  #endif
    }
    
    GAlgState = eAlgEEPROMWrite;                // found a solution
    GCurrentC = MinVSWRCValue;                  // assert it to relays
    GCurrentL = MinVSWRLValue;
#ifdef CONDITIONAL_ALG_DEBUG
    Serial.print("Full tune: best found: ");               
    Serial.print("VSWR=");
    Serial.print(GMinVSWRFound);
    Serial.print (" L=");
    Serial.print(MinVSWRLValue);
    Serial.print (" C=");
    Serial.print(MinVSWRCValue);
    Serial.println();
#endif
  }
  else if (GMinVSWRFound < VSUCCESSVSWR)                        // else if good enough quick tune
  {
    GAlgState = eAlgEEPROMWrite;                // found a solution
    GCurrentC = MinVSWRCValue;                  // assert it to relays
    GCurrentL = MinVSWRLValue;
#ifdef CONDITIONAL_ALG_DEBUG
    Serial.print("Quick tune: best found: ");               
    Serial.print("VSWR=");
    Serial.print(GMinVSWRFound);
    Serial.print (" L=");
    Serial.print(MinVSWRLValue);
    Serial.print (" C=");
    Serial.print(MinVSWRCValue);
    Serial.println();
#endif
    GIsQuickTune = false;
  }
  else                                                          // initiate full tune
  {
#ifdef CONDITIONAL_ALG_DEBUG
    Serial.println("Quick tune: no solution found ");               
#endif
// for full tune: set L=0; Step C=0-255 at coarse step
    GCurrentZ = false;            // set Low Z for initial search
    GCurrentL = 0;                // current inductance value
    GCurrentC = 0;                // current capacitance value
    GCurrentLStep = GTuneParamArray[GFreqRow].LCoarseStep;            // coarse step L value
    GCurrentCStep = GTuneParamArray[GFreqRow].CCoarseStep;            // coarse step C value
    GCurrentLMinRange = 0;                                            // min sweep =0 for L tune
    GCurrentLMaxRange = GTuneParamArray[GFreqRow].LMax;               // max sweep range for L tune
    GCurrentCMinRange = 0;                                            // min sweep =0 for C tune
    GCurrentCMaxRange = GTuneParamArray[GFreqRow].CMax;               // max sweep range for C tune
    GTuneActive = true;                                               // set active
    GAlgState = eLowZCoarseStepC;                                     // set state
    SendCandidateSolution();                                          // drive hardware
    GMinVSWRFound = VMAXVSWR;                                         // initialise VSWR best value found = worst possible
    GIsQuickTune = false;                                             // cancel quick tune state
  }
}




//
// function algorithm code periodic tick
//
void AlgorithmTick(void)
{
  int VSWRValue;                          // 100*VSWR found in current step
  bool ValidNewStep = false;              // set true if not yet time to move onto next step
  byte Step;

//
// see if terminated
//
  if(GTuneActive == false)
    GAlgState = eAlgIdle;
  
//
// if executing algorithm, get VSWR and store if better than last
//
  if ((GAlgState != eAlgIdle) && (GAlgState != eAlgEEPROMWrite))
  {
    VSWRValue = GetVSWR();
    VSWRToPrint = VSWRValue;
    if (VSWRValue < GMinVSWRFound)
    {
      GMinVSWRFound = VSWRValue;
      MinVSWRLValue = GCurrentL;              // L value corresponding to min VSWR
      MinVSWRCValue = GCurrentC;              // C  Value corresponding to min VSWR
    }
#ifdef CONDITIONAL_ALG_DEBUG
    PrintSolution();
#endif
//
// now work out proposed next step (if state changes, this may be overridden)
//
    ValidNewStep = FindNextStep();
  }

//
// now see what the sequencer tells us to do next!
// in most cases, nothing more if ValidNewStep == true
//
  switch(GAlgState)
  {
    case eAlgIdle:

      break;
      
    case eLowZCoarseStepC:
      if (!ValidNewStep)                        // if that step has finished
      {
        GMinVSWRFoundLowZ = GMinVSWRFound;      // store the best VSWR we got in low Z
        MinVSWRLValueFoundLowZ = MinVSWRLValue; // L value corresponding to min VSWR (I think L = 0)
        MinVSWRCValueFoundLowZ = MinVSWRCValue; // C  Value corresponding to min VSWR
        GMinVSWRFound = VMAXVSWR;               // initialise VSWR best value found = worst possible
        GAlgState = eHighZCoarseStepL;          // step the high Z setting to find min VSWR next
// hold L=0; Step C=0-255 at coarse step
        GCurrentZ = true;                       // setup for  High Z for initial search
        GCurrentL = 0;                          // current inductance value
        GCurrentC = 0;                          // current capacitance value
      }
      break;

    case eHighZCoarseStepL:
      if (!ValidNewStep)                            // if that step has finished - choose whether high or low Z was better
      {
        if (GMinVSWRFoundLowZ < GMinVSWRFound)      // if Low Z was better
        {
#ifdef CONDITIONAL_ALG_DEBUG
          Serial.println("LowZ was better.");
#endif
// coarse step L=0-255; C=best value from low Z step
          GCurrentZ = false;                        // re-select Low Z
          GMinVSWRFound = VMAXVSWR;                 // initialise VSWR best value found = worst possible
          GAlgState = eLowZCoarseStepL;             // step L in low Z
          GCurrentC = MinVSWRCValueFoundLowZ;       // best C value found in low Z search
          GCurrentLMinRange = 0;
          GCurrentLMaxRange = 255;
          GCurrentL = GCurrentLMinRange;            // start at L=0
        }
        else                                        // high Z was better  
        {
#ifdef CONDITIONAL_ALG_DEBUG
          Serial.println("HighZ was better.");
#endif
// L= best value from current state; coarse step C=0-255
          GMinVSWRFound = VMAXVSWR;                 // initialise VSWR best value found = worst possible
          GAlgState = eHighZCoarseStepC;            // step C in high Z setting to find min VSWR next
          GCurrentL = MinVSWRLValue;                // best L value found in high Z search
          GCurrentCMinRange = 0;
          GCurrentCMaxRange = 255;
          GCurrentC = 0;                            // start at 0
        }
      }
      break;

    case eLowZCoarseStepL:
      if (!ValidNewStep)                                    // if that step has finished
      {
        GMinVSWRFound = VMAXVSWR;                           // initialise VSWR best value found = worst possible
        GAlgState = eLowZMidStepC;                          // step the low Z setting 
// L= best value from current state; mid step C range=current value +/- 2 coarse step
        Step = GTuneParamArray[GFreqRow].CCoarseStep;       // coarse step C value
        GCurrentL = MinVSWRLValue;                          // best inductance value
        GCurrentCMinRange = constrain(GCurrentC - 2* Step, 0, GTuneParamArray[GFreqRow].CMax); // current capacitance value +/-
        GCurrentCMaxRange = constrain(GCurrentC + 2* Step, 0, GTuneParamArray[GFreqRow].CMax);
        GCurrentC = GCurrentCMinRange;                      
      }

      break;

    case eLowZMidStepC:
      if (!ValidNewStep)                                    // if that step has finished
      {
        GMinVSWRFound = VMAXVSWR;                           // initialise VSWR best value found = worst possible
        GAlgState = eLowZMidStepL;                          // step the low Z setting 
// C= best value from current state; mid step L range =current value +/- 2 coarse step
        Step = GTuneParamArray[GFreqRow].LCoarseStep;       // coarse step L value
        GCurrentC = MinVSWRCValue;                          // best capacitance value
        GCurrentLMinRange = constrain(GCurrentL - 2* Step, 0, GTuneParamArray[GFreqRow].LMax); // current inductance value +/-
        GCurrentLMaxRange = constrain(GCurrentL + 2* Step, 0, GTuneParamArray[GFreqRow].LMax);
        GCurrentL = GCurrentLMinRange;                      
      }
      break;

    case eLowZMidStepL:
      if (!ValidNewStep)                                    // if that step has finished
      {
        GMinVSWRFound = VMAXVSWR;                           // initialise VSWR best value found = worst possible
        GAlgState = eLowZFineStepC;                         // step the low Z setting 
// L= best value from current state; fine step C range=current value +/- 2 mid step
        Step = GTuneParamArray[GFreqRow].CMidStep;          // mid step C value
        GCurrentL = MinVSWRLValue;                          // best inductance value
        GCurrentCMinRange = constrain(GCurrentC - 2* Step, 0, GTuneParamArray[GFreqRow].CMax); // current capacitance value +/-
        GCurrentCMaxRange = constrain(GCurrentC + 2* Step, 0, GTuneParamArray[GFreqRow].CMax);
        GCurrentC = GCurrentCMinRange;                      
      }
      break;

    case eLowZFineStepC:
      if (!ValidNewStep)                                    // if that step has finished
      {
        GMinVSWRFound = VMAXVSWR;                           // initialise VSWR best value found = worst possible
        GAlgState = eLowZFineStepL;                         // step the low Z setting 
// C= best value from current state; fine step L range =current value +/- 2 mid step
        Step = GTuneParamArray[GFreqRow].LMidStep;          // mid step L value
        GCurrentC = MinVSWRCValue;                          // best capacitance value
        GCurrentLMinRange = constrain(GCurrentL - 2* Step, 0, GTuneParamArray[GFreqRow].LMax); // current inductance value +/-
        GCurrentLMaxRange = constrain(GCurrentL + 2* Step, 0, GTuneParamArray[GFreqRow].LMax);
        GCurrentL = GCurrentLMinRange;                      
      }
      break;

    case eLowZFineStepL:
      if (!ValidNewStep)                            // if that step has finished
      {
        AssessTune();                               // finisned - decide how to proceed
      }
      break;

    case eHighZCoarseStepC:
      if (!ValidNewStep)                                    // if that step has finished
      {
        GMinVSWRFound = VMAXVSWR;                           // initialise VSWR best value found = worst possible
        GAlgState = eHighZMidStepL;                         // step the high Z setting 
// C= best value from current state; mid step L range =current value +- 2 coarse steps
        Step = GTuneParamArray[GFreqRow].LCoarseStep;       // coarse step L value
        GCurrentC = MinVSWRCValue;                          // best capacitance value
        GCurrentLMinRange = constrain(GCurrentL - 2* Step, 0, GTuneParamArray[GFreqRow].LMax); // current inductance value +/-
        GCurrentLMaxRange = constrain(GCurrentL + 2* Step, 0, GTuneParamArray[GFreqRow].LMax);
        GCurrentL = GCurrentLMinRange;                      
      }
      break;

    case eHighZMidStepL:
      if (!ValidNewStep)                                    // if that step has finished
      {
        GMinVSWRFound = VMAXVSWR;                           // initialise VSWR best value found = worst possible
        GAlgState = eHighZMidStepC;                         // step the high Z setting 
// L= best value from current state; mid step C range =current value +- 2 coarse steps
        Step = GTuneParamArray[GFreqRow].CCoarseStep;       // coarse step C value
        GCurrentL = MinVSWRLValue;                          // best inductance value
        GCurrentCMinRange = constrain(GCurrentC - 2* Step, 0, GTuneParamArray[GFreqRow].CMax); // current capacitance value +/-
        GCurrentCMaxRange = constrain(GCurrentC + 2* Step, 0, GTuneParamArray[GFreqRow].CMax);
        GCurrentC = GCurrentCMinRange;                      
      }
      break;

    case eHighZMidStepC:
      if (!ValidNewStep)                                    // if that step has finished
      {
        GMinVSWRFound = VMAXVSWR;                           // initialise VSWR best value found = worst possible
        GAlgState = eHighZFineStepL;                        // step the high Z setting 
// C= best value from current state; fine step L range =current value +- 2 mid steps
        Step = GTuneParamArray[GFreqRow].LMidStep;          // mid step L value
        GCurrentC = MinVSWRCValue;                          // best capacitance value
        GCurrentLMinRange = constrain(GCurrentL - 2* Step, 0, GTuneParamArray[GFreqRow].LMax); // current inductance value +/-
        GCurrentLMaxRange = constrain(GCurrentL + 2* Step, 0, GTuneParamArray[GFreqRow].LMax);
        GCurrentL = GCurrentLMinRange;                      
      }
      break;

    case eHighZFineStepL:
      if (!ValidNewStep)                                    // if that step has finished
      {
        GMinVSWRFound = VMAXVSWR;                           // initialise VSWR best value found = worst possible
        GAlgState = eHighZFineStepC;                        // step the high Z setting 
// L= best value from current state; fine step C range =current value +- 2 mid steps
        Step = GTuneParamArray[GFreqRow].CMidStep;          // mid step C value
        GCurrentL = MinVSWRLValue;                          // best inductance value
        GCurrentCMinRange = constrain(GCurrentC - 2* Step, 0, GTuneParamArray[GFreqRow].CMax); // current capacitance value +/-
        GCurrentCMaxRange = constrain(GCurrentC + 2* Step, 0, GTuneParamArray[GFreqRow].CMax);
        GCurrentC = GCurrentCMinRange;                      
      }
      break;

    case eHighZFineStepC:
      if (!ValidNewStep)                            // if that step has finished
      {
        AssessTune();                               // finished - decide what to do
      }
      break;

    case eAlgEEPROMWrite:
      GAlgState = eAlgIdle;                         // finished calculating
      LCU_UI_SetTuning(false);
      break;
  }
//
// finally send L,C, low/high Z switch setting to hardware if we are in a search state
//  
  if (GAlgState != eAlgIdle)
    SendCandidateSolution();
}



//
// cancel algorithm
//
void CancelAlgorithm(void)
{
  GTuneActive = false;
}




//
// function to initiate a tune algorithm sequence - beginning with a quick tune
// this initialises the data structures so that a series of timer ticks will step through the cycle
// set variables for both L and C initial quick tune fine steps
//
void InitiateQuickTune(bool StartQuick)
{
  byte Step;

  if (StartQuick)
  {
    GIsQuickTune = true;                                // set "this is a quick tune attempt"
    GCurrentZ = GetHiLoZ();                             // get Z for initial search
    GCurrentL = GetInductance();                        // current inductance value
    GCurrentC = GetCapacitance();                       // current capacitance value
    if (GCurrentZ)                                      // high Z
    {
      GAlgState = eHighZFineStepL;                      // step the high Z setting 
// C= best value from current state; fine step L range =current value +- 2 mid steps
      Step = GTuneParamArray[GFreqRow].LMidStep;        // mid step L value
      GCurrentC = MinVSWRCValue;                        // best capacitance value
      GCurrentLMinRange = constrain(GCurrentL - 2* Step, 0, GTuneParamArray[GFreqRow].LMax); // current inductance value +/-
      GCurrentLMaxRange = constrain(GCurrentL + 2* Step, 0, GTuneParamArray[GFreqRow].LMax);
      GCurrentL = GCurrentLMinRange;                      
    }
    else
    {
      GAlgState = eLowZFineStepC;                         // step the low Z setting 
// L= best value from current state; fine step C range=current value +/- 2 mid step
      Step = GTuneParamArray[GFreqRow].CMidStep;          // mid step C value
      GCurrentL = MinVSWRLValue;                          // best inductance value
      GCurrentCMinRange = constrain(GCurrentC - 2* Step, 0, GTuneParamArray[GFreqRow].CMax); // current capacitance value +/-
      GCurrentCMaxRange = constrain(GCurrentC + 2* Step, 0, GTuneParamArray[GFreqRow].CMax);
      GCurrentC = GCurrentCMinRange;                      
    }
  }
  else                                                                // start a normal full tune
  {
// hold L=0; Step C=0-255 at coarse step
    GIsQuickTune = false;                                             // set "this is a quick tune attempt"
    GCurrentZ = false;            // set Low Z for initial search
    GCurrentL = 0;                // current inductance value
    GCurrentC = 0;                // current capacitance value
    GCurrentLStep = GTuneParamArray[GFreqRow].LCoarseStep;            // coarse step L value
    GCurrentCStep = GTuneParamArray[GFreqRow].CCoarseStep;            // coarse step C value
    GCurrentLMinRange = 0;                                            // min sweep =0 for L tune
    GCurrentLMaxRange = GTuneParamArray[GFreqRow].LMax;               // max sweep range for L tune
    GCurrentCMinRange = 0;                                            // min sweep =0 for C tune
    GCurrentCMaxRange = GTuneParamArray[GFreqRow].CMax;               // max sweep range for C tune
    GAlgState = eLowZCoarseStepC;                                     // set state
  }
  GTuneActive = true;                                               // set active
  SendCandidateSolution();                                          // drive hardware
  GMinVSWRFound = VMAXVSWR;                                         // initialise VSWR best value found = worst possible
}
