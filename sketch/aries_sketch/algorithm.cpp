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
// this is the new version, table driven algorithm
/////////////////////////////////////////////////////////////////////////

#include "hwdriver.h"
#include "globalinclude.h"
#include "cathandler.h"
#include "LCD_UI.h"



#define VSUCCESSVSWR 150                  // threshold for "successful" tune VSWR = 1.5
#define VMAXVSWR 65535



//
// type definition for sequence state variable
//
enum EAlgorithmState
{ 
  eAlgIdle,
  eAlgCoarse1, 
  eAlgCoarse2,
  eAlgMid1,
  eAlgMid2,
  eAlgFine1, 
  eAlgFine2,
  eAlgEEPROMWrite   
}; 


//
// structure for tune parameters
//
struct STuneParams
{
  byte FreqMax;                       // max freq (MHz) covered by this row
  byte Alg1StartRow;                  // starting row for table 2 (alg stage 1 params)
  byte Alg1NumRows;                   // number of rows at this frequency in that table  
  byte LMax;                          // max inductance (0-255)
  byte CMax;                          // max capacitance (0-255)
  byte Stage1bStep;                   // step size to use  (min=0, max=param max)
  byte Stage2MidRange;                // +/- range for mid search
  byte Stage2MidStep;                 // step size for mid step
  byte Stage2FineRange;               // +/- range for fine search
};

//
// structure for a sweep (algorithm phase 1 is a set of these in a table)
//
struct SSweepSet
{
  bool IsHighZ;                       // true for a high Z search
  bool IsSweepingL;                   // true if sweeping inductance
  byte MinSteppedValue;               // min value for paramter being stepped
  byte MaxSteppedValue;               // max value for paramter being stepped
  byte StepSize;                      // step size
  byte FixedParam;                    // value to use for the paramter not stepped
};


//
// structure for best result found
//
struct SResult
{
  byte CValue;                        // Capacitance value
  byte LValue;                        // inductance value
  bool HighZ;                         // true for a high Z search
  unsigned int VSWR;                  // 100* best VSWR value found
  bool IsSweepingL;                   // true if sweeping inductance (needed to select stage 1b sweep)
};


//
// array of frequency table records;
// this is indexed by the GFreqRow variable
// the entire algorithm can be changed by replacing this table
//
#define VNUMTUNEROWS 6                // this tells how far to step
STuneParams GTuneParamArray[] = 
// FMax,startrow,#rows, Lmax, Cmax, S1bStep, S2Mid+-, S2MidStep, S2Fine+- 
{
  {1, 0, 12, 255, 255, 24, 32, 4, 8},        // 1.8MHz band
  {3, 12, 12, 255, 255, 24, 32, 4, 8},        // 3.5MHz band
  {7, 24, 12, 140, 140, 12, 24, 4, 8},        // 7MHz band
  {14, 36, 12, 63, 63, 6, 12, 2, 8},        // 14MHz band
  {29, 48, 12, 40, 40, 3, 8, 1, 8},        // 21 & 28MHz band
  {64, 60, 12, 20, 20, 2, 8, 1, 8},        // 50MHz band
};


//
// array of frequency table records;
// this is indexed by the GFreqRow variable
// 
#define VNUMSTAGE1ROWS 72                 // this tells how far to step


SSweepSet GStage1Array[] = 
// highZ,LSweep, MinStepped,MaxStepped, Step, Fixed
{
  {false, false, 0, 255, 24, 0},            // 1.8MHz try1 loZ step C
  {false, true, 0, 255, 24, 0},             // 1.8MHz try1 loz step L
  {true, false, 0, 255, 24, 0},             // 1.8MHz try1 hiZ step C
  {true, true, 0, 255, 24, 0},              // 1.8MHz try1 hiZ step L
  {false, false, 0, 255, 24, 32},           // 1.8MHz try2 loZ step C
  {false, true, 0, 255, 24, 32},            // 1.8MHz try2 loz step L
  {true, false, 0, 255, 24, 32},            // 1.8MHz try2 hiZ step C
  {true, true, 0, 255, 24, 32},             // 1.8MHz try2 hiZ step L
  {false, false, 0, 255, 24, 64},           // 1.8MHz try3 loZ step C
  {false, true, 0, 255, 24, 64},            // 1.8MHz try3 loz step L
  {true, false, 0, 255, 24, 64},            // 1.8MHz try3 hiZ step C
  {true, true, 0, 255, 24, 64},             // 1.8MHz try3 hiZ step L

  {false, false, 0, 255, 24, 0},            // 3.5MHz try1 loZ step C
  {false, true, 0, 255, 24, 0},             // 3.5MHz try1 loz step L
  {true, false, 0, 255, 24, 0},             // 3.5MHz try1 hiZ step C
  {true, true, 0, 255, 24, 0},              // 3.5MHz try1 hiZ step L
  {false, false, 0, 255, 24, 24},           // 3.5MHz try2 loZ step C
  {false, true, 0, 255, 24, 24},            // 3.5MHz try2 loz step L
  {true, false, 0, 255, 24, 24},            // 3.5MHz try2 hiZ step C
  {true, true, 0, 255, 24, 24},             // 3.5MHz try2 hiZ step L
  {false, false, 0, 255, 24, 48},           // 3.5MHz try3 loZ step C
  {false, true, 0, 255, 24, 48},            // 3.5MHz try3 loz step L
  {true, false, 0, 255, 24, 48},            // 3.5MHz try3 hiZ step C
  {true, true, 0, 255, 24, 48},             // 3.5MHz try3 hiZ step L

  {false, false, 0, 140, 12, 0},            // 7MHz try1 loZ step C
  {false, true, 0, 140, 12, 0},             // 7MHz try1 loz step L
  {true, false, 0, 140, 12, 0},             // 7MHz try1 hiZ step C
  {true, true, 0, 140, 12, 0},              // 7MHz try1 hiZ step L
  {false, false, 0, 140, 12, 12},           // 7MHz try2 loZ step C
  {false, true, 0, 140, 12, 12},            // 7MHz try2 loz step L
  {true, false, 0, 140, 12, 12},            // 7MHz try2 hiZ step C
  {true, true, 0, 140, 12, 12},             // 7MHz try2 hiZ step L
  {false, false, 0, 140, 12, 24},           // 7MHz try3 loZ step C
  {false, true, 0, 140, 12, 24},            // 7MHz try3 loz step L
  {true, false, 0, 140, 12, 24},            // 7MHz try3 hiZ step C
  {true, true, 0, 140, 12, 24},             // 7MHz try3 hiZ step L

  {false, false, 0, 63, 6, 0},              // 14MHz try1 loZ step C
  {false, true, 0, 63, 6, 0},               // 14MHz try1 loz step L
  {true, false, 0, 63, 6, 0},               // 14MHz try1 hiZ step C
  {true, true, 0, 63, 6, 0},                // 14MHz try1 hiZ step L
  {false, false, 0, 63, 6, 12},             // 14MHz try2 loZ step C
  {false, true, 0, 63, 6, 12},              // 14MHz try2 loz step L
  {true, false, 0, 63, 6, 12},              // 14MHz try2 hiZ step C
  {true, true, 0, 63, 6, 12},               // 14MHz try2 hiZ step L
  {false, false, 0, 63, 6, 24},             // 14MHz try3 loZ step C
  {false, true, 0, 63, 6, 24},              // 14MHz try3 loz step L
  {true, false, 0, 63, 6, 24},              // 14MHz try3 hiZ step C
  {true, true, 0, 63, 6, 24},               // 14MHz try3 hiZ step L

  {false, false, 0, 35, 3, 0},              // 21-28MHz try1 loZ step C
  {false, true, 0, 35, 3, 0},               // 21-28MHz try1 loz step L
  {true, false, 0, 35, 3, 0},               // 21-28MHz try1 hiZ step C
  {true, true, 0, 35, 3, 0},                // 21-28MHz try1 hiZ step L
  {false, false, 0, 35, 3, 5},              // 21-28MHz try2 loZ step C
  {false, true, 0, 35, 3, 5},               // 21-28MHz try2 loz step L
  {true, false, 0, 35, 3, 5},               // 21-28MHz try2 hiZ step C
  {true, true, 0, 35, 3, 5},                // 21-28MHz try2 hiZ step L
  {false, false, 0, 35, 3, 10},             // 21-28MHz try3 loZ step C
  {false, true, 0, 35, 3, 10},              // 21-28MHz try3 loz step L
  {true, false, 0, 35, 3, 10},              // 21-28MHz try3 hiZ step C
  {true, true, 0, 35, 3, 10},               // 21-28MHz try3 hiZ step L

  {false, false, 0, 20, 2, 0},              // 50MHz try1 loZ step C
  {false, true, 0, 20, 2, 0},               // 50MHz try1 loz step L
  {true, false, 0, 20, 2, 0},               // 50MHz try1 hiZ step C
  {true, true, 0, 20, 2, 0},                // 50MHz try1 hiZ step L
  {false, false, 0, 20, 2, 3},              // 50MHz try2 loZ step C
  {false, true, 0, 20, 2, 3},               // 50MHz try2 loz step L
  {true, false, 0, 20, 2, 3},               // 50MHz try2 hiZ step C
  {true, true, 0, 20, 2, 3},                // 50MHz try2 hiZ step L
  {false, false, 0, 20, 2, 6},              // 50MHz try3 loZ step C
  {false, true, 0, 20, 2, 6},               // 50MHz try3 loz step L
  {true, false, 0, 20, 2, 6},               // 50MHz try3 hiZ step C
  {true, true, 0, 20, 2, 6},                // 50MHz try3 hiZ step L
};




//
// global variables
//
byte GFreqMHz;                  // frequency in MHz
EAlgorithmState GAlgState;      // sequencer state variable
bool GIsQuickTune;              // true if we are trying a quick tune
bool GTuneActive;               // bool set true when algorithm running. Clear it to terminate.

//
// algorithm variables/
//
byte GFreqRow;                  // row number on freq data table (GTuneParamArray)
byte GStage1Row;                // row number in stage 1 table (GStage1Array). Offset, beginning at zero 
bool GCurrentZ;                 // high/low impedance setting
byte GCurrentL;                 // current inductance value
// byte GCurrentC;              // current capacitance value
//
// params of min VSWR found in an algorithm state, and the current setting
//
SResult GCurrentSetting;        // current L/C/Z and VSWR
SResult GBestFoundSoFar;        // best found so far
SSweepSet GCurrentSweep;        // paramters for current sweep

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
    Serial.print(" MHz; 1st table row = ");
    Serial.print(GFreqRow);
    Serial.println();
#endif
}



//
// debug code - get state name as a string
//
String GetStateName(void)
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
      
    case eAlgCoarse1:
      StateName = "Coarse stage 1, row=";
      StateName += GStage1Row;
      StateName += ": ";
      break;

    case eAlgCoarse2:
      StateName = "Coarse stage 2: ";
      break;

    case eAlgMid1:
      StateName = "1st Mid Step: ";
      break;

    case eAlgMid2:
      StateName = "2nd Mid Step: ";
      break;

    case eAlgFine1:
      StateName = "1st Fine Step: ";
      break;

    case eAlgFine2:
      StateName = "2nd Fine Step: ";
      break;

    case eAlgEEPROMWrite:
      StateName = "Write EEPROM: ";
      break;
  }
  return StateName;
}


//
// debug code - print solution to serial
// given a string to print first. 
// 2nd parameter sets whether to print current solution(true) or best found.
void PrintSolution(String Text, bool IsCurrent)
{
  float VSWRPrintValue;
  SResult* StructPtr;

  if(IsCurrent)
    StructPtr = &GCurrentSetting;
  else
    StructPtr = &GBestFoundSoFar;
//
// print state name, then the current solution
//  
  Serial.print(Text);
  VSWRPrintValue = ((float)(StructPtr -> VSWR)) / 100.0;
  Serial.print("VSWR=");
  Serial.print(VSWRPrintValue);
  if(StructPtr -> HighZ)
    Serial.print ("; HiZ;");
  else
    Serial.print ("; LoZ;");
  Serial.print (" L=");
  Serial.print(StructPtr -> LValue);
  Serial.print (" C=");
  Serial.print(StructPtr -> CValue);
  Serial.println();
}



//
// send solution to hardware
// uses the global variables for the values set
// depending on its paramter it sends the current setting, or the best found
//
void SendCandidateSolution(bool IsCurrent)
{
  SResult* StructPtr;

  if(IsCurrent)
    StructPtr = &GCurrentSetting;
  else
    StructPtr = &GBestFoundSoFar;
    
  SetInductance(StructPtr -> LValue);               // inductance 0-255
  SetCapacitance(StructPtr -> CValue);              // capacitance 0-255
  SetHiLoZ(StructPtr -> HighZ);                     // true for high Z
  DriveSolution();
}



//
// get start settings into current setting from sweep
//
void InitialiseCurrentFromSweep(void)
{
  GCurrentSetting.HighZ = GCurrentSweep.IsHighZ;
  if(GCurrentSweep.IsSweepingL)
  {
    GCurrentSetting.CValue = GCurrentSweep.FixedParam;
    GCurrentSetting.LValue = GCurrentSweep.MinSteppedValue;
  }
  else
  {
    GCurrentSetting.LValue = GCurrentSweep.FixedParam;
    GCurrentSetting.CValue = GCurrentSweep.MinSteppedValue;
  }
}


// 
// set up for next stage 1 row
// parameter true for getting 1st row; otherwise it increments.
// result true if OK, else it has run out of rows
//
bool GetStage1Row(bool IsFirst)
{
  bool Result = true;
  byte AbsoluteRow;
  
  if(IsFirst)
    GStage1Row = 0;                                 // point at first row
  else
  {
    GStage1Row++;                                   // point at next row, and check if we have run out
    if(GStage1Row >= GTuneParamArray[GFreqRow].Alg1NumRows)
      Result = false;
  }
//
// if there are rows left, copy current one out and set up for sweep
// with Z set, and L,C at first values
//
  if (Result)
  {
    AbsoluteRow = GStage1Row + GTuneParamArray[GFreqRow].Alg1StartRow;
    GCurrentSweep = GStage1Array[AbsoluteRow];
    InitialiseCurrentFromSweep();
#ifdef CONDITIONAL_ALG_DEBUG
    Serial.print("Setting up row=");
    Serial.print(AbsoluteRow);
    Serial.println();
#endif
  }
  return Result;
}



//
// function called to assess tune result.
// there are 4 possible outcomes:
// 1. full tune, successful: cancel tuning, store best solution & success report
// 2. full tune, but not successful. cancel tuning, store best solution & set failure report
// 3. quick tune, successful result. cancel tuning, store best solution & report
// 4. quick tune, unsuccessful result. Initiate full tune.
// After this has been called, the current solution struct is sent to the relays as "best found" even if not meeting VSWR target
//
void AssessTune(void)
{
  String Text;                                                  // debug labelling
  bool GotRow;                                                  // true if we find 1st row of tune
  
  if (!GIsQuickTune)                                            // if this was a full tune
  {
    if (GBestFoundSoFar.VSWR < VSUCCESSVSWR)                    // if successful full tune
    {
      GAlgState = eAlgEEPROMWrite;
      SendCandidateSolution(false);
      SetTuneResult(true, GBestFoundSoFar.LValue, GBestFoundSoFar.CValue, GBestFoundSoFar.HighZ);
#ifdef CONDITIONAL_ALG_DEBUG
      Text = "Full tune: successful best found: ";
      PrintSolution(Text, false);                               // print best solution to serial port
#endif
    }
    else                                                        // unsuccessful full tune
    {
      GAlgState = eAlgEEPROMWrite;
      SendCandidateSolution(false);
      SetTuneResult(false, GBestFoundSoFar.LValue, GBestFoundSoFar.CValue, GBestFoundSoFar.HighZ);    // sends fail message and writes "no solution"
  #ifdef CONDITIONAL_ALG_DEBUG
      Text = "Full tune FAIL: unsuccessful best found: ";               
      PrintSolution(Text, false);                               // print best solution to serial port
  #endif
    }
  }
//
// now the two cases where it was a quick tune
//
  else if (GBestFoundSoFar.VSWR < VSUCCESSVSWR)                        // else if good enough quick tune
  {
    GIsQuickTune = false;
    GAlgState = eAlgEEPROMWrite;
    SendCandidateSolution(false);
    SetTuneResult(true, GBestFoundSoFar.LValue, GBestFoundSoFar.CValue, GBestFoundSoFar.HighZ);
#ifdef CONDITIONAL_ALG_DEBUG
    Text = "Quick tune: successful best found: ";
    PrintSolution(Text, false);                                 // print best solution to serial port
#endif
  }
  else                                                          // unsuccessful quick - initiate full tune
  {
#ifdef CONDITIONAL_ALG_DEBUG
    Serial.println("Quick tune: no solution found ");               
#endif
// for full tune: set L=0; Step C=0-255 at coarse step
    GIsQuickTune = false;                                             // cancel quick tune state
    GBestFoundSoFar.VSWR = VMAXVSWR;                                  // initialise VSWR best value found = worst possible
    GAlgState = eAlgCoarse1;                                          // set state
//
// copy out the first row of the stage 1 algorithm table
//
    GotRow = GetStage1Row(true);
    SendCandidateSolution(true);                                      // drive hardware
  }
}



//
// find the next L/C step
// sets the next value to use into global structure GCurrent
// the step paramters to use are defined in structure GCurrentStepSet
// returns true if a valid new value, false if we are already at the end (ie time to move on)
//
bool FindNextStep(void)
{
  bool Result = true;                 // return value
  int NewValue;                       // calculated new value: deliberately use int to trap under or overrange

//
// now apply the step
//
  if (GCurrentSweep.IsSweepingL)      // if inductance, increment GCurrentL
  {
    if (GCurrentSetting.LValue == GCurrentSweep.MaxSteppedValue)                     // if reached end
      Result = false;
    else
    {
      NewValue = (int)GCurrentSetting.LValue;
      NewValue = constrain(NewValue + GCurrentSweep.StepSize, GCurrentSweep.MinSteppedValue, GCurrentSweep.MaxSteppedValue);
      GCurrentSetting.LValue = (byte)NewValue;
    }
  }
  else                                                      // else if capacitance, increment GCurrentC
  {
    if (GCurrentSetting.CValue == GCurrentSweep.MaxSteppedValue)                     // if reached end
      Result = false;
    else
    {
      NewValue = (int)GCurrentSetting.CValue;
      NewValue = constrain(NewValue + GCurrentSweep.StepSize, GCurrentSweep.MinSteppedValue, GCurrentSweep.MaxSteppedValue);
      GCurrentSetting.CValue = (byte)NewValue;
    }
  }
  return Result;                                            // false if end of current step
}


void SetupNextSweep(byte SweepRange)
{
  if(GCurrentSweep.IsSweepingL)
  {
    GCurrentSweep.FixedParam = GBestFoundSoFar.CValue;        // fixed at current capacitance value
    GCurrentSweep.MinSteppedValue = constrain(GBestFoundSoFar.LValue - SweepRange, 0, GTuneParamArray[GFreqRow].LMax); // current inductance value +/-
    GCurrentSweep.MaxSteppedValue = constrain(GBestFoundSoFar.LValue + SweepRange, 0, GTuneParamArray[GFreqRow].LMax); // current inductance value +/-
  }
  else                                                        // we are sweeping C
  {
    GCurrentSweep.FixedParam = GBestFoundSoFar.LValue;        // fixed at current inductance value
    GCurrentSweep.MinSteppedValue = constrain(GBestFoundSoFar.CValue - SweepRange, 0, GTuneParamArray[GFreqRow].CMax); // current capacitance value +/-
    GCurrentSweep.MaxSteppedValue = constrain(GBestFoundSoFar.CValue + SweepRange, 0, GTuneParamArray[GFreqRow].CMax); // current capacitance value +/-
  }
//
// finally get 1st algorithm condition of the new sweep
//
  InitialiseCurrentFromSweep();                                   // will be sent to h/w at the end
}





//
// function algorithm code periodic tick
//
void AlgorithmTick(void)
{
  bool ValidNewRow;                                     // true if new stage 1 row available
  bool ValidNewStep = false;                            // set true if not yet time to move onto next step
  byte SweepRange;                                      // sweep range
  String Text;

//
// see if terminated mid-tune by the PC
//
  if((GTuneActive == false) && (GAlgState != eAlgIdle))
  {
    GAlgState = eAlgIdle;
    LCU_UI_SetTuning(false);
  }
  
//
// if executing algorithm, get VSWR and store if better than last
// then try advance to new step
//
  if ((GAlgState != eAlgIdle) && (GAlgState != eAlgEEPROMWrite))
  {
    GCurrentSetting.VSWR = GetVSWR();
    if (GCurrentSetting.VSWR < GBestFoundSoFar.VSWR)
    {
      GBestFoundSoFar = GCurrentSetting;
      GBestFoundSoFar.IsSweepingL = GCurrentSweep.IsSweepingL;
    }
#ifdef CONDITIONAL_ALG_DEBUG
    Text = GetStateName();
    PrintSolution(Text, true);                // print current step and its VSWR
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
    case eAlgIdle:                                    // do nothing in idle!
      break;

    case eAlgCoarse1: 
      if(!ValidNewStep)                               // if we have exhausted current search, try new row
      {
        ValidNewRow = GetStage1Row(false);            // try move to next row. 
        if(!ValidNewRow)                              // if this fails, change state
        {
          // we need to construct a sweep set for stage 1b
          GAlgState = eAlgCoarse2;
          GCurrentSweep.IsHighZ = GBestFoundSoFar.HighZ;                  // copy best Z setting
          GCurrentSweep.IsSweepingL = !GBestFoundSoFar.IsSweepingL;       // opposite sweep needed now
          GCurrentSweep.StepSize = GTuneParamArray[GFreqRow].Stage1bStep;
          if(GCurrentSweep.IsSweepingL)
          {
            GCurrentSweep.MinSteppedValue = 0;                            // stage 1b will start at 0
            GCurrentSweep.MaxSteppedValue = GTuneParamArray[GFreqRow].LMax;
            GCurrentSweep.FixedParam = GBestFoundSoFar.CValue;            // if we now sweep L, copy C value from best so far
          }
          else
          {
            GCurrentSweep.MinSteppedValue = 0;                            // stage 1b will start at 0
            GCurrentSweep.MaxSteppedValue = GTuneParamArray[GFreqRow].CMax;
            GCurrentSweep.FixedParam = GBestFoundSoFar.LValue;            // if we now sweep C, copy L value from best so far
          }
          InitialiseCurrentFromSweep();                                   // will be sent to h/w at the end
        }
      }
      break;

    case eAlgCoarse2:
      if(!ValidNewStep)                                                   // take best found and set up mid sweep
      {
        // we need to construct a sweep set for stage 2 1st mid sweep; keep the Z setting
        GAlgState = eAlgMid1;
        GCurrentSweep.IsSweepingL = !GCurrentSweep.IsSweepingL;           // opposite sweep needed now
        GCurrentSweep.StepSize = GTuneParamArray[GFreqRow].Stage2MidStep;
        SweepRange = GTuneParamArray[GFreqRow].Stage2MidRange;
        SetupNextSweep(SweepRange);                                       // set sweep range parameters
      }
      break;

    case eAlgMid1:
      if(!ValidNewStep)                                                   // take best found and set up 2nd mid sweep
      {
        // we need to construct a sweep set for stage 2 1st mid sweep; keep the Z setting
        GAlgState = eAlgMid2;
        GCurrentSweep.IsSweepingL = !GCurrentSweep.IsSweepingL;           // opposite sweep needed now
        GCurrentSweep.StepSize = GTuneParamArray[GFreqRow].Stage2MidStep;
        SweepRange = GTuneParamArray[GFreqRow].Stage2MidRange;
        SetupNextSweep(SweepRange);                                       // set sweep range parameters
      }
      break;

    case eAlgMid2:
      if(!ValidNewStep)                                                   // take best found and set up fine sweep
      {
        // we need to construct a sweep set for stage 2 1st mid sweep; keep the Z setting
        GAlgState = eAlgFine1;
        GCurrentSweep.IsSweepingL = !GCurrentSweep.IsSweepingL;           // opposite sweep needed now
        GCurrentSweep.StepSize = 1;
        SweepRange = GTuneParamArray[GFreqRow].Stage2FineRange;
        SetupNextSweep(SweepRange);                                       // set sweep range parameters
      }
      break;

    case eAlgFine1: 
      if(!ValidNewStep)                                                   // take best found and set up 2nd fine sweep
      {
        // we need to construct a sweep set for stage 2 1st mid sweep; keep the Z setting
        GAlgState = eAlgFine2;
        GCurrentSweep.IsSweepingL = !GCurrentSweep.IsSweepingL;           // opposite sweep needed now
        GCurrentSweep.StepSize = 1;
        SweepRange = GTuneParamArray[GFreqRow].Stage2FineRange;
        SetupNextSweep(SweepRange);                                       // set sweep range parameters
      }
      break;

    case eAlgFine2:
      if(!ValidNewStep)                                                   // finished final stage
        AssessTune();
      break;

    case eAlgEEPROMWrite:
      GAlgState = eAlgIdle;                         // finished calculating
      GTuneActive = false;
      LCU_UI_SetTuning(false);
      break;
  }
//
// finally send L,C, low/high Z switch setting to hardware if we are in a search state
//  
  if ((GAlgState != eAlgIdle) && (GAlgState != eAlgEEPROMWrite))
    SendCandidateSolution(true);
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
// At the moment, this is always called with a quick tune first.
// the frequency will already have been set/
//
// for quick tune: set to state fine 1, sweeping L according to sweep range in frequency dependent algorithm table
//
// for full tune: set 1st sweep of the stage 1 parameter table
//
void InitiateTune(bool StartQuick)
{
  bool GotRow;                                          // true if found 1st row of algorithm
  byte SweepRange;                                           // tune range

  if (StartQuick)
  {
    GIsQuickTune = true;                                // set "this is a quick tune attempt"
    GAlgState = eAlgFine1;                              // set state
//
// create the sweep definition; sweep L first
//
    SweepRange = GTuneParamArray[GFreqRow].Stage2FineRange;
    GCurrentSweep.IsHighZ = GetHiLoZ();
    GCurrentSweep.IsSweepingL = true;
    GCurrentSweep.StepSize = 1;                         // fine step
    GCurrentSweep.FixedParam = GetCapacitance();        // fixed at current capacitance value
    GCurrentSweep.MinSteppedValue = constrain(GetInductance() - SweepRange, 0, GTuneParamArray[GFreqRow].LMax); // current inductance value +/-
    GCurrentSweep.MaxSteppedValue = constrain(GetInductance() + SweepRange, 0, GTuneParamArray[GFreqRow].LMax); // current inductance value +/-
//
// finally get 1st algorithm condition of the new sweep
//
    InitialiseCurrentFromSweep();                                   // will be sent to h/w at the end
  }
  else                                                                // start a normal full tune
  {
// for full tune: set L=0; Step C=0-255 at coarse step
    GIsQuickTune = false;                                             // cancel quick tune state
    GAlgState = eAlgCoarse1;                                          // set state
//
// copy out the first row of the stage 1 algorithm table
//
    GotRow = GetStage1Row(true);
  }
  GTuneActive = true;                                               // set active
  SendCandidateSolution(true);                                      // drive hardware
  GBestFoundSoFar.VSWR = VMAXVSWR;                                  // initialise VSWR best value found = worst possible
}
