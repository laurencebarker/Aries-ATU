/////////////////////////////////////////////////////////////////////////
//
// Andromeda front panel controller by Laurence Barker G8NJJ
// this sketch provides a knob and switch interface through USB and CAT
//
//
// CAT handler.cpp
// this file holds the CAT handling code
// responds to parsed messages, and initiates message sends
// this is the main body of the program!
/////////////////////////////////////////////////////////////////////////

#include "globalinclude.h"
#include "cathandler.h"
#include <stdlib.h>


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




//
// function to send back a hardware version message
//
void MakeHardwareVersionMessage(void)
{
  int Version;

  Version = HWVERSION;
  MakeCATMessageNumeric(eZZZH,Version);
}


//
// function to send back a software version message
//
void MakeSoftwareVersionMessage(void)
{
  int Version;

  Version = SWVERSION;
  MakeCATMessageNumeric(eZZZS,Version);
}


//
// function to send back a product ID message
//
void MakeProductIDMessage(void)
{
  int Version;

  Version = PRODUCTID;
  MakeCATMessageNumeric(eZZZT,Version);
}


//
// function to send back a tune success message
//
void MakeTuneSuccessMessage(bool Result)
{
  MakeCATMessageBool(eZZOX,Result);
}


//
// handle TUNE on/off CAT message from PC
//
void SetTuneOnOff(bool State)
{
  
}


//
// handle ATU on/off CAT message from PC
//
void SetATUOnOff(bool State)
{
  
}


//
// handle frequency change CAT message from PC
//
void SetNewFrequency(char* FreqString)
{
  
}


//
// handle a change of antenna CAT command from PC
//
void HandleAntennaChange(int Antenna)
{
  
}


//
// handle an erase solution CAT command from PC
//
void HandleEraseSolutions(int Antenna)
{
  
}


//
// handle an L/C fine tune CAT command from PC
//
void HandleLCFineTune(int Command)
{
  
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
    case eZZOC:                                                       // Antenna change
      HandleAntennaChange(ParsedParam);
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
    case eZZZH:                                                       // h/w version reply
      MakeHardwareVersionMessage();
      break;

    case eZZZS:                                                       // s/w version reply
      MakeSoftwareVersionMessage();
      break;

    case eZZZT:                                                       // RX2 stereo balance
      MakeProductIDMessage();
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
