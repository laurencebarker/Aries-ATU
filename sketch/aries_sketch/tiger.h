/////////////////////////////////////////////////////////////////////////
//
// Aries ATU controller sketch by Laurence Barker G8NJJ
// this sketch controls an L-match ATU network
// with a CAT interface to connect to an HPSDR control program
// copyright (c) Laurence Barker G8NJJ 2019
//
// the code is written for an Arduino Nano Every module
//
// tiger.h
// this file holds the CAT parsing code
// large CAT file = tiger....
/////////////////////////////////////////////////////////////////////////
#ifndef __tiger_h
#define __tiger_h
#include <Arduino.h>


//
// firstly an enumerated list of all of the CAT commands
// ordered as per documentation, not alphsabetically!
enum ECATCommands
{
  eZZTU,                          // TUNE on/off (from PC to Arduino)
  eZZTV,                          // frequency change (from PC to Arduino)
  eZZOC,                          // antenna change (from PC to Arduino)
  eZZOZ,                          // erase tuning solutions (from PC to Arduino)
  eZZZE,                          // other encoder for fine tune L/C
  eZZOX,                          // Tune success (from Arduino to PC)
  eZZOV,                          // ATU enable (from PC to Arduino)
  eZZZS,                          // s/w version
  eNoCommand                      // this is an exception condition
};


typedef enum
{
  eNone,                          // no parameter
  eBool,                          // boolean parameter
  eNum,                           // numeric parameter     
  eStr                            // string parameter
}ERXParamType;



//
// this struct holds a record to describe one CAT command
//
struct SCATCommands
{
  char* CATString;                // eg "ZZAR"
  ERXParamType RXType;            // type of parameter expected on receive
  int MinParamValue;              // eg "-999"
  int MaxParamValue;              // eg "9999"
  byte NumParams;                 // number of parameter bytes in a "set" command
  bool AlwaysSigned;              // true if the param version should always have a sign
};



extern SCATCommands GCATCommands[];

//
// initialise CAT handler
//
void InitCAT(void);


//
// ScanParseSerial()
// scans input serial stream for characters; parses complete commands
// when it finds one
//
void ScanParseSerial(void);



//
// ParseCATCmd()
// Parse a single command in the local input buffer
// process it if it is a valid command
//
void ParseCATCmd(void);

//
// create CAT message:
// this creates a "basic" CAT command with no parameter
// (for example to send a "get" command)
//
void MakeCATMessageNoParam(ECATCommands Cmd);


//
// make a CAT command with a numeric parameter
//
void MakeCATMessageNumeric(ECATCommands Cmd, long Param);

//
// make a CAT command with a bool parameter
//
void MakeCATMessageBool(ECATCommands Cmd, bool Param);

//
// make a CAT command with a string parameter
// the string is truncated if too long, or padded with spaces if too short
//
void MakeCATMessageString(ECATCommands Cmd, char* Param);



#endif //not defined
