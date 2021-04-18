/////////////////////////////////////////////////////////////////////////
//
// Aries ATU controller sketch by Laurence Barker G8NJJ
// this sketch controls an L-match ATU network
// with a CAT interface to connect to an HPSDR control program
// copyright (c) Laurence Barker G8NJJ 2019
//
// the code is written for an Arduino Nano 33 IoT module
//
// algorithm.h:  tuning algorithm to find ATU tune solution
/////////////////////////////////////////////////////////////////////////
#ifndef __tunealgorithm_h
#define __tunealgorithm_h

extern bool GTuneActive;              // bool set true when algorithm running. Clear it to terminate.
extern bool GQuickTuneEnabled;         // true if quick tune allowed


//
// function to initialise algorithm code
//
void InitialiseAlgorithm(void);


//
// function algorithm code periodic tick
// this is a (~16ms) tick handler to execute one step.

void AlgorithmTick(void);


//
// find the frequency row to use
// sets the row variable for tuning parameters to use
// paramters is the required frequency (units of MHz)
// 
void FindFreqRow(byte FrequencyMHz);


//
// set frequency to be used by the tuning algorithm
// used to set limits to search
//
void AlgorithmSetFrequency(byte FreqMHz);

//
// function to initiate a tune algorithm sequence
// this initialises the data structures so that a series of timer ticks will step through the cycle
// StartQuick if the algorithm should try a "quick tune" first
//
void InitiateTune(bool StartQuick);


//
// cancel algorithm
//
void CancelAlgorithm(void);



#endif
