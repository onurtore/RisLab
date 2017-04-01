#pragma once

#include "Public.h"
#include "MathCB.h"

// HapticCallBack: For updating haptic loop and generating forces
class  HapticCallBack
{
public:
	HHD hHD1;
	HHD hHD2;

	bool keepHapticLoop;
	SbVec3f myF; 
	SbVec3f myNewTranslation;
	SbVec3f myNewTranslation01;

	// AYSE: move here
	// for outputs
	int curTrial;

	// AYSE: role exchange variables
	bool tiltOn;	/* this toggles on/off the tilting of the board */
	bool hipsOn;	/* this toggles on/off rendering of the hips */
	bool vectorOn;
	bool RvectorOn;
	bool scoreTextOn;
	bool warningOn;
	bool soundOn;


	// AYSE: VISUALIZATION VARIABLES
	
	bool roleIconZoomOn;
	
	bool scorePrintedOnScreen; 
	
	bool hitWalls;

	int activeForces[4];
	bool isGameFinished;
	bool guidedStyli[2]; // for each stylus, the computer guides or not
	bool userControlledStyli[2]; // for each stylus, the user can control the axis or not

	bool separateAxes; 

	HapticCallBack();
	virtual int initialize_phantom(bool myboolean);
	//virtual HDCallbackCode HDCALLBACK MyHapticLoop(void *pUserData);
	//virtual void dummyHapticLoop();
	virtual void stopHaptics();
	virtual int update_stylus_position(Point pt);
	int isGameOver(); // AYSE: returns true if all targets in scene are hit 
	// AYSE: generates force for buzzing effect to be activated when role exchange occurs
	// buzzing is active iff boolOn == true and userWantsControl == false, i.e. computer
	// requests control.

	/* Helper Functions to prevent warm motors. */
	void PrintMotorTemp(const HDdouble *aMotorTemp, HDint nNumMotors);
	void PreventWarmMotors(hduVector3Dd force);


};