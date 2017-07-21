#pragma once

#include "Public.h"
#include "MathCB.h"

typedef SbVec3f Point;
typedef SbVec3f Vector;

// HapticCallBack: For updating haptic loop and generating forces
class  HapticCallBack
{
public:
	bool keepHapticLoop;
	SbVec3f myF; // ozgur - gstVector myF;
	SbVec3f myNewTranslation;
	SbVec3f myNewTranslation01;

	// AYSE: move here
	// for outputs
	int curTrial;

	// AYSE: role exchange variables
	bool soundOn;	/* this creates a sound whenever role exchange occurs */
	bool buzzOn;	/* this creates a buzz effect whenever role exchange occurs */
	bool tremorOn;	/*	this creates a tremor effect with random amplitude 
						and frequency while computer control is dominant */
	bool tiltOn;	/* this toggles on/off the tilting of the board */
	bool hipsOn;	/* this toggles on/off rendering of the hips */
	

	// AYSE: VISUALIZATION VARIABLES
	bool scoreTextOn;
	bool roleIconZoomOn;
	
	bool scorePrintedOnScreen; 

	// used to set the control algorithm the guidance subsystem uses
	int guidanceFlag;
	int guidanceMethod;
	int activeForces[4];
	bool isGameFinished;
	bool guidedStyli[2]; // for each stylus, the computer guides or not
	bool userControlledStyli[2]; // for each stylus, the user can control the axis or not


	bool userFlaggedMoment; // put a flag whenever the user presses F key, used for debugging purposes
	bool separateAxes; 

	bool fellInPit;
	bool waitOnTarget;
	int	 pitfallCount;

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
	void generateBuzzForce(bool userWantsControl = false);
	// AYSE: clears force for buzzing effect 
	void clearBuzzForce();
	// AYSE: generates force for tremor effect to be activated when computer is in control
	void generateTremorForce();
	// AYSE: clears force for tremor effect 
	void clearTremorForce();

	void displayEffects(bool ctrlMode, bool userGivesCtrl);
	
	/* Helper Functions to prevent warm motors. */
	void PrintMotorTemp(const HDdouble *aMotorTemp, HDint nNumMotors);
	void PreventWarmMotors(hduVector3Dd force);

};