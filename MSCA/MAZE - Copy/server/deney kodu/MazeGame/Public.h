#pragma once

// C/C++
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <assert.h>
#include <memory.h> 
#include <string.h>
#include <time.h>
#include <math.h>
#include <tchar.h>
#include <limits.h>
#include <process.h>
#include <dos.h> 
#include <conio.h>
#include <windows.h>
#include <mmsystem.h> // for PlaySound method
#include <vector>
using namespace std;

#include "MathCB.h"

// INVENTOR
#include <Inventor/SoDB.h>
#include <Inventor/Win/SoWin.h> 
#include <Inventor/SoInput.h>
#include <Inventor/Win/viewers/SoWinExaminerViewer.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoDirectionalLight.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoCamera.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/events/SoKeyboardEvent.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoUnits.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoCylinder.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/draggers/SoDragger.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/nodes/SoTexture2.h>
#include <Inventor/nodes/SoAsciiText.h>
#include <Inventor/nodes/SoText2.h>
#include <Inventor/nodes/SoText3.h>
#include <Inventor/nodes/SoSwitch.h>
#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoSelection.h>
#include <Inventor/manips/SoTransformBoxManip.h>
#include <Inventor/nodekits/SoWrapperKit.h>
#include <Inventor/fields/SoMFString.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoFont.h>
#include <Inventor/nodes/SoDrawStyle.h>

//open haptics lib's
#include <HD/hd.h>
#include <HDU/hduRecord.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>



#include "DataRecord.h"
//#include "ForceAngleRec.h"


#include <iostream>
#include <fstream>
using namespace std;


typedef SbVec3f Point;
typedef SbVec3f Vector;

#define DEVICE1 "Left"  //phantom1"//Omnigrey"
//#define DEVICE2 "Omni"

#define DOWN 1   
#define UP 0

#define BLUE 0		// x axis
#define GREEN 1		// z axis

#define HIP_RANDPOS 15	// max distance from CIP
#define RECORD_DATA 
//#define RECORD_FORCE
//#define START_TARGET

#define G	9.81f



// STATIC GLOBALS
#define PHANTOM_PREMIUM		// define if using a phantom premium device, 
// otherwise assumed to be using phantom omni 
#ifdef PHANTOM_PREMIUM
#define F_SCALE_FACTOR 0.7f // factor to scale force in case it is greater than FORCE_LIMIT
static HDdouble FORCE_LIMIT = 4.0f; /// F_SCALE_FACTOR;
#else
#define F_SCALE_FACTOR 1.0f // factor to scale force in case it is greater than FORCE_LIMIT
static HDdouble FORCE_LIMIT = 2.0f;
#endif

#define _CIP			0
#define _HIP			1

#define _CIP2			2
#define _HIP2			3

#define COLLISION_ON
#define ROTATION_ON
#define LINE_COLL 
//#define FORCEVECTOR_ON

static HDfloat	KP_LIMIT; // stiffness 
static HDfloat	KD_LIMIT; // damping 

//#define PRINTANGLE
const int gravityBlendFactor = 2;
const float gravityBlendStep = 20;

//#ifdef DO_INITIAL_FREE_TRIAL
//#ifdef PLAYONCE
//const int NUM_TRIALS_PER_COND = 2;
//#else
//const int NUM_TRIALS_PER_COND = 15;//cigil 11 di
//#endif
//#else
//#ifdef PLAYONCE
//const int NUM_TRIALS_PER_COND = 1;
//#elif MIXED_SCENE
//const int NUM_TRIALS_PER_COND = 21;//cigil 5;
//#else
//const int NUM_TRIALS_PER_COND = 10;//cigil 5;
//#endif
//#endif

const int NUM_TRIALS_PER_COND_S = 16;//9;//10;
const int NUM_TRIALS_PER_COND_M = 21;//11;//12;

#define COLOR_GREEN				Vector(0.0f, 1.0f, 0.0f)
#define COLOR_BLUE				Vector(0.0f, 0.0f, 1.0f)
#define COLOR_GREY				Vector(0.8f, 0.8f, 0.8f)

#define COLOR_GREEN_MATTE		Vector(0.6f, 1.0f, 0.1f)
#define COLOR_BLUE_MATTE		Vector(0.1f, 0.3f, 0.8f)
#define COLOR_GREEN_PALE		Vector(0.0f, 0.6f, 0.0f)



#define COLOR_BOARD				Vector(0.8f, 0.9f, 0.6f)
#define COLOR_BOARD_BOUNDARY	Vector(0.3f, 0.5f, 0.08f)

#define COLOR_CIP				COLOR_GREEN//Vector(0.0f, 0.95f, 0.3f)
#define COLOR_HIP				COLOR_BLUE //Vector(0.0f, 0.35f, 0.8f)
#define COLOR_NIP				Vector(0.5f, 0.8f, 0.2f)
#define gravitation             9.80665f
#define FULL_SCREEN
//#define PRINT_FORCE
//#define PRINT_CORNERS
//#define HIP_CONTROL_ON

//#define DYNAMIC_FRICTION_ON
#define STATIC_FRICTION_ON

const int	SCHEDULER_RATE		= 1000;

const float DELTA_T				= 0.001f;

const float BLEND_TIME			= (SCHEDULER_RATE == 500) ? 250.0f : 300.0f;
const int	BLEND_TICK_COUNT	= 2 * 5; // # of markers used for role exchange visualization 

const float BOARD_WIDTH			= 200.0f; // Onur : Training board width 
const float BOARD_DEPTH_STR     = 30.0f;//15.0f;
const float BOARD_DEPTH_MIXED	= 82.0f;//15.0f; // Onur : Training board height
const float BOARD_RADIUS		= 20.0f;//10.0f; 

const float BOARD_HEIGHT		= 5.0f; 
const float IP_RADIUS			= 2.0f;
const float IP_TARGET_HEIGHT    = 2.0f;
const float NIP_RADIUS			= 0.005f;
const float transhc =0.5f;
const float transn =0.05f;
const float BOARD_THC			= 2.0f;

#ifdef STATIC_FRICTION_ON
const float COEFFICIENT_FRICTION_STATIC          = 0.19f;
const float COEFFICIENT_FRICTION_KINETIC	     = 0.15f;
const float COEFFICIENT_FRICTION_ROT_STATIC      = 0.20f;
const float COEFFICIENT_FRICTION_ROT             = 0.19f;
#else
const float COEFFICIENT_FRICTION_STATIC   = 0.0f;
const float COEFFICIENT_FRICTION_ROT      = 0.0f;
#endif

#ifdef DYNAMIC_FRICTION_ON
const float COEFFICIENT_FRICTION_DYNAMIC  = 1.0f;
#else
const float COEFFICIENT_FRICTION_DYNAMIC  = 0.0f;
#endif
//Onur

const double translateforceConstant = 1.0f;
const double rotationForceConstant = 1.0f;
//const double translateForceConstants[10]	= {0.2,0.4,0.6,0.8,1.0,1.2,1.4,1.6,1.8,2.0};
//const double rotationForceConstants[10]	= {0.2,0.4,0.6,0.8,1.0,1.2,1.4,1.6,1.8,2.0};




//agents need to wait on target for 5 seconds
const int TIME_TO_WAIT_ON_TARGET = 5000;
//agents need to complete the trial in 1 min for the straight scenario
const int TIME_TO_COMPLETE_TRIAL_STRAIGHT = 60000; 
//agents need to complete the trial in 1/2 min for the rotational scenario
const int TIME_TO_COMPLETE_TRIAL_ROTATIONAL = 30000; 
//agents need to complete the trial in 2 min for the mixed scenario
const int TIME_TO_COMPLETE_TRIAL_MIXED = 120000; 

#define LONGEST_SPRING_LENGTH_HIP		FORCE_LIMIT / kpHN
#define LONGEST_SPRING_LENGTH_CIP		FORCE_LIMIT / kpCN
const float   MAX_SPRING_DIST = 37.0f;


//Onur : Important things
const float BALL_WIDTH  = 8.0f;//8.0f;
const float BALL_HEIGHT = 4.0f;
const float BALL_DEPTH  = 9.0f;//3.0f;//16.0f;//8.0f; //changed
const float BALL_MASS   = 4.0f;//10.0f;
//Important things


const float BALL_RADIUS			= 4.66f;	

const float TARGET_HEIGHT		= BALL_HEIGHT + 2;
const float TARGET_WIDTH		= BALL_WIDTH + 2;
const float TARGET_DEPTH		= BALL_DEPTH + 2;



//AYSE: cancel all shifts
const float SHIFT_Z	=0;//-36.0f;//-60.0f;
const float SHIFT_X	=0;//-40.00f;//30.0f;


const float wallTx = 50.0f;//13.0f;//-20.0f;//-40.0f;
const float wallTz = 0.0f;//-5.0f+SHIFT_Z;//-40.0f;
const float wallTx2 = -65.0f;//57.0f;//20.0f;
const float wallTy = -2.0f;
const float wallTz2 = 28.0f;
const float wallHeight = 5.0f;
const float wallWidth = 100.0f;
const float wallDepth = 30.0f;
const float wallWidth23 = 70.0f;
const float wallDepth23 = 26.0f;
const float wallTx3 = -65.0f;//35.0f;//-20.0f;//-40.0f;
const float wallTz3 = -28.00f;//-5.0f+SHIFT_Z;//-40.0f;

const float INITIAL_TARGET		= 45.0f;
const float TARGET_REACH_EPS_X	= TARGET_WIDTH / 7; // we can assume target is reached within +- 5 pts.
const float TARGET_REACH_EPS_Z	= TARGET_HEIGHT / 7; // we can assume target is reached within +- 5 pts.
const float HORIZONTAL_ALIGN_EPS = (10.0f/180.0f) * PI; // we can assume the target is horizontal reached within +- 10 rads.

const int   expNo=1;

/* AYSE: Render two simple horizontal planes to contstrain user movement to x-z plane. */
const float HORIZONTAL_PLANE_L = 40.0f;
const float HORIZONTAL_PLANE_U = 60.0f;


const float middleGap = BALL_WIDTH * 4 / 5.0; // the gap between two obstacles
const float obstacleDepth = BOARD_RADIUS - middleGap/2;

const int PERMS[5][7]= {{0, 1, 2, 3, 4, 5, 6},// 7},
					    {1, 2, 3, 4, 5, 6, 0},// 1},
					    {2, 3, 4, 5, 6, 0, 1},//, 2},
					    {4, 5, 6, 0, 1, 2, 3},//, 4},
                        {6, 0, 1, 2, 3, 4, 5}};//, 6}};


const int PERMS_cond[5][7] = {{5, 1, 2, 3, 1, 4, 1},//, 5},
                             {1, 2, 3, 1, 4, 1, 5},//, 1},
                             {2, 3, 1, 4, 1, 5, 1},//, 2},
                             {1, 4, 1, 5, 1, 2, 3},//, 1},
                             {1, 5, 1, 2, 3, 1, 4}};//, 1}};



const int practice_PERMS[5][2] = {{1, 2},// 7},
                                  {2, 1},// 1},
                                  {1, 2},//, 2},
                                  {1, 0},//, 4},
                                  {2, 1}};//, 6}};


const int practice_PERMS_cond[5][2] = {{1 , 1},//, 5},
                                       {1 , 1},//, 1},
                                       {1 , 1},//, 2},
                                       {1 , 1},//, 1},
                                       {1 , 1}};//, 1}};

const int PRACTICE_COUNT  =  2; // practice trial number for the case that the game and practice g,ven together
const int PRACTICE_COUNTS = 2; // it is the magnitude for practice mode

const int PERMS_MIXED_COND [5][10]  ={ {1, 1, 2, 1, 3, 1, 4, 1, 5, 1},
                                 {2, 1, 3, 1, 4, 1, 5, 1, 1, 1},
                                 {3, 1, 4, 1, 5, 1, 1, 1, 2, 1},
								 {4, 1, 5, 1, 1, 1, 2, 1, 3, 1},
								 {5, 1, 1, 1, 2, 1, 3, 1, 4, 1}};

const int PERMS_MIXED [5][10]  ={ {1, 0, 2, 0, 3, 0, 4, 0, 5, 0},
                                 {2, 0, 3, 0, 4, 0, 5, 0, 1, 0},
                                 {3, 0, 4, 0, 5, 0, 1, 0, 2, 0},
								 {4, 0, 5, 0, 1, 0, 2, 0, 3, 0},
								 {5, 0, 1, 0, 2, 0, 3, 0, 4, 0}};

const int CONDITIONAL_HARMONY = 6 ;
const int STRAIGHT_TIME_UP = 55000;
const int MIXED_TIME_UP = 115000;