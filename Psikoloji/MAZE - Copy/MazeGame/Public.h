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

// INVENTOR
#include <Inventor/SoDB.h>
#include <Inventor/Win/SoWin.h> 
#include <Inventor/SoInput.h>
#include <Inventor/Win/viewers/SoWinExaminerViewer.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoDirectionalLight.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoCamera.h>

#include <Inventor/nodes/SoDirectionalLight.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/Win/SoWinRenderArea.h>

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
#include <HDU/hduVector.h>



#include "DataRecord.h"
//#include "ForceAngleRec.h"


#include <iostream>
#include <fstream>
using namespace std;


typedef SbVec3f Point;
typedef SbVec3f Vector;

#define DEVICE1 "Device1"
#define DEVICE2 "Device2"

#define DOWN 1   
#define UP 0

#define BLUE 0		// x axis
#define GREEN 1		// z axis

#define HIP_RANDPOS 15	// max distance from CIP
#define RECORD_DATA 
//#define RECORD_FORCE
//#define START_TARGET
#define G 9.81
#define PI 3.1415926535897932384626433832795
#define COEFFICIENT_FRICTION  0//0.0008
// STATIC GLOBALS
//#define PHANTOM_PREMIUM		// define if using a phantom premium device, 
							// otherwise assumed to be using phantom omni 
//#ifdef PHANTOM_PREMIUM
//#define FORCE_LIMIT 4.0		// 4.0 for phantom premium
//#define F_SCALE_FACTOR 2.0f // factor to scale force in case it is greater than FORCE_LIMIT
//#else
//#define FORCE_LIMIT 2.2		// 2.2 for phantom omni
//#define F_SCALE_FACTOR 2.0f // factor to scale force in case it is greater than FORCE_LIMIT
//#endif

#define _CIP			0
#define _HIP			1

#define _CIP2			2
#define _HIP2			3

#define COLLISION_ON
#define ROTATION_ON
#define LINE_COLL 
//#define FORCEVECTOR_ON
static HDdouble FORCE_LIMIT = 2.0f;
static HDfloat	KP_LIMIT; // stiffness 
static HDfloat	KD_LIMIT; // damping 


#define F_SCALE_FACTOR 1.f//0.75f // factor to scale force in case it is greater than FORCE_LIMIT
//#define PRINTANGLE
const int gravityBlendFactor = 2;
const float gravityBlendStep = 20;

#ifdef DO_INITIAL_FREE_TRIAL
	#ifdef PLAYONCE
		#define NUM_TRIALS_PER_COND 2
	#else
		#define NUM_TRIALS_PER_COND 6
	#endif
#else
	#ifdef PLAYONCE
		#define NUM_TRIALS_PER_COND 1
	#else
		#define NUM_TRIALS_PER_COND 5
	#endif
#endif

#define COLOR_GREEN				Vector(0.0, 1.0, 0.0)
#define COLOR_BLUE				Vector(0.0, 0.0, 1.0)
#define COLOR_GREY				Vector(0.8, 0.8, 0.8)

#define COLOR_BOARD				Vector(0.8f,0.9f,0.6f)
#define COLOR_BOARD_BOUNDARY	Vector(0.7f,0.8f,0.4f)

#define COLOR_CIP				COLOR_GREEN//Vector(0.0f, 0.95f, 0.3f)
#define COLOR_HIP				COLOR_BLUE //Vector(0.0f, 0.35f, 0.8f)
#define COLOR_NIP				Vector(0.5f, 0.8f, 0.2f)
#define gravitation             9.80665
//#define FULL_SCREEN
//#define PRINT_FORCE
//#define PRINT_CORNERS
//#define HIP_CONTROL_ON
const int	SCHEDULER_RATE		= 1000;

const float BLEND_TIME			= (SCHEDULER_RATE == 500) ? 250.0f : 300;
const int	BLEND_TICK_COUNT	= 2 * 5; // # of markers used for role exchange visualization 

const float BOARD_WIDTH			= 140.0f;
const float new_width			=7.0f;
const float new_depth			=10.0f;
const float BOARD_HEIGHT			= 5.0f;
const float IP_RADIUS			= 2.0f;
const float NIP_RADIUS			= 0.005f;
const float transhc =0.5f;
const float transn=0.05f;
const float BOARD_THC			= 2.0f;
const float BALL_RADIUS			= 4.66f;	// AYSE: enlarge ball. original: 0.8f;






const float bRadius=4.5f;
const float ball_width=8.0f;
const float ball_height=4.0f;
const float ball_depth=3.0f;//16.0f;//8.0f; //changed


 const  float fw	=-38.0f;//-60.0f;
 const	float fwx	=58.00f;//-48.0f;

 const float PI2=3.1415926535897932384626433832795f;
const float wallTx=12.0f;//13.0f;//-20.0f;//-40.0f;
const	float wallTz=-48.0f;//-5.0f+fw;//-40.0f;
const float wallTx2=56.0f;//57.0f;//20.0f;
const float wallTy=2.0f;
const	float wallTz2=-48.0f;//-10.0f;
const	float wallHeight=10.0f;
const	float wallWidth=8.0f;//20
const	float wallDepth=40.0f; //100.0f;//40//32
const float wallTx3=34.0f;//35.0f;//-20.0f;//-40.0f;
const	float wallTz3=-12.00f;//-5.0f+fw;//-40.0f;
const float OFFSET=170.0f;
const float INITIAL_TARGET=30.0f;
const float TARGET_DIM=10.0f;