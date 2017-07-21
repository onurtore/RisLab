#pragma once
/* list of public functions  */
#ifndef _PUBLIC_H_
#define _PUBLIC_H_

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
// ozgur - #include <iostream.h>
#include <process.h>
#include <dos.h> 
#include <conio.h>
#include <windows.h>
#include <mmsystem.h> // for PlaySound method
#include <vector>
// INVENTOR
#include <Inventor/SoDB.h>
#include <Inventor/Win/SoWin.h> 
#include <Inventor/SoInput.h>
#include <Inventor/Win/viewers/SoWinExaminerViewer.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoDirectionalLight.h>
#include <Inventor/annex/FXViz/nodes/SoShadowGroup.h>
#include <Inventor/annex/FXViz/nodes/SoShadowSpotLight.h>
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

#include "SoundPlayerThread.h"
//#include "SimHapCallback.h"
//#include "HapticCallBack.h"
//#include "DataRecord.h"

#include <iostream>
#include <fstream>
using namespace std;

#define DOWN 1   
#define UP 0

// guidance options
#define GUIDE_IN_GIVEN_ORDER 0

#define BLUE 0		// x axis
#define GREEN 1		// z axis

#define HIP_RANDPOS 15	// max distance from CHIP
#define RECORD_DATA 

// ctrl modes
#define USER_CTRL 0
#define BLND_CTRL 1
#define BOTH_CTRL 2
//#define USE_F_MAG

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

static HDdouble FORCE_LIMIT = 2.2;
static HDfloat	KP_LIMIT; // stiffness 
static HDfloat	KD_LIMIT; // damping 


#define F_SCALE_FACTOR 1.0f // 0.75f // factor to scale force in case it is greater than FORCE_LIMIT


const int gravityBlendFactor = 2;
const float gravityBlendStep = 20;

#define TARGET_COUNT		4//*(2*gravityBlendFactor-1)
#define OBSTACLE_COUNT		TARGET_COUNT
#define NUM_OBS_CORNERS		5
#define BONUS_COUNT			0//TARGET_COUNT
#define KEYFRAME_COUNT		3*TARGET_COUNT

//#define BTW_TARGETS_INTERP_COUNT 3
//static int btwTargetsInterpIndex = 1;

#define BONUS_PT			5

// AYSE: time to stay on target to move on to the other
#define TIME_REQ_ON_TARGET 100
#define TIME_SCORE_INFO_VALID 2

// SOUND GLOBALS
//#define FX_CTRL_REQ_USER_X ""sound/You can take on"
//#define FX_CTRL_REQ_COMP_X ""//"sound/Can I take on.wav"
//#define FX_CTRL_REQ_USER_Z ""//"sound/You can take on"
//#define FX_CTRL_REQ_COMP_Z ""//"sound/Can I take on.wav"
//#define FX_CTRL_REQ_USER ""//sound/You can take on"
//#define FX_CTRL_REQ_COMP ""//sound/You can take on"

//#define FX_CTRL_USER_X "sound/pegconn.wav"
//#define FX_CTRL_COMP_X "sound/pegdisc.wav"
//#define FX_CTRL_USER_Z "sound/pegconn.wav"
//#define FX_CTRL_COMP_Z "sound/pegdisc.wav"
//#define FX_CTRL_USER "sound/pegconn.wav"
//#define FX_CTRL_COMP "sound/pegdisc.wav" /*I control the x axis now.wav*/

//#define DO_INITIAL_FREE_TRIAL
//#define USE_COLOR_CODE			// different color for different conditions

//#define PLAYONCE

#ifdef DO_INITIAL_FREE_TRIAL
	#ifdef PLAYONCE
		#define NUM_TRIALS_PER_COND 2
	#else
		#define NUM_TRIALS_PER_COND 4
	#endif
#else
	#ifdef PLAYONCE
		#define NUM_TRIALS_PER_COND 1
	#else
		#define NUM_TRIALS_PER_COND 3
	#endif
#endif

// AYSE: global color definitions
#define COLOR_BG_UC				SbColor(0,0,0.4)
#define COLOR_BG_SC				SbColor(0.2,0,0.2)
#define COLOR_BG_RE				SbColor(0.4,0,0)
#define COLOR_BG_RE_VHC			SbColor(0,0,0)
#define COLOR_BG_BLACK			SbColor(0,0,0)

#define COLOR_GREEN				Vector(0.0, 1.0, 0.0)
#define COLOR_BLUE				Vector(0.0, 0.0, 1.0)
#define COLOR_GREY				Vector(0.8, 0.8, 0.8)

#define COLOR_BOARD				Vector(0.8f,0.9f,0.6f)
#define COLOR_BOARD_BOUNDARY	Vector(0.7f,0.8f,0.4f)

#define COLOR_TARGET_IND_ON		Vector(0.0, 0.5, 0.0)
#define COLOR_TARGET_IND_OFF	COLOR_BOARD
#define COLOR_TARGET_ACQUIRED	Vector(0.9f,0.5f,0.5f)//Vector(0.5f,0.9f,0.5f)
#define COLOR_TARGET_ON			Vector(0.2f,0.9f,0.2f)//Vector(0.9f,0.5f,0.5f)
#define COLOR_TARGET_OFF		COLOR_GREY


#define COLOR_CIP				COLOR_GREEN//Vector(0.0f, 0.95f, 0.3f)
#define COLOR_HIP				COLOR_BLUE //Vector(0.0f, 0.35f, 0.8f)
#define COLOR_NIP				Vector(0.1f, 0.1f, 0.1f)

#define COLOR_CROLE				COLOR_GREEN
#define COLOR_HROLE				COLOR_GREEN

#define TUNNEL_IMAGE	 "img/tunnel.png"//robot.jpg";
#define COMP_ICON_IMAGE	 "img/robot.jpg"//robot.jpg";
#define USER_ICON_IMAGE	 "img/user.jpg"

// AYSE: Add levels to game, these fill affect tunnel widths
const int LEVEL_EASY	= 1;
const int LEVEL_NORMAL	= 2;
const int LEVEL_HARD	= 3;
const int LEVEL_HARDEST	= 4;
const int LEVEL_DEFAULT	= LEVEL_HARDEST;

const int	SCHEDULER_RATE		= 1000;

const float BLEND_TIME			= (SCHEDULER_RATE == 500) ? 250.0f : 300;
const int	BLEND_TICK_COUNT	= 2 * 5; // # of markers used for role exchange visualization 

const float HIP_RADIUS			= 0.2f;
const float BALL_RADIUS			= 1.5f;	// AYSE: enlarge ball. original: 0.8f;
const float TARGET_RADIUS		= 0.5f;
const float TARGET_HEIGHT		= 3.0f;
const float OBSTACLE_WIDTH		= 25.0f;
const float OBSTACLE_HEIGHT		= 2.0f;
const float OBSTACLE_DEPTH		= 7;
const float TUNNEL_HEIGHT		= OBSTACLE_HEIGHT;

const float BONUS_RADIUS		= 1.0f;
const float BOARD_WIDTH			= 80.0f;	
const float BAR_HEIGHT			= 0.6 * BOARD_WIDTH / (BLEND_TICK_COUNT+1);
const float ICON_HEIGHT			= 3 * BAR_HEIGHT;
const float ICON_HEIGHT_MIN		= BAR_HEIGHT;
const float ICON_HEIGHT_MAX		= 5 * BAR_HEIGHT;
const float BAR_RADIUS			= ICON_HEIGHT / 8;
// zero position 
const float POSZERO				= 0;


const float CLOSE_ENOUGH		= 5;
#endif