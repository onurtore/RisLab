/////////////////////////////////////////////////////////////////
//
// A sample program for haptic board game phantom
// by S. Ozgur Oguz and Ayse Kucukyilmaz 
// Aug 10, 2009
//
/////////////////////////////////////////////////////////////////

#include "Public.h"
#include "HapticCallBack.h"
#include "MathCB.h"
#include "HapticInterfacePoint.h"
#include <GL/gl.h>
#include <windows.h>                              // Header File For Windows
#include "DataRecord.h"



#include "GraphicsController.h"
//#include "GameController.h"

// FUNCTION PROTOTYPES
void myKeyPressCB(void *, SoEventCallback *);
void MyHandleClose(void *,class SoWinComponent *);
void graphicsTimerCallback(void *data, SoSensor *);
void InitialSetup();
void displayUsage();
void GenerateDefaultGraphics(SoSeparator *);
void outputToScreen();
void updatePath();
void initScreenText(SoSeparator* root);
void initRoleVisualization(SoSeparator* root);
SoSeparator* drawRoleExBars();
SoSeparator* drawRoleIcons();
void visualizeRoleExchange();
void setScoreText(char* score);
void setCounterText(char* counter);
void setWarningText(char* warning);

// GLOBALS
SoTransform *stylus_transform;
Vector stylusOffset;
SbMatrix stylusTransMatrix1;
//SbMat stylusTransMatrix1;
SoTransform *stylus_transform2;
Vector stylusOffset2;
SbMatrix stylusTransMatrix2;
//SbMat stylusTransMatrix2;
float stylusR;
float stylusR2;

float ballRadius;

SoTransform *boardRotateX, *boardRotateZ;
SbMatrix boardRotateMatrixX, boardRotateMatrixZ;

//SoTransform *CHIPTransX, *CHIPTransZ, *NHIPTransX, *NHIPTransZ, *HIPTransX, *HIPTransZ;
//SbMatrix *CHIPTrMatX, *CHIPTrMatZ, *NHIPTrMatX, *NHIPTrMatZ, *HIPTrMatX, *HIPTrMatZ;

float targetRadius;
Point targetCenters				[TARGET_COUNT];
Point obstacleCenters			[OBSTACLE_COUNT];

Point keyframeGoalCenters		[KEYFRAME_COUNT];
Point masterKeyframeGoalCenters	[KEYFRAME_COUNT];

float obstacleOrientations		[OBSTACLE_COUNT]; // angles with which obstacles are oriented on the board in radians
float tunnelOrientations		[TARGET_COUNT]; // angles with which obstacles are oriented on the board in radians
//Point bonusCenters			[BONUS_COUNT];

/* AYSE: used for holding the score and counter text variables
	     if these are set directly on screen the graphics thread is disturbed */
string scoreString;
string counterString;
string warningString;

static int  leavePitCounter				= 0;

//EXTERNED VARIABLES
extern int contactKeyframe [KEYFRAME_COUNT];
extern int goalId;
extern Vector ctrlForce;
extern float angleX, angleZ;				// rotation angle of the board
extern float accX, accZ;
extern float positionX, positionZ;
extern float velocityX, velocityZ;
extern float o_massBall;		// mass of the ball - gr
extern float kpBN, kdBN, kpCT, kdCT;
//extern Ball *ball;
extern HapticInterfacePoint *CHIP, *NHIP, *HIP;
//AYSE: externed from Haptic.cpp for role exchange visualization

extern int ctrlMode;
extern float blendTime;
extern float blendAlpha, blendAlpha;
extern bool userWantsCtrl, userGivesCtrl;

extern SbVec3f* computedForces;

extern int keyframeGoalOrders [NUM_TRIALS_PER_COND][TARGET_COUNT];

// AYSE: VISUALIZATION VARIABLES
// path visualization
SoCoordinate3	*pathBCoor, *chipPathCoor, *nhipPathCoor;
SoLineSet		*pathBLines, *chipPathLines, *nhipPathLpines;
SoFont			*pathBFont;

//AYSE: for printing the score on screen
SoAsciiText *scoreText;

//AYSE: for printing the remaining time counter on screen
SoAsciiText *counterText;

//AYSE: for printing the warnings on screen to motivate user
SoAsciiText *warningText;

//AYSE: for printing the trial # on screen
SoAsciiText *trialCounterText;

//SoMaterial * trialStatusMat[NUM_TRIALS_PER_COND];

//AYSE: for printing debug text on screen
SoText2 *txtFc,	*txtTh, *txtAcc, *txtVel, *txtPos;
SoText2 *txtMass, *txtKp1, *txtKd1;
SoText2 *txtKp4, *txtKd4;

//AYSE: for visualizing role bars
SoMaterial	*compBarColor[BLEND_TICK_COUNT][2], *userBarColor[BLEND_TICK_COUNT][2];

//AYSE: for visualizing role icons
SoCube		*compShape, *userShape;

//AYSE: for visualizing role lights
SoMaterial	*compLightColor[2], *userLightColor[2];

// vars for path vis.
SbVec3f bPathPts[2], chipPathPts[2], nhipPathPts[2];
int32_t bPathIndices[1], chipPathIndices[1], nhipPathIndices[1];
int gBallPathUpdCnt = 0;
int gBallPathCnt = -1;
SoTransform *(targetTrans[TARGET_COUNT]);

// GLOBALS for synchronization
// variables need to be setup by InitialSetup()
bool myStylusFlag;

HapticCallBack *effect;
vector <HDSchedulerHandle> callbackHandlers;;


extern int initialize_phantom(bool myboolean);
extern void stopHaptics();
extern HDCallbackCode HDCALLBACK MyHapticLoop(void *pUserData);
extern HDCallbackCode HDCALLBACK DutyCycleCallback(void *pUserData);
extern HDCallbackCode HDCALLBACK QueryMotorTemp(void *pUserData);
extern HDCallbackCode HDCALLBACK ComputeForceCallback(void *pUserData);
extern HDCallbackCode HDCALLBACK BeginFrameCallback(void *);
extern HDCallbackCode HDCALLBACK EndFrameCallback(void *);
extern char *RecordCallback(void *pUserData);
extern HDdouble *aMotorTemp = 0;

// file output
DataRecord *dataRec;

GraphicsController *graphCtrller;
//GameController *gameCtrller;

int main(int argc, char **argv)
{

	dataRec = new DataRecord();
		
	if (argc == 5)
	{
		dataRec->userID = atoi(argv[1]);
		dataRec->cond	= atoi(argv[2]);
		dataRec->perm	= atoi(argv[3]);
		dataRec->level 	= atoi(argv[4]);
	}
	


	HWND myWindow = SoWin::init(argv[0]);

	if(myWindow == NULL) 
		return(-1);
	
	//SetWindowLong(myWindow,GWL_STYLE,GetWindowLong(myWindow,GWL_STYLE) & !WS_BORDER & !WS_SIZEBOX & !WS_DLGFRAME);
	//SetWindowPos(myWindow,HWND_TOP,0,0,100,100,SWP_SHOWWINDOW);
	//displayUsage();
	
	// *****************************************************************
	// create the scene graph
	//gameCtrller = new GameController();
	graphCtrller = new GraphicsController();
	//gameCtrller->addGraphicsController(graphCtrller);

	// Create the effect 
	effect = new HapticCallBack;

	SoundPlayerThread::hapticCb = effect;

	GenerateDefaultGraphics(graphCtrller->getRoot());
	// AYSE: set the difficulty level dynamically 
	//graphCtrller->setLevel(LEVEL_DEFAULT);
	graphCtrller->setLevel(dataRec->level);
		
	InitialSetup();

	SoWinExaminerViewer *myViewer = new SoWinExaminerViewer(myWindow);
	myViewer->setAnimationEnabled(false);
	myViewer->setSceneGraph(graphCtrller->getRoot());

	
	// reorient the camera so that we see the whole board from above
	SoCamera *myCamera = myViewer->getCamera();
	myCamera->position.setValue(0, BOARD_WIDTH+20, BOARD_WIDTH+20/* 100, 200*/); 
	myCamera->pointAt(SbVec3f(0, -BOARD_WIDTH-20, -BOARD_WIDTH-20/* -100, -200*/));

	
	myViewer->saveHomePosition();
	myViewer->setTitle("Yeditepe University");
	myViewer->setSize(SbVec2s(1500, 900));
	myViewer->setDecoration(FALSE);
	myViewer->setFullScreen(FALSE);

#ifdef USE_COLOR_CODE
	if (dataRec->cond == UC)
		myViewer->setBackgroundColor(COLOR_BG_UC);
	else if (dataRec->cond == SC)
		myViewer->setBackgroundColor(COLOR_BG_SC);
	else if (dataRec->cond == RE)
		myViewer->setBackgroundColor(COLOR_BG_RE);
	else if (dataRec->cond == REVHC)
		myViewer->setBackgroundColor(COLOR_BG_RE_VHC);
#else
	myViewer->setBackgroundColor(COLOR_BG_BLACK);
#endif

	//myViewer->setFeedbackVisibility(FALSE); 
	
	myViewer->setWindowCloseCallback(MyHandleClose, graphCtrller->getRoot());
	myViewer->show();
	//ShowWindow(hWnd,SW_SHOW);      // Show The Window

	SoWin::show(myWindow);
	SoWin::mainLoop();
}

void GenerateDefaultGraphics(SoSeparator * root)
{
	//displayUsage();

	// initialize the servo loop (key-press event)
	// An event callback node so we can receive key press events
	SoEventCallback *myEventCB = new SoEventCallback;
	myEventCB->addEventCallback(SoKeyboardEvent::getClassTypeId(), myKeyPressCB, root);
	root->addChild(myEventCB);

	
	SoShadowSpotLight *light =new SoShadowSpotLight;

	light->direction.setValue(5,0,0);
	light->on = true;
	
	root->addChild(light);
	

	
	initScreenText(graphCtrller->getRoot());

	initRoleVisualization(graphCtrller->getRoot());

	targetRadius = TARGET_RADIUS;
	
	targetCenters[0] = Point( (BOARD_WIDTH/2 - 10), 0, -(BOARD_WIDTH/2 - 10) );
	targetCenters[1] = Point( (BOARD_WIDTH/2 - 10), 0,  (BOARD_WIDTH/2 - 10) );
	targetCenters[2] = Point(-(BOARD_WIDTH/2 - 10), 0, -(BOARD_WIDTH/2 - 10) );
	targetCenters[3] = Point(-(BOARD_WIDTH/2 - 10), 0,  (BOARD_WIDTH/2 - 10) );
	// AYSE: uncomment to add new targets
	//targetCenters[3] = Point( -3, 0, -1 );
	//targetCenters[4] = Point(  7, 0,  7 );
	//targetCenters[5] = Point( -2, 0, -8 );
	//targetCenters[6] = Point( -1, 0,  5 );
	//targetCenters[7] = Point(  3, 0, -7 );

	// AYSE: remove obstacles
	// AYSE: Locate obstacles on board 
	obstacleCenters[0] = targetCenters[0]/* - Point(5,0,5)*/;
	obstacleCenters[1] = targetCenters[1]/* - Point(-5,0,5)*/;
	obstacleCenters[2] = targetCenters[2] /*- Point(5,0,-5)*/;
	obstacleCenters[3] = targetCenters[3] /*+ Point(5,0,5)*/;

	// angles with which obstacles are oriented on the board
	obstacleOrientations[0] = -135;//-M_PI/8;
	obstacleOrientations[1] =  135;// M_PI/8;
	obstacleOrientations[2] = -45;//-6*M_PI/8;
	obstacleOrientations[3] =  45;// 6*M_PI/8;

	ballRadius = BALL_RADIUS; // AYSE 0.8;

	boardRotateX = new SoTransform;
	boardRotateZ = new SoTransform;

	root->addChild(boardRotateX);
	root->addChild(boardRotateZ);

	graphCtrller->addInterfacePoint(0); // cip
	graphCtrller->addInterfacePoint(1); // hip
	graphCtrller->addInterfacePoint(2); // nip

	// add the ball
	graphCtrller->addBall();
	graphCtrller->addBoard();

	// add cylinders
	for(int i=0; i < TARGET_COUNT; i++)
	{
		graphCtrller->addTarget(targetCenters[i]);
	}

	// AYSE: remove obstacles
	//AYSE: add obstacles as potential field regions
	for(int i=0; i < OBSTACLE_COUNT; i++)
	{
		graphCtrller->addObstacle(obstacleCenters[i], obstacleOrientations[i]);
	}


	float angle, x, z;
	keyframeGoalCenters[0]	= targetCenters[0] + Point(-11,0,11);
	keyframeGoalCenters[1]	= targetCenters[0];
	keyframeGoalCenters[2]	= keyframeGoalCenters[0] + Point(-3,0,3);
	
	keyframeGoalCenters[3]	= targetCenters[1] + Point(-12,0,-12);
	keyframeGoalCenters[4]	= targetCenters[1];
	keyframeGoalCenters[5]	= keyframeGoalCenters[3] + Point(-3,0,-3);
	
	keyframeGoalCenters[6]	= targetCenters[2] + Point(11,0,11);
	keyframeGoalCenters[7]	= targetCenters[2];
	keyframeGoalCenters[8]	= keyframeGoalCenters[6] + Point(3,0,3);
	
	keyframeGoalCenters[9]	= targetCenters[3] + Point(12,0,-12);
	keyframeGoalCenters[10]	= targetCenters[3];
	keyframeGoalCenters[11]	= keyframeGoalCenters[9] + Point(3,0,-3);
	

	for (int i=0; i < KEYFRAME_COUNT; i++)
	{
		masterKeyframeGoalCenters[i] = keyframeGoalCenters[i];
	
	//	SoSeparator* keyframeSep[NUM_OBS_CORNERS];
	//	SoCube* keyframeMarker = new SoCube;
	//	SoTransform* keyframeTrans[NUM_OBS_CORNERS];
	//	
	//	keyframeMarker->height.setValue(2*OBSTACLE_HEIGHT);

	//	keyframeSep[i] = new SoSeparator;
	//	keyframeTrans[i] = new SoTransform;
	//	keyframeTrans[i]->translation.setValue(keyframeGoalCenters[i]);
	//	
	//	keyframeSep[i]->addChild(keyframeTrans[i]);
	//	keyframeSep[i]->addChild(keyframeMarker);
	//	
	//	root->addChild(keyframeSep[i]);
		//cout << keyframeGoalCenters[i][0] << " " << keyframeGoalCenters[i][2] << endl;
	}

	for (int i=0; i < OBSTACLE_COUNT; i++)
	{
		ObstacleGraphics* obsGr = graphCtrller->obstacleGrs->at(i);
		angle = obstacleOrientations[i];
	
		//obstacleCornerPts[i][0] = Point(5*obs->getWidth() / 6.0f, 0, obs->getDepth()/2.0);
		//obstacleCornerPts[i][1] = Point(- obs->getWidth() / 6.0f, 0, obs->getDepth()/2.0);
		//obstacleCornerPts[i][2] = Point(-0.7*obs->getWidth(), 0, 0);
		//obstacleCornerPts[i][3] = Point(- obs->getWidth() / 6.0f, 0, -obs->getDepth()/2.0);
		//obstacleCornerPts[i][4] = Point(5*obs->getWidth() / 6.0f, 0, -obs->getDepth()/2.0);

		float toWall = BOARD_WIDTH / 2.0f - abs(obsGr->getPosition()[0]);
	
		obsGr->obstacleCornerPts[0] = Point(1.6*toWall, 0, obsGr->getDepth() * obsGr->obs->getTunnelWidthFactor());
		obsGr->obstacleCornerPts[1] = Point(-1.5*toWall, 0, obsGr->getDepth() * obsGr->obs->getTunnelWidthFactor());
		obsGr->obstacleCornerPts[2] = Point(-0.9*obsGr->getWidth(), 0, 0);
		obsGr->obstacleCornerPts[3] = Point(-1.5*toWall, 0, -obsGr->getDepth() * obsGr->obs->getTunnelWidthFactor());
		obsGr->obstacleCornerPts[4] = Point(1.6*toWall, 0, -obsGr->getDepth() * obsGr->obs->getTunnelWidthFactor());

		for (int j = 0; j < NUM_OBS_CORNERS; j++)
		{
			x = obsGr->obstacleCornerPts[j][0];
			z = obsGr->obstacleCornerPts[j][2];
			

			obsGr->obstacleCornerPts[j] = obstacleCenters[i] + Point(x * cos(TO_RADIANS(angle)) + z * sin(TO_RADIANS(angle)), 
																 0, 
																-x * sin(TO_RADIANS(angle)) + z * cos(TO_RADIANS(angle)));
		}

		//SoSeparator* cornerSep[NUM_OBS_CORNERS];
		//SoCube* cornerMarker = new SoCube;
		//SoTransform* cornerTrans[NUM_OBS_CORNERS];

		//obsGr->obsCornersDrawStyle->style = SoDrawStyleElement::FILLED;
		//
		//cornerMarker->height.setValue(2*OBSTACLE_HEIGHT);

		//for (int j = 0; j < NUM_OBS_CORNERS; j++)
		//{
		//	cornerSep[i] = new SoSeparator;
		//	cornerTrans[i] = new SoTransform;
		//	cornerTrans[i]->translation.setValue(obsGr->obstacleCornerPts[j]);
		//	
		//	cornerSep[i]->addChild(obsGr->obsCornersDrawStyle);
		//	cornerSep[i]->addChild(obsGr->cornerMat);
		//	cornerSep[i]->addChild(cornerTrans[i]);
		//	cornerSep[i]->addChild(cornerMarker);
		//	
		//	root->addChild(cornerSep[i]);
		//}
	}					

	////////////////////////////////
	// Set up the timer callback to update the stylus
	SoTimerSensor *graphicUpdate = new SoTimerSensor(graphicsTimerCallback, root);
	graphicUpdate->setInterval(1.0/40.0);
	graphicUpdate->schedule();
}

// handle key-press event here
void myKeyPressCB(void *userData, SoEventCallback *eventCB)
{
	HDErrorInfo error;
	const SoEvent *event = eventCB->getEvent();

	if (SO_KEY_PRESS_EVENT(event, UP_ARROW)) 
	{
		// initialize phantom servo loop 
		if(effect->keepHapticLoop == false) // changed to global var - ozgur
		{
			effect->keepHapticLoop = true;

			effect->initialize_phantom(effect->keepHapticLoop);
			printf("Game started!\n");
			
			// begin frame at the beginning of each servo loop
			callbackHandlers.push_back( hdScheduleAsynchronous(BeginFrameCallback, (void*)0, HD_MAX_SCHEDULER_PRIORITY) );
			// end frame at the end of each servo loop
			callbackHandlers.push_back( hdScheduleAsynchronous(EndFrameCallback, (void*)0, HD_MIN_SCHEDULER_PRIORITY) );

			//callbackHandlers.push_back( hdScheduleAsynchronous(ComputeForceCallback, computedForces, HD_DEFAULT_SCHEDULER_PRIORITY) );
			callbackHandlers.push_back( hdScheduleAsynchronous(MyHapticLoop, (void*)0, HD_DEFAULT_SCHEDULER_PRIORITY) );
			//callbackHandlers.push_back( hdScheduleAsynchronous(DutyCycleCallback, (void*)0, HD_MIN_SCHEDULER_PRIORITY) );

			// record at the end of each frame
			//callbackHandlers.push_back( hdScheduleAsynchronous(RecordCallback, (void*)0, HD_MIN_SCHEDULER_PRIORITY) );

			hdSetSchedulerRate(SCHEDULER_RATE);
			hdStartScheduler();

			SoundPlayerThread::run();
					
			if (HD_DEVICE_ERROR(error = hdGetError()))
			{
				//hduPrintError(stderr, &error, "Failed to start the scheduler");
				cout << "Failed to start the scheduler" << endl;
				return;
			}
	
			//FILE *pFile = fopen("recordServoLoopData.txt","w");
			//hdStartRecord(pFile,RecordCallback,NULL,5000);
		}
	}
	else if (SO_KEY_PRESS_EVENT(event, DOWN_ARROW)) 
	{
		// stop phantom servo loop 
		if( effect->keepHapticLoop == true )	// changed to global var - ozgur
		{
			effect->keepHapticLoop = false;
			printf("haptic is stopped\n");
			
			SoundPlayerThread::aTimer->Stop();
			SoundPlayerThread::aTimer->Enabled = false;

			//Sleep(100);
			effect->stopHaptics();
			exit(0);
		}
	}

	else if (SO_KEY_PRESS_EVENT(event, RIGHT_ARROW)) // AYSE: increase level 
	{
		int level = graphCtrller->getLevel();

		if (level < LEVEL_HARDEST)
			graphCtrller->setLevel(level+1);
	}
	else if (SO_KEY_PRESS_EVENT(event, LEFT_ARROW)) // AYSE: decrease level 
	{
		
		int level = graphCtrller->getLevel();

		if (level > LEVEL_EASY)
			graphCtrller->setLevel(level-1);
	}
	// toggle on/off visual cues
	else if (SO_KEY_PRESS_EVENT(event, F2))
	{
		effect->soundOn = !(effect->soundOn);
		//cout << "soundOn " << effect->soundOn << endl;
	}
	else if (SO_KEY_PRESS_EVENT(event, F3))
	{
		effect->buzzOn = !(effect->buzzOn);
		//cout << "buzzOn " << effect->buzzOn << endl;
	}
	else if (SO_KEY_PRESS_EVENT(event, F4))
	{
		effect->tremorOn = !(effect->tremorOn);
		//cout << "tremorOn " << effect->tremorOn << endl;
	}
	else if (SO_KEY_PRESS_EVENT(event, F5))
	{
		effect->tiltOn = !(effect->tiltOn);
		//cout << "tiltOn " << effect->tiltOn << endl;
	}
	else if (SO_KEY_PRESS_EVENT(event, F6))
	{
		effect->hipsOn = !(effect->hipsOn);
		//cout << "hipsOn " << effect->hipsOn << endl;
	}
	else if (SO_KEY_PRESS_EVENT(event, F8))
	{
		effect->scoreTextOn = !(effect->scoreTextOn);
		//cout << "scoreTextOn " << scoreTextOn << endl;
	}
	else if (SO_KEY_PRESS_EVENT(event, F9))
	{
		effect->roleIconZoomOn = !(effect->roleIconZoomOn);
		//cout << "roleIconsOn " << roleBarsOn << endl;
	}
	else if (SO_KEY_PRESS_EVENT(event, F11))
	{
		if(myStylusFlag == false){
			pathBCoor->point.deleteValues(0,gBallPathCnt-1);
			pathBLines->numVertices.deleteValues(0,gBallPathCnt-1);
			chipPathCoor->point.deleteValues(0,gBallPathCnt-1);
			chipPathLines->numVertices.deleteValues(0,gBallPathCnt-1);
			gBallPathUpdCnt = 0; 
			gBallPathCnt = -1; 
		}
	}
	else if (SO_KEY_PRESS_EVENT(event, Q))
	{
		effect->userFlaggedMoment = true;
	}
	eventCB->setHandled();
}

// handle the window close message
void MyHandleClose(void *,class SoWinComponent *)
{
	//turn off phantom if it is on
	if(effect->keepHapticLoop ==true)
	{
		effect->keepHapticLoop = false;
		printf("handle closed\n");
		
		SoundPlayerThread::aTimer->Stop();
		SoundPlayerThread::aTimer->Enabled = false;

		//Sleep(100);
		effect->stopHaptics();
		exit(0);
	}
	//exit(0);
};

void updatePath()
{
	Ball *ball;
	ball = graphCtrller->ballGr->getBall();
	bPathPts[gBallPathUpdCnt] = SbVec3f(ball->posX, 0, ball->posZ);
	chipPathPts[gBallPathUpdCnt] = graphCtrller->iPs[0].getPosition();
	gBallPathUpdCnt++;
	if ( (gBallPathUpdCnt % 2) == 0 )
	{
		// Set the coordinates to draw lines between
		pathBCoor->point.setValues(gBallPathCnt, 2, bPathPts);
		chipPathCoor->point.setValues(gBallPathCnt, 2, chipPathPts);
		bPathIndices[0] = 2;
		//pathBLines->startIndex.setValue(0);
		pathBLines->numVertices.setValues(gBallPathCnt, 1, bPathIndices);
		chipPathLines->numVertices.setValues(gBallPathCnt, 1, bPathIndices);
	}

	if ((gBallPathUpdCnt % 2) == 0) gBallPathUpdCnt = 0;
	gBallPathCnt++;
}

void graphicsTimerCallback(void *data, SoSensor *)
{
	// WARNING : every data should be synchronized !!
	// get the stylus from haptic loop
	if(myStylusFlag == true)
	{
		int keyframesPerObs = KEYFRAME_COUNT / OBSTACLE_COUNT;
		/* AYSE: add visual feedback to illustrate who has the control */
		visualizeRoleExchange();

		float bpX, bpY, bpZ, cpX, cpY, cpZ, npX, npY, npZ, hpX, hpY, hpZ;
		graphCtrller->ballGr->getPosition().getValue(bpX, bpY, bpZ);
		graphCtrller->iPs[0].getPosition().getValue(cpX, cpY, cpZ);
		graphCtrller->iPs[1].getPosition().getValue(hpX, hpY, hpZ);
		graphCtrller->iPs[2].getPosition().getValue(npX, npY, npZ);
		
		if(effect->hipsOn)
		{
			//updatePath(); // HACK: AYSE: this is problematic 
			for (int i = 0; i < 3; i++)
			{
				graphCtrller->iPs[i].setRadius(HIP_RADIUS);
			}
		}
		else 
		{
			// don't draw the sphere for hip
			for (int i = 0; i < 3; i++)
			{
				graphCtrller->iPs[i].setRadius(0);
			}
		}

		graphCtrller->ballGr->setTranslate(Vector(bpX, bpY, bpZ));

		graphCtrller->iPs[0].transfMat->translation.setValue(cpX, cpY, cpZ);
		graphCtrller->iPs[1].transfMat->translation.setValue(hpX, hpY, hpZ);
		graphCtrller->iPs[2].transfMat->translation.setValue(npX, npY, npZ);
		// rotate separator placed on scene graph before board
		boardRotateX->setMatrix(boardRotateMatrixX);
		boardRotateZ->setMatrix(boardRotateMatrixZ);
			

		TargetGraphics *targetGr;
		
		// Color the target
		targetGr = (TargetGraphics *)graphCtrller->targetGrs->at(goalId/keyframesPerObs);
		targetGr->mat->diffuseColor.setValue(COLOR_TARGET_ON);
		targetGr->targetIndicatorMat->diffuseColor.setValue(COLOR_TARGET_IND_ON);			

		// mark hit cylinders
		for (int i = 0; i < goalId/keyframesPerObs; i++)
		{
			targetGr = (TargetGraphics *)graphCtrller->targetGrs->at(i);			
			targetGr->targetIndicatorMat->diffuseColor.setValue(COLOR_TARGET_IND_OFF);
			
			// treat last keyframe specially since it ends the game
			if (goalId == KEYFRAME_COUNT - 1)
				targetGr->mat->diffuseColor.setValue(COLOR_TARGET_OFF);
			else
				targetGr->mat->diffuseColor.setValue(COLOR_TARGET_ACQUIRED);
		}

		// mark unhit cylinders
		for(int i = goalId/keyframesPerObs+1; i < TARGET_COUNT; i++)
		{			
			targetGr = (TargetGraphics *)graphCtrller->targetGrs->at(i);
			targetGr->mat->diffuseColor.setValue(COLOR_TARGET_OFF);
			targetGr->targetIndicatorMat->diffuseColor.setValue(COLOR_TARGET_IND_OFF);
		}

		if (goalId % keyframesPerObs == keyframesPerObs-1)// exiting
		{
			// treat last keyframe specially since it ends the game

			// Color the next target on exit
			// unmark current target
			int nextTargetId;// = (goalId/keyframesPerObs < TARGET_COUNT-1) ? goalId/keyframesPerObs+1 : 0;
			
			if (goalId == KEYFRAME_COUNT - 1)
			{
				nextTargetId = keyframeGoalOrders[effect->curTrial][0];

				targetGr = (TargetGraphics *)graphCtrller->targetGrs->at(goalId/keyframesPerObs);
				targetGr->mat->diffuseColor.setValue(COLOR_TARGET_OFF);
				targetGr->targetIndicatorMat->diffuseColor.setValue(COLOR_TARGET_IND_OFF);

				targetGr = (TargetGraphics *)graphCtrller->masterTargetGrs->at(nextTargetId);
				targetGr->mat->diffuseColor.setValue(COLOR_TARGET_ON);
				targetGr->targetIndicatorMat->diffuseColor.setValue(COLOR_TARGET_IND_ON);
			}
			else
			{
				nextTargetId = (goalId/keyframesPerObs < TARGET_COUNT-1) ? goalId/keyframesPerObs+1 : 0;

				targetGr = (TargetGraphics *)graphCtrller->targetGrs->at(goalId/keyframesPerObs);
				targetGr->mat->diffuseColor.setValue(COLOR_TARGET_ACQUIRED);
				targetGr->targetIndicatorMat->diffuseColor.setValue(COLOR_TARGET_IND_OFF);
					
				targetGr = (TargetGraphics *)graphCtrller->targetGrs->at(nextTargetId);
				targetGr->mat->diffuseColor.setValue(COLOR_TARGET_ON);
				targetGr->targetIndicatorMat->diffuseColor.setValue(COLOR_TARGET_IND_ON);
			}
		}


	
		/* color the tunnels so that the users can visually 
		observe that they fell into the pit */
		ObstacleGraphics * obsGr;
		
		for(int i = 0; i < OBSTACLE_COUNT; i++)
		{
			obsGr = graphCtrller->obstacleGrs->at(i);
			int whichBound = -1;
			for(int j = 0; j < NUM_OBS_CORNERS-1; j++)
			{
				if(graphCtrller->hitBounds[i][j])
				{
					whichBound = (j == 0) ? 0 : 1;
				}
			}

			if (whichBound >= 0)
			{
				obsGr->changeColor(OBSTACLE_CONTACT_COLOR,whichBound);
				setWarningText("Fault! Try again!");
				effect->scorePrintedOnScreen = true;
				
				for (int k = 0; k < KEYFRAME_COUNT; k++)
				{
					contactKeyframe[k] = 0;
				}
		
				if (leavePitCounter == 0)
				{
					effect->pitfallCount++;
									
					leavePitCounter++;
				}
			}
			else
			{
				obsGr->changeColor(OBSTACLE_COLOR,0);
				obsGr->changeColor(OBSTACLE_COLOR,1);
			}


			// arrange tunnels according to the level
			float toWall = BOARD_WIDTH / 2.0f - abs(obsGr->getPosition()[0]);

			obsGr->trans1->translation.setValue(obsGr->getWidth()/2.0f - toWall, 
										 0, 
										 -obsGr->getDepth() * obsGr->obs->getTunnelWidthFactor());
			obsGr->trans2->translation.setValue(obsGr->getWidth()/2.0f - toWall, 
										 0, 
										 obsGr->getDepth() * obsGr->obs->getTunnelWidthFactor());

			obsGr->obstacleCornerPts[0] = Point(1.6*toWall, 0, obsGr->getDepth() * obsGr->obs->getTunnelWidthFactor());
			obsGr->obstacleCornerPts[1] = Point(-1.5*toWall, 0, obsGr->getDepth() * obsGr->obs->getTunnelWidthFactor());
			obsGr->obstacleCornerPts[2] = Point(-0.9 * obsGr->getWidth(), 0, 0);
			obsGr->obstacleCornerPts[3] = Point(-1.5*toWall, 0, -obsGr->getDepth() * obsGr->obs->getTunnelWidthFactor());
			obsGr->obstacleCornerPts[4] = Point(1.6*toWall, 0, -obsGr->getDepth() * obsGr->obs->getTunnelWidthFactor());

			float x, z;
			for (int j = 0; j < NUM_OBS_CORNERS; j++)
			{
				x = obsGr->obstacleCornerPts[j][0];
				z = obsGr->obstacleCornerPts[j][2];

				obsGr->obstacleCornerPts[j] = obsGr->obs->getPosition() + 
												Point(x * cos(TO_RADIANS(obsGr->obs->getAngle())) + z * sin(TO_RADIANS(obsGr->obs->getAngle())), 
												0, 
												-x * sin(TO_RADIANS(obsGr->obs->getAngle())) + z * cos(TO_RADIANS(obsGr->obs->getAngle())));
			}
		}
		// AYSE: display score and counter texts
		counterText->string.setValue(counterString.c_str());
		scoreText->string.setValue(scoreString.c_str());
		warningText->string.setValue(warningString.c_str());

		if (!(effect->fellInPit))
		{
			leavePitCounter = 0;
		}
		
		char trialString[20]; 
			
		if (effect->isGameOver() == -1)
		{
			sprintf(trialString, "Repetition # %d", effect->curTrial+1);
//			trialStatusMat[effect->curTrial-1]->diffuseColor.setValue(0.15,1,0.30);
		}
		else
		{
			sprintf(trialString, "Repetition # %d", effect->curTrial);
		}

		trialCounterText->string.setValue(trialString);
		

		//if(effect->isGameOver() == 1)
		//{
		//	effect->keepHapticLoop = false;
		////	//Sleep(100);
		////	/*effect->stopHaptics(); */
		//}
		//outputToScreen();
		myStylusFlag = false;
	}
}

void outputToScreen()
{
#ifdef SCREEN_TEXTS
	char caFc[50], caTh[50], caAcc[50], caVel[50], caPos[50];
	char caMass[50], caKp1[50], caKd1[50], caKp4[50], caKd4[50];

	sprintf_s(caFc,"Fc: %.7f N, \t \t  \t \t %.7f N", ctrlForce[0], ctrlForce[2]);
	txtFc->string.setValue(caFc);

	sprintf_s(caTh,"Ang: %.7f deg, \t \t  \t%.7f deg", angleX*57, angleZ*57);
	txtTh->string.setValue(caTh);

	sprintf_s(caAcc,"Acc: %.7f cm/ms^2, \t %.7f cm/ms^2", ball->accX, ball->accZ);
	txtAcc->string.setValue(caAcc);

	sprintf_s(caVel,"Vel: %.7f cm/ms, \t \t %.7f cm/ms", ball->velX, ball->velZ);
	txtVel->string.setValue(caVel);

	sprintf_s(caPos,"Pos: %.7f cm, \t \t \t %.7f cm", ball->posX, ball->posZ);
	txtPos->string.setValue(caPos);

	sprintf_s(caMass,"Mass: %.2f kg", ball->mass);
	txtMass->string.setValue(caMass);

	sprintf_s(caKp1,"Kp1: %.7f", kpBN);
	txtKp1->string.setValue(caKp1);

	sprintf_s(caKd1,"Kd1: %.7f", kdBN);
	txtKd1->string.setValue(caKd1);
		
	sprintf_s(caKp4,"Kp4: %.7f", kpCT);
	txtKp4->string.setValue(caKp4);

	sprintf_s(caKd4,"Kd4: %.7f", kdCT);
	txtKd4->string.setValue(caKd4);

#endif
}

void InitialSetup()
{
	effect->keepHapticLoop = false;
	// initial transformation set to identity.
	stylusTransMatrix1 = SbMatrix(1,0,0,0,
						   0,1,0,0,
						   0,0,1,0,
						   0,0,0,1);
	myStylusFlag = false;

	/* Seed the random-number generator with current time */
	srand ( (unsigned)time(NULL) );

}

void displayUsage()
{
	cout << "F2 : toggle sound on/off." << endl
 		 << "F3 : toggle buzzing on/off." << endl
		 << "F4 : toggle tremor on/off." << endl
		 << "F5 : toggle board tilt on/off." << endl
		 << "F6 : toggle hips visualization on/off." << endl
		 << "F8 : toggle score text on/off." << endl
		 << "F9 : toggle role bars on/off." << endl
		 << "Q  : flag a moment." << endl
		 << "<- : decrease level." << endl
		 << "-> : increase level." << endl;
}

void setScoreText(int score)
{
	char str[20];
	sprintf(str, "Time Score: %3d", (int)(score/1000));
	scoreString = str; 
}

void setCounterText(char* counter)
{
	counterString = counter;
}

void setWarningText(char* warning)
{
	warningString = warning;
}

void initScreenText(SoSeparator* root)
{
	if(effect->scoreTextOn)
	{
		SoSeparator *scoreTextSep;
		SoTransform *scoreTextTrans;
		SoMaterial  *scoreTextMat;
		SoFont *scoreFont = new SoFont;
		scoreFont->name.setValue("Verdana");
		scoreFont->size.setValue(6.0f);

		scoreTextSep = new SoSeparator;
		scoreTextTrans = new SoTransform;
		scoreTextTrans->translation.setValue(0, 0, BOARD_WIDTH*0.65);
		scoreTextTrans->rotation.setValue(SbVec3f(1,0,0), TO_RADIANS(-45));
		scoreTextMat = new SoMaterial;
		scoreTextMat->ambientColor.setValue(0.15,1,0.30);
		//scoreTextMat->diffuseColor.setValue(0.0,0.0,0.0);
		scoreText = new SoAsciiText;
		scoreText->justification = SoAsciiText::CENTER;
		scoreText->string = "Time Score:   0";

		scoreTextSep->addChild(scoreFont);
		scoreTextSep->addChild(scoreTextTrans);
		scoreTextSep->addChild(scoreTextMat);
		scoreTextSep->addChild(scoreText);

		root->addChild(scoreTextSep);	

		SoSeparator *counterTextSep;
		SoTransform *counterTextTrans;
		SoMaterial  *counterTextMat;
		SoFont *counterFont = new SoFont;
		counterFont->name.setValue("Verdana");
		counterFont->size.setValue(6.0f);

		counterTextSep = new SoSeparator;
		counterTextTrans = new SoTransform;
		counterTextTrans->translation.setValue(0, 3, 0);
		counterTextTrans->rotation.setValue(SbVec3f(1,0,0), TO_RADIANS(-45));
		counterTextMat = new SoMaterial;
		counterTextMat->ambientColor.setValue(0.15,1,0.30);
		counterText = new SoAsciiText;
		counterText->justification = SoAsciiText::CENTER;
		counterText->string = "Press Esc, then Up Arrow to Start";

		counterTextSep->addChild(counterFont);
		counterTextSep->addChild(counterTextTrans);
		counterTextSep->addChild(counterTextMat);
		counterTextSep->addChild(counterText);

		root->addChild(counterTextSep);


		SoSeparator *warningTextSep;
		SoTransform *warningTextTrans;
		SoMaterial  *warningTextMat;
		SoFont *warningFont = new SoFont;
		warningFont->name.setValue("Verdana");
		warningFont->size.setValue(6.0f);

		warningTextSep = new SoSeparator;
		warningTextTrans = new SoTransform;
		warningTextTrans->translation.setValue(0, 3, 12);
		warningTextTrans->rotation.setValue(SbVec3f(1,0,0), TO_RADIANS(-45));
		warningTextMat = new SoMaterial;
		warningTextMat->diffuseColor.setValue(0.9,0.15,0.30);
		warningText = new SoAsciiText;
		warningText->justification = SoAsciiText::CENTER;
		warningText->string = " ";

		warningTextSep->addChild(warningFont);
		warningTextSep->addChild(warningTextTrans);
		warningTextSep->addChild(warningTextMat);
		warningTextSep->addChild(warningText);

		root->addChild(warningTextSep);


		//SoSeparator *trialStatusSep;
		//SoTransform *trialStatusTrans;
		//SoCylinder  *trialStatusBars[5];
		//SoTransform *trialStatusBarsTrans[5];
		//trialStatusSep = new SoSeparator;
		//
		//trialStatusTrans = new SoTransform;
		//trialStatusTrans->translation.setValue(-BOARD_WIDTH/2 - BOARD_WIDTH/(2*NUM_TRIALS_PER_COND), 3, -BOARD_WIDTH - 30);
		//trialStatusTrans->rotation.setValue(SbVec3f(0,0,1), TO_RADIANS(90));
		//trialStatusSep->addChild(trialStatusTrans);

		//for (int i = 0; i < NUM_TRIALS_PER_COND; i++)
		//{
		//	trialStatusBars[i]	= new SoCylinder;
		//	trialStatusBars[i]->height = BOARD_WIDTH/NUM_TRIALS_PER_COND;
		//	trialStatusBarsTrans[i] = new SoTransform;
		//	trialStatusBarsTrans[i]->translation.setValue(0,-BOARD_WIDTH/NUM_TRIALS_PER_COND,0);
		//	trialStatusMat[i]	= new SoMaterial;
		//	trialStatusMat[i]->diffuseColor.setValue(0.30,0.15,0.15);
 
		//	trialStatusSep->addChild(trialStatusBarsTrans[i]);
		//	trialStatusSep->addChild(trialStatusMat[i]);
		//	trialStatusSep->addChild(trialStatusBars[i]);
		//}

		//root->addChild(trialStatusSep);

		SoSeparator *trialCounterTextSep;
		SoTransform *trialCounterTextTrans;
		SoMaterial  *trialCounterTextMat;
		SoFont *trialCounterFont = new SoFont;
		trialCounterFont->name.setValue("Verdana");
		trialCounterFont->size.setValue(8.0f);

		trialCounterTextSep = new SoSeparator;
		trialCounterTextTrans = new SoTransform;
		trialCounterTextTrans->translation.setValue(BOARD_WIDTH - 10, 3, -BOARD_WIDTH - 30);
		trialCounterTextTrans->rotation.setValue(SbVec3f(1,0,0), TO_RADIANS(-45));
		trialCounterTextMat = new SoMaterial;
		trialCounterTextMat->ambientColor.setValue(0.15,1,0.30);
		trialCounterText = new SoAsciiText;
		trialCounterText->justification = SoAsciiText::RIGHT;
		trialCounterText->string = "Repetition # 0";

		trialCounterTextSep->addChild(trialCounterFont);
		trialCounterTextSep->addChild(trialCounterTextTrans);
		trialCounterTextSep->addChild(trialCounterTextMat);
		trialCounterTextSep->addChild(trialCounterText);

		root->addChild(trialCounterTextSep);

		SoFont *conditionTextFont = new SoFont;
		SoSeparator *conditionTextSep = new SoSeparator;
		SoTransform *conditionText1Trans = new SoTransform;
		SoMaterial  *conditionText1Mat = new SoMaterial;
		SoTransform *conditionText2Trans = new SoTransform;
		SoMaterial  *conditionText2Mat = new SoMaterial;
		SoTransform *conditionText3Trans = new SoTransform;
		SoMaterial  *conditionText3Mat = new SoMaterial;
		SoTransform *conditionText4Trans = new SoTransform;
		SoMaterial  *conditionText4Mat = new SoMaterial;
		SoAsciiText *conditionText1 = new SoAsciiText;
		SoAsciiText *conditionText2 = new SoAsciiText;
		SoAsciiText *conditionText3 = new SoAsciiText;
		SoAsciiText *conditionText4 = new SoAsciiText;
		conditionTextFont->name.setValue("Verdana");
		conditionTextFont->size.setValue(6.0f);

		conditionText1Trans->translation.setValue(-BOARD_WIDTH - 10, 3, -BOARD_WIDTH - 30);
		conditionText1Trans->rotation.setValue(SbVec3f(1,0,0), TO_RADIANS(-45));
		conditionText2Trans->translation.setValue(0, -8, 0);
		conditionText3Trans->translation.setValue(0, -8, 0);
		conditionText4Trans->translation.setValue(0, -8, 0);
		conditionText1Mat->ambientColor.setValue(0.015,0.1,0.030);
		conditionText2Mat->ambientColor.setValue(0.015,0.1,0.030);
		conditionText3Mat->ambientColor.setValue(0.015,0.1,0.030);
		conditionText4Mat->ambientColor.setValue(0.015,0.1,0.030);
		
		if (dataRec->cond == UC || dataRec->cond == REVHC)
		{
			//conditionText1Mat->diffuseColor.setValue(0.15,1,0.30);
			// AYSE: edited for difficulty adjustment experiment
			if(dataRec->level == LEVEL_EASY)
				conditionText1Mat->diffuseColor.setValue(0.15,1,0.30);
				
			if(dataRec->perm == 1)
			{
				if(dataRec->level == LEVEL_NORMAL)
					conditionText2Mat->diffuseColor.setValue(0.15,1,0.30);
				else if(dataRec->level == LEVEL_HARD)
					conditionText3Mat->diffuseColor.setValue(0.15,1,0.30);
				else if(dataRec->level == LEVEL_HARDEST)
					conditionText4Mat->diffuseColor.setValue(0.15,1,0.30);
			}
			else if(dataRec->perm == 2)
			{
				if(dataRec->level == LEVEL_NORMAL)
					conditionText4Mat->diffuseColor.setValue(0.15,1,0.30);
				else if(dataRec->level == LEVEL_HARD)
					conditionText2Mat->diffuseColor.setValue(0.15,1,0.30);
				else if(dataRec->level == LEVEL_HARDEST)
					conditionText3Mat->diffuseColor.setValue(0.15,1,0.30);
			}
			else if(dataRec->perm == 3)
			{
				if(dataRec->level == LEVEL_NORMAL)
					conditionText3Mat->diffuseColor.setValue(0.15,1,0.30);
				else if(dataRec->level == LEVEL_HARD)
					conditionText4Mat->diffuseColor.setValue(0.15,1,0.30);
				else if(dataRec->level == LEVEL_HARDEST)
					conditionText2Mat->diffuseColor.setValue(0.15,1,0.30);
			}


		}
/*
		else if (dataRec->cond == SC)
		{
			if (dataRec->perm == 1 || dataRec->perm == 2)
				conditionText2Mat->diffuseColor.setValue(0.15,1,0.30);	
			else if (dataRec->perm == 4 || dataRec->perm == 5)
				conditionText3Mat->diffuseColor.setValue(0.15,1,0.30);	
			else if (dataRec->perm == 3 || dataRec->perm == 6)
				conditionText4Mat->diffuseColor.setValue(0.15,1,0.30);
		}
		else if (dataRec->cond == RE)
		{
			if (dataRec->perm == 1 || dataRec->perm == 6)
				conditionText3Mat->diffuseColor.setValue(0.15,1,0.30);	
			else if (dataRec->perm == 2 || dataRec->perm == 5)
				conditionText4Mat->diffuseColor.setValue(0.15,1,0.30);	
			else if (dataRec->perm == 3 || dataRec->perm == 4)
				conditionText2Mat->diffuseColor.setValue(0.15,1,0.30);
		}
		else if (dataRec->cond == REVHC)
		{
			if (dataRec->perm == 1 || dataRec->perm == 4)
				conditionText4Mat->diffuseColor.setValue(0.15,1,0.30);	
			else if (dataRec->perm == 2 || dataRec->perm == 3)
				conditionText3Mat->diffuseColor.setValue(0.15,1,0.30);	
			else if (dataRec->perm == 5 || dataRec->perm == 6)
				conditionText2Mat->diffuseColor.setValue(0.15,1,0.30);
		}
*/
		conditionText1->string = "PRACTICE GAME";
		conditionText2->string = "GAME A";
		conditionText3->string = "GAME B";
		conditionText4->string = "GAME C";

		conditionTextSep->addChild(conditionTextFont);
		conditionTextSep->addChild(conditionText1Trans);
		conditionTextSep->addChild(conditionText1Mat);
		conditionTextSep->addChild(conditionText1);
		conditionTextSep->addChild(conditionText2Trans);
		conditionTextSep->addChild(conditionText2Mat);
		conditionTextSep->addChild(conditionText2);
		conditionTextSep->addChild(conditionText3Trans);
		conditionTextSep->addChild(conditionText3Mat);
		conditionTextSep->addChild(conditionText3);
		conditionTextSep->addChild(conditionText4Trans);
		conditionTextSep->addChild(conditionText4Mat);
		conditionTextSep->addChild(conditionText4);

		root->addChild(conditionTextSep);
	}
}


void initRoleVisualization(SoSeparator* root)
{
	if(effect->roleIconZoomOn)
	{
		root->addChild(drawRoleIcons());
	}
}

SoSeparator* drawRoleExBars()
{
	SoSeparator *barsSep, *compBarSep, *userBarSep;
	// AYSE: uncomment to enable headings on bars
	//	SoText2		*compLabel, *userLabel;
	SoCube		*compShape, *userShape;
	SoTexture2	*compTexture, *userTexture;
	SoCylinder	*compBar[BLEND_TICK_COUNT], *userBar[BLEND_TICK_COUNT];

	Vector barPos = Vector(BOARD_WIDTH/2+10, 0, BOARD_WIDTH/2+10 /*0*/);

	SoMaterial	*compShapeMat, *userShapeMat;	

	SoTransform //*compLabelTrans, *userLabelTrans, 
				*compShapeTrans, *userShapeTrans, 
				*compBarTrans, *userBarTrans,
				*compBarTickTrans[BLEND_TICK_COUNT], *userBarTickTrans[BLEND_TICK_COUNT];

	// AYSE: uncomment to enable headings on bars
	//SoFont *labelFont = new SoFont;
	//labelFont->name.setValue("Verdana");
	//labelFont->size.setValue(18.0f);
	//barsSep->addChild(labelFont);

	barsSep		= new SoSeparator;

	// bar for displaying computer's degree of control.
	// located to the left of the board
	compBarSep		= new SoSeparator;
	compBarTrans	= new SoTransform;

	compBarTrans->translation.setValue(-barPos[0], barPos[1], barPos[2]);
	compBarTrans->rotation.setValue( SbVec3f (1, 0, 0), TO_RADIANS(135) );
	compBarSep->addChild(compBarTrans);

	for(int i = 0; i < BLEND_TICK_COUNT; i++)
	{
		compBar[i]			= new SoCylinder;
		compBarColor[i][0]	= new SoMaterial; // default grey -- control is on user
		compBarTickTrans[i]		= new SoTransform;

		compBar[i]->height.setValue(BAR_HEIGHT);
		compBar[i]->radius.setValue(BAR_RADIUS);
		compBarTickTrans[i]->translation.setValue(0,-BAR_HEIGHT,0);
		compBarSep->addChild(compBarColor[i][0]);
		compBarSep->addChild(compBarTickTrans[i]);
		compBarSep->addChild(compBar[i]);
	}

	compTexture			= new SoTexture2;
	compTexture->filename.setValue(COMP_ICON_IMAGE);
	compShape			= new SoCube;
	compShape->height.setValue(ICON_HEIGHT);
	compShape->width.setValue(ICON_HEIGHT);
	compShape->depth.setValue(0.1);
	compShapeTrans		= new SoTransform;
	compShapeTrans->translation.setValue(0,-2*BAR_HEIGHT,0);
	compShapeMat		= new SoMaterial;
	compBarSep->addChild(compShapeTrans);
	compBarSep->addChild(compShapeMat);
	compBarSep->addChild(compTexture);
	compBarSep->addChild(compShape);


	// AYSE: uncomment to enable headings on bars
	//compLabel			= new SoText2;
	//compLabel->string	= "Comp";
	//compLabelTrans		= new SoTransform;
	//compLabelTrans->translation.setValue(-10,-15,0);		
	//compBarSep->addChild(compLabelTrans);
	//compBarSep->addChild(compLabel);

	// bar for displaying user's degree of control.
	// located to the right of the board
	userBarSep = new SoSeparator;
	userBarTrans	= new SoTransform;
	
	userBarTrans->translation.setValue(barPos[0], barPos[1], barPos[2]);
	userBarTrans->rotation.setValue( SbVec3f (1, 0, 0), TO_RADIANS(135) );
	userBarSep->addChild(userBarTrans);

	for(int i = 0; i < BLEND_TICK_COUNT; i++)
	{
		userBar[i]			= new SoCylinder;
		userBarColor[i][0]	= new SoMaterial; // default green -- control is on user
		userBarTickTrans[i]		= new SoTransform;

		userBar[i]->height.setValue(BAR_HEIGHT);
		userBar[i]->radius.setValue(BAR_RADIUS);
		userBarColor[i][0]->diffuseColor.setValue(COLOR_HROLE);
		userBarTickTrans[i]->translation.setValue(0,2*BAR_HEIGHT,0);

		userBarSep->addChild(userBarColor[i][0]);
		userBarSep->addChild(userBarTickTrans[i]);
		userBarSep->addChild(userBar[i]);
	}

	userTexture			= new SoTexture2;
	userTexture->filename.setValue(USER_ICON_IMAGE);
	userShape			= new SoCube;
	userShape->height.setValue(ICON_HEIGHT);
	userShape->width.setValue(ICON_HEIGHT);
	userShape->depth.setValue(0.1);
	userShapeTrans		= new SoTransform;
	userShapeTrans->translation.setValue(0,-2*BAR_HEIGHT,0);
	userShapeMat		= new SoMaterial;
	userBarSep->addChild(userShapeTrans);
	userBarSep->addChild(userShapeMat);
	userBarSep->addChild(userTexture);
	userBarSep->addChild(userShape);

	//userLabel = new SoText2;
	//userLabel->string = "User";
	//userLabelTrans		= new SoTransform;
	//userLabelTrans->translation.setValue(-5,-15,0);	
	//userBarSep->addChild(userLabelTrans);
	//userBarSep->addChild(userLabel);

	barsSep->addChild(compBarSep);
	barsSep->addChild(userBarSep);

	return barsSep;
}


SoSeparator* drawRoleIcons()
{
	SoSeparator *iconsSep, *compIconSep, *userIconSep;
	// AYSE: uncomment to enable headings on bars
	//	SoText2		*compLabel, *userLabel;
	SoTexture2	*compTexture, *userTexture;
	SoMaterial	*compShapeMat, *userShapeMat;	
	SoTransform *compIconTrans, *userIconTrans;

	Vector iconPos = Vector(BOARD_WIDTH/6.0f, 0, -BOARD_WIDTH+10 /*0*/);

	iconsSep		= new SoSeparator;

	compIconSep		= new SoSeparator;
	compShape		= new SoCube;
	compTexture		= new SoTexture2;
	compShapeMat	= new SoMaterial;
	compIconTrans	= new SoTransform;
	
	userIconSep		= new SoSeparator;
	userShape		= new SoCube;
	userTexture		= new SoTexture2;
	userShapeMat	= new SoMaterial;
	userIconTrans	= new SoTransform;

	// AYSE: uncomment to enable headings on bars
	//SoFont *labelFont = new SoFont;
	//labelFont->name.setValue("Verdana");
	//labelFont->size.setValue(18.0f);
	//barsSep->addChild(labelFont);

	compIconTrans->translation.setValue(-iconPos[0], iconPos[1], iconPos[2]);
	compIconTrans->rotation.setValue( SbVec3f (1, 0, 0), TO_RADIANS(135) );
	compIconSep->addChild(compIconTrans);

	compTexture->filename.setValue(COMP_ICON_IMAGE);
	
	compShape->height.setValue(ICON_HEIGHT_MIN);
	compShape->width.setValue(ICON_HEIGHT_MIN);
	compShape->depth.setValue(0.1);

	compIconSep->addChild(compShapeMat);
	compIconSep->addChild(compTexture);
	compIconSep->addChild(compShape);

	// AYSE: uncomment to enable headings on bars
	//compLabel			= new SoText2;
	//compLabel->string	= "Comp";
	//compLabelTrans		= new SoTransform;
	//compLabelTrans->translation.setValue(-10,-15,0);		
	//compBarSep->addChild(compLabelTrans);
	//compBarSep->addChild(compLabel);

	userIconTrans->translation.setValue(iconPos[0], iconPos[1], iconPos[2]);
	userIconTrans->rotation.setValue( SbVec3f (1, 0, 0), TO_RADIANS(135) );
	userIconSep->addChild(userIconTrans);

	userTexture->filename.setValue(USER_ICON_IMAGE);
	userShape->height.setValue(ICON_HEIGHT_MAX);
	userShape->width.setValue(ICON_HEIGHT_MAX);
	userShape->depth.setValue(0.1);

	userIconSep->addChild(userShapeMat);
	userIconSep->addChild(userTexture);
	userIconSep->addChild(userShape);

	iconsSep->addChild(compIconSep);
	iconsSep->addChild(userIconSep);

	return iconsSep;
}


void visualizeRoleExchange()
{
	if ( dataRec->cond == RE || dataRec->cond == REVHC )
	{
		if (effect->roleIconZoomOn)
		{
			float tickStep = blendTime / BLEND_TICK_COUNT; // zoom icons at each tickStep in blending
			int i = 0;
			float zoomStep = (ICON_HEIGHT_MAX - ICON_HEIGHT_MIN) / tickStep;

			float h,w;
			if (userWantsCtrl) // AYSE: don't do redundant checks here... just for completeness
			{
				i = (int) floor(blendAlpha / tickStep);
			
				h = compShape->height.getValue();
				w = compShape->width.getValue();
				h = h > ICON_HEIGHT_MIN + zoomStep*i ? h - zoomStep*i : ICON_HEIGHT_MIN;
				w = w > ICON_HEIGHT_MIN + zoomStep*i ? w - zoomStep*i : ICON_HEIGHT_MIN;

				compShape->height.setValue(h);
				compShape->width.setValue(w);

				h = userShape->height.getValue();
				w = userShape->width.getValue();
				h = h < ICON_HEIGHT_MAX - zoomStep*i ? h + zoomStep*i : ICON_HEIGHT_MAX;
				w = w < ICON_HEIGHT_MAX - zoomStep*i ? w + zoomStep*i : ICON_HEIGHT_MAX;

				userShape->height.setValue(h);
				userShape->width.setValue(w);
			}
			else if (userGivesCtrl) // AYSE: don't do redundant checks here... just for completeness)
			{
				i = (int) floor(blendAlpha / tickStep);

				h = compShape->height.getValue();
				w = compShape->width.getValue();
				h = h < ICON_HEIGHT_MAX - zoomStep*i ? h + zoomStep*i : ICON_HEIGHT_MAX;
				w = w < ICON_HEIGHT_MAX - zoomStep*i ? w + zoomStep*i : ICON_HEIGHT_MAX;

				compShape->height.setValue(h);
				compShape->width.setValue(w);

				h = userShape->height.getValue();
				w = userShape->width.getValue();
				h = h > ICON_HEIGHT_MIN + zoomStep*i ? h - zoomStep*i : ICON_HEIGHT_MIN;
				w = w > ICON_HEIGHT_MIN + zoomStep*i ? w - zoomStep*i : ICON_HEIGHT_MIN;

				userShape->height.setValue(h);
				userShape->width.setValue(w);
			}
		}
	}
}