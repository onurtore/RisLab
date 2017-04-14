/**************************************************************************
Developed by: Cigil Ece Madan and Ayse Kucukyilmaz
Purpose: The server loop for haptic.
Date: Jul. 22, 2013
**************************************************************************/

// INCLUDE FILES
#include "..\DataTransferSocket\Network\NetworkServices.h"
#include "..\DataTransferSocket\Network\NetworkData.h"

#include "..\DataTransferSocket\Server\ServerApp.h"

#include <float.h>
#include "Public.h"

#include "GraphicsController.h"
#include "HapticCallBack.h"
#include "HapticInterfacePoint.h"
#include "BgTimer.h"
#include <fstream>
#include "SoundPlayerThread.h"


//Onur
#include "baseChange.h" //Base Change for 2D coordinate systems
#include "wheretoGo.h" // Path Finder Algorithm ( WaweFront + Potantiel Field) 
#include <vector>
#include "public2.h"

// initial position 
// TODO: input from user
double initX = -15;
double initY = -40;
double initZ = -40;

// goalposition
// TODO: input from user
double goalX = 0;
double goalY = -15;
double goalZ = 15;


// desired position to reach
double desiredX;
double desiredY;
double desiredZ;


// controller gains
//double kp = 0.7;
//double kd = 1.3;
//double ki = 0.0001;

double error1x;
double error1y;
double error1z;

double error2x;
double error2y;
double error2z;

double errTotalX = 0;
double errTotalY = 0;
double errTotalZ = 0;

//step sizes
double dx;
double dy;
double dz;
double OnurRuns = 0;

hduVector3Dd Onurforce;



//Onur 


#define NO_COLLISION	0
#define COLLISION_DOWN	21
#define COLLISION_UP	22
#define COLLISION_RIGHT	11
#define COLLISION_LEFT	12

//Onur 
int  kalamar = 0;

// EXTERNED VARIABLES
extern HapticCallBack *effect;
extern GraphicsController *graphCtrller;
extern void setScoreText(int score);
extern void setWarningText(char* warning);
extern void setMessageText(char* warning);
extern void setTargetText1(char* target);
extern void setTargetText2(char* target);
extern void setGameOverText(char* target);
extern void setTrialText(int trialNum);

extern float ballRadius;
//Onur
extern vector<double> hapticForce;
extern wheretoGo path;


Vector boundary;
float boundaryThickness;
static HDdouble *gServoMotorTemp = 0;
static HDint gNumMotors;
static HDdouble *aMotorTemp = 0;

SbVec3f *computedForces = new SbVec3f(0, 0, 0);
static HDdouble fFromBall[3] = { 0, 0, 0 }, fSprFromBtoN[3] = { 0, 0, 0 },
fFromNtoC[3] = { 0, 0, 0 }, fFromNtoH[3] = { 0, 0, 0 };
float cpX, cpY, cpZ, cvX, cvY, cvZ, caX, caY, caZ;
int wall_hit = 0;
static volatile HDboolean bRampDownForces = FALSE;
static volatile HDboolean bRampUpForces = FALSE;

extern SbMatrix stylusTransMatrix1;
extern SbMatrix stylusTransMatrix2;
extern float stylusR;
extern float stylusR2;
extern Vector stylusOffset;
extern Vector stylusOffset2;

extern bool myStylusFlag;
extern SbMatrix boardRotateMatrixX;
extern SbMatrix boardRotateMatrixZ;
// path visualization vars from graphics.cpp
extern SoCoordinate3	*pathBCoor;
extern SoLineSet		*pathBLines;
extern SoSeparator		*scoreTimeSep;
extern SoTransform		*mySTStrTrans;



int crunsw = 0; // collision runs (wall)
int crunsb = 0; // collision runs (boundary)

// data saving
Vector targetPosA1;
Vector targetPosA2;
float targetAngleA1;
float targetAngleA2;

int trialNo;
int errorTime = 0;
int totalConflictTime = 0;

// var for path vis
int ballPathPtCnt = 0, ballPathUpdCnt = 0;

//ofstream fp;


// GLOBALS
SbVec3f new_probe_tip;
//SbVec3f new_probe_tip2;

float P[3], P2[3], V[3], V2[3];

//int Interpolation_time = 100;
//float prevForceX = 0.0f; float prevForceY = 0.0f; float prevForceZ = 0.0f; 
//float nextForceX = 0.0f; float nextForceY = 0.0f; float nextForceZ = 0.0f;

//walls in mixed scenario
float wallZ11 = wallTz + wallDepth*0.5;
float wallZ12 = wallTz - wallDepth*0.5;
float wallZ21 = wallTz2 + wallDepth23 * 0.5;
float wallZ22 = wallTz2 - wallDepth23 * 0.5;
float wallZ31 = wallTz3 + wallDepth23 * 0.5;
float wallZ32 = wallTz3 - wallDepth23 * 0.5;

float wallX1 = wallTx;
float wallX2 = wallTx2;
float wallX3 = wallTx3;

//walls in rotational scenario
float wallX1_rot = SHIFT_X;
float wallZ11_rot = SHIFT_Z + (obstacleDepth + middleGap) * 0.5 + obstacleDepth *0.5;
float wallZ12_rot = SHIFT_Z + (obstacleDepth + middleGap) * 0.5 - obstacleDepth *0.5;
float wallX2_rot = SHIFT_X;
float wallZ21_rot = SHIFT_Z - (obstacleDepth + middleGap) * 0.5 + obstacleDepth *0.5;
float wallZ22_rot = SHIFT_Z - (obstacleDepth + middleGap) * 0.5 - obstacleDepth *0.5;

int wall_n = 3;
float xwall[3][3];


int xcoll;
/* ozgur - */
static long int elapsed = 0, elapsedTurn, cntF = 0;//The variables in these two lines
static time_t t;	       		// are used to calculate the server rate.
static long int runs = 0, runsPerTrial = 0;
float maxForceFromBoard = 2.0f; // for each axis

static int  scoreTime = 0;



#ifdef PHANTOM_PREMIUM
float kpHN = 0.25f;
float kpCN = 0.25f;//0.55
float kdN = 0.001f;//0.001
#else
//Force on Ball
float kpHN = 0.6f;
float kpCN = 0.6f;
float kdN = 0.001f;
#endif

//WALL
float dpHN = 0.025f;
float dpCN = 0.025f;
float kpHW = 0.050f;//YOK
float kpCW = 0.050f;//YOK
float K_WALL_X = .04f;
float K_WALL_Z = .04f;
float Kd_WALL = 0.0003F;
float K_WALL_X1 = .01f;
float K_WALL_Z1 = .01f;
float Kd_WALL1 = 0.0003F;

Vector currentPos = Vector(0, 0, 0);
Vector fhw = Vector(0, 0, 0);
Vector fcw = Vector(0, 0, 0);
float boundaryforceHX = 0.0f;
float boundaryforceHZ = 0.0f;
float boundaryforceCX = 0.0f;
float boundaryforceCZ = 0.0f;


float f_forceX = 0;
float f_forceY = 0;
float f_forceZ = 0;

float frictionX[2]; // not used
float frictionZ[2];// not used


//Ball *ball;
extern DataRecord *dataRec;

float	sumFs[3] = { 0.0f, 0.0f, 0.0f }, sqrFs[3] = { 0.0f, 0.0f, 0.0f };

extern vector <HDSchedulerHandle> callbackHandlers;
void setforces(int r11, int r22, int r33, int r44, int forcetypes);
void calcForceFromBoardRot(Point pt, Vector &fBoard);
Vector boundaryColl(float angle, Vector ballPosNext);
Vector boundaryColl_full(Vector ballPosNext);
Vector calculateFriction();


//Onur Bu fonksiyonun içeriðini deðiþtirdim
Vector calcBallPos();

// find the closest wall
Vector collX(Vector ball);

// check for collisions and find corrected position for the ball
Vector xwallColl(float wallx1, float wallx2, float wallz1, float wallz2, Vector ballPosNext, float angle, float wWidth, int wallID);

Vector xwallColl_full(float wallx1, float wallx2, float wallz1, float wallz2, float wallx1f, float wallx2f, float wallz1f, float wallz2f, Vector ballPosNext, float angle, float wWidth);


//create 48 points of the object for collision detection
void createLen(float lenArray[48][3]);

float InvSqrt(float x);
HDdouble instRate;
int forcetypes;

Vector tempF1, tempF2;

Vector initialC(int type);

int giveWarningW;
int giveWarningB;
int giveWarning;
int giveWarningG;
int target1Achieved, target2Achieved;

Vector fOnHip, fOnCip, forceC;
Vector fOnHip_scaled, fOnCip_scaled, fResistance, fFriction;
Vector firstPosHip, firstPosCip;

double kp = 0.4;
double ki = 0.001;
//Onur
int timeToGoInit = 3000;
vector<double> Algorithm_force(3);
//int timeToGoInit = 15000;
int wait = 1000;
bool gameStart;

double initialPos[3];//={6, 15 , -66};
double initialPosC[3];//={-40+SHIFT_X, -93 , 0+SHIFT_Z};//{-78, -93 , -37};//4 -89-94
double initialPosH[3];//={-64+SHIFT_X, -93 , 0+SHIFT_Z};//-4 -97 -100

int initialPosAcquired = 0;

Vector errorC, totErrC;
Vector totErrorHip, totErrorCip;


float TARGET_POS_A1[NUM_TRIALS_PER_COND_M][3]; // target Positions for player 1
float TARGET_POS_A2[NUM_TRIALS_PER_COND_M][3]; // target Positions for player 2
float TARGET_ANGLE_A1[NUM_TRIALS_PER_COND_M] = { 0 }; // target angle for player 1
float TARGET_ANGLE_A2[NUM_TRIALS_PER_COND_M] = { 0 }; // target angle for player 2
float LIMIT_ANGLE[COND_COUNT][2] = { 0.0 }; //upper and lower angle limits for rotational scenarios


float PRACTICE_TARGET_POS_A1_S[PRACTICE_COUNTS][3]; // target Positions for player 1 in practice mode
float PRACTICE_TARGET_POS_A2_S[PRACTICE_COUNTS][3]; // target Positions for player 2 in practice mode
float PRACTICE_TARGET_ANGLE_A1[PRACTICE_COUNTS] = { 0.0 };
float PRACTICE_TARGET_ANGLE_A2[PRACTICE_COUNTS] = { 0.0 };
float PRACTICE_TARGET_POS_A1_M[PRACTICE_COUNTS][3];
float PRACTICE_TARGET_POS_A2_M[PRACTICE_COUNTS][3];

// the sercer will 
//	initiate the maze game 
//	listen incoming connections
//	get position of device 2 from client
//	calculate forces and send forces to the client
ServerApp *server;

void serverLoop(void * arg);

HapticCallBack::HapticCallBack()
{
	curTrial = 0;

	server = new ServerApp();
	// create thread with arbitrary argument for the run function
	_beginthread(serverLoop, 0, 0);

	// both styli are guided whence a guidance method is selected
	guidedStyli[BLUE] = false;
	guidedStyli[GREEN] = false;
	userControlledStyli[BLUE] = true;
	userControlledStyli[GREEN] = true;

	soundOn = true;
	tiltOn = false;
	hipsOn = true;
	vectorOn = false;
	RvectorOn = true;
	scoreTextOn = true;
	warningOn = true;

	hitWalls = false;

	// initially user controls both axes
	separateAxes = false;
	keepHapticLoop = false;
	isGameFinished = true;

	//	ball = new Ball;
	// *** initialize vars from graphics controller class *** //
	boundary[0] = graphCtrller->boardGr->boundary[0];
	boundary[1] = graphCtrller->boardGr->boundary[1];
	boundary[2] = graphCtrller->boardGr->boundary[2];

	boundaryThickness = graphCtrller->boardGr->boundaryThickness;


	aMotorTemp = 0;
}

int HapticCallBack::initialize_phantom(bool myboolean)
{
	// Coordinate system has origin at reset position  //
	// phantom is reset upon entry into program so put //
	// phantom in reset position when starting program //
	// get the phantom up and running //
	HDErrorInfo error;

	hHD1 = hdInitDevice(DEVICE1);

	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		//hduPrintError(stderr, &error, "Failed to initialize haptic device");
		printf("Failed to initialize haptic device");
		myboolean = false;
		return(-1);
	}
	/* Query the number of output DOF (i.e. num motors). */
	hdGetIntegerv(HD_OUTPUT_DOF, &gNumMotors);
	aMotorTemp = (HDdouble *)malloc(sizeof(HDdouble) * gNumMotors);
	gServoMotorTemp = (HDdouble *)malloc(sizeof(HDdouble) * gNumMotors);

	hdEnable(HD_FORCE_OUTPUT);
	hdEnable(HD_FORCE_RAMPING);
	hdEnable(HD_MAX_FORCE_CLAMPING);
	hdEnable(HD_SOFTWARE_VELOCITY_LIMIT);
	hdEnable(HD_SOFTWARE_FORCE_IMPULSE_LIMIT);

	hdGetDoublev(HD_NOMINAL_MAX_CONTINUOUS_FORCE, &FORCE_LIMIT);
	hdGetFloatv(HD_NOMINAL_MAX_STIFFNESS, &KP_LIMIT);
	hdGetFloatv(HD_NOMINAL_MAX_DAMPING, &KD_LIMIT);

	cout << endl << "Phantom id: " << (int)hHD1 << " is initialized." << endl;

	// AYSE SERVER CODE commented
	/*	hHD2 = hdInitDevice(DEVICE2);

	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
	hduPrintError(stderr, &error, "Failed to initialize haptic device");
	myboolean = false;
	return(-1);
	}
	*/
	/* Query the number of output DOF (i.e. num motors). */
	/*	hdGetIntegerv(HD_OUTPUT_DOF, &gNumMotors);
	aMotorTemp = (HDdouble *) malloc(sizeof(HDdouble) * gNumMotors);
	gServoMotorTemp = (HDdouble *) malloc(sizeof(HDdouble) * gNumMotors);

	hdEnable(HD_FORCE_OUTPUT);
	hdEnable(HD_FORCE_RAMPING);
	hdEnable(HD_MAX_FORCE_CLAMPING);
	hdEnable(HD_SOFTWARE_VELOCITY_LIMIT);
	hdEnable(HD_SOFTWARE_FORCE_IMPULSE_LIMIT);

	hdGetDoublev(HD_NOMINAL_MAX_CONTINUOUS_FORCE, &FORCE_LIMIT);
	hdGetFloatv(HD_NOMINAL_MAX_STIFFNESS, &KP_LIMIT);
	hdGetFloatv(HD_NOMINAL_MAX_DAMPING, &KD_LIMIT);

	cout << endl << "Phantom id: " << (int)hHD2 << " is initialized." << endl;
	*/
	return(0);
}

HDCallbackCode HDCALLBACK BeginFrameCallback(void *)
{
	hdBeginFrame(effect->hHD1);
	// AYSE SERVER CODE commented
	//	hdBeginFrame(effect->hHD2);

	return HD_CALLBACK_CONTINUE;
}
HDCallbackCode HDCALLBACK EndFrameCallback(void *)
{
	// AYSE SERVER CODE commented
	hdEndFrame(effect->hHD1);
	//	hdEndFrame(effect->hHD2);
	return HD_CALLBACK_CONTINUE;
}

/*******************************************************************************
Log in a separate thread
*******************************************************************************/
char *RecordCallback(void *pUserData)
{
	char *c = new char[200];
	//sprintf(c,"%f %f %f", getTicks(), runs, effect->curTrial, ball
	//						);
	return c;
}



/*******************************************************************************
A method for checking that the scheduler is not being overloaded:
*******************************************************************************/
HDCallbackCode HDCALLBACK DutyCycleCallback(void *pUserData)
{
	double timeElapsed = hdGetSchedulerTimeStamp();
	if (timeElapsed > 1 / (float)SCHEDULER_RATE)
	{
		assert(false && "Scheduler has exceeded scheduler rate.");
		cout << "Scheduler has exceeded scheduler rate: " << timeElapsed << endl;
	}
	return HD_CALLBACK_CONTINUE;
}

/*******************************************************************************
Callback that queries motor temperature.
*******************************************************************************/
HDCallbackCode HDCALLBACK QueryMotorTemp(void *pUserData)
{
	aMotorTemp = (HDdouble *)pUserData;

	hdGetDoublev(HD_MOTOR_TEMPERATURE, aMotorTemp);

	return HD_CALLBACK_DONE;
}

/*******************************************************************************
Callback for haptic loop.
*******************************************************************************/
HDCallbackCode HDCALLBACK MyHapticLoop(void *pUserData)
{
	fhw = Vector(0.0, 0.0, 0.0);
	fcw = Vector(0.0, 0.0, 0.0);

	boundaryforceHX = 0.0f;
	boundaryforceHZ = 0.0f;
	boundaryforceCX = 0.0f;
	boundaryforceCZ = 0.0f;

	static bool firstTime = true;

	static Vector forceFB1 = Vector(0, 0, 0);
	static Vector forceFB2 = Vector(0, 0, 0);

	Ball *ball = graphCtrller->ballGr->getBall();
	Ball *ball2 = graphCtrller->ballGr2->getBall();

	float cpX, cpY, cpZ, hpX, hpY, hpZ;
	float bpX, bpY, bpZ;
	float cvX, cvY, cvZ, hvX, hvY, hvZ;
	float hhpX, hhpY, hhpZ, hcpX, hcpY, hcpZ;
	float hhvX, hhvY, hhvZ, hcvX, hcvY, hcvZ;
	// velocities are calculated, not read from devices
	//float hipvX, hipvY, hipvZ, cipvX, cipvY, cipvZ;

	float angle_rot = 0.0f;
	float angle_rotVel = 0.0f;
	float angle_rotAcc = 0.0f;

	Vector ballPosNext; // calculated next ball position

	char str[20] = " ";

	static int conflictCounter = 0;

	Point myP1, myP2;
	hduVector3Dd pos;
	//hduVector3Dd vel;
	hdMakeCurrentDevice(effect->hHD1);
	hdGetDoublev(HD_CURRENT_POSITION, pos);
	// velocities are calculated, not read from devices
	//hdGetDoublev(HD_CURRENT_VELOCITY, vel);

	myP1.setValue((float)pos[0], (float)pos[1], (float)(pos[2])/**1.5f*/);

	hpX = myP1[0];
	hpY = myP1[1];
	hpZ = myP1[2];

	// velocities are calculated, not read from devices
	//hipvX = vel[0];
	//hipvY = vel[1];
	//hipvZ = vel[2];

	// AYSE SERVER CODE commented
	//hdMakeCurrentDevice(effect->hHD2);
	//hdGetDoublev(HD_CURRENT_POSITION, pos);
	//hdGetDoublev(HD_CURRENT_VELOCITY, vel);
	pos = server->positionData;
	//vel = server->velocityData;

	myP2.setValue((float)pos[0], (float)pos[1], (float)(pos[2])/**1.5f*/);
	cpX = myP2[0];
	cpY = myP2[1];
	cpZ = myP2[2];

	// velocities are calculated, not read from devices
	//cipvX = vel[0];
	//cipvY = vel[1];
	//cipvZ = vel[2];

	effect->PrintMotorTemp(aMotorTemp, gNumMotors);

	//graphCtrller->hip.setVelocity( Point(hvX,hvY,hvZ));
	//graphCtrller->cip.setVelocity( Point(cvX,cvY,cvZ));



	// TODO: correct these
	// define initial configurations 


	if (dataRec->scenario == STRAIGHT)
	{
		dataRec->NUM_TRIALS_PER_COND = NUM_TRIALS_PER_COND_S;

		if ((dataRec->perm == 0) || (dataRec->perm == 2) || (dataRec->perm == 3) || (dataRec->perm == 7))
		{
			initialPosC[0] = -40 + SHIFT_X;
			initialPosC[1] = -93;
			initialPosC[2] = SHIFT_Z;

			initialPosH[0] = -64 + SHIFT_X;
			initialPosH[1] = -93;
			initialPosH[2] = SHIFT_Z;
		}
		else
		{
			initialPosC[0] = 64 + SHIFT_X;
			initialPosC[1] = -93;
			initialPosC[2] = SHIFT_Z;

			initialPosH[0] = 40 + SHIFT_X;
			initialPosH[1] = -93;
			initialPosH[2] = SHIFT_Z;
		}

	}
	else if (dataRec->scenario == MIXED)
	{
		dataRec->NUM_TRIALS_PER_COND = NUM_TRIALS_PER_COND_M;
		initialPosC[0] = -61 + SHIFT_X;
		initialPosC[1] = -93;
		initialPosC[2] = SHIFT_Z;

		initialPosH[0] = -85 + SHIFT_X;
		initialPosH[1] = -93;
		initialPosH[2] = SHIFT_Z;

	}


	//if (dataRec->scenario == STRAIGHT)
	//{
	//	if (dataRec->cond == FULL_CONFLICT)
	//	{
	//		initialPosC[0] = SHIFT_X + BALL_WIDTH / 2;
	//		initialPosC[1] = -93;
	//		initialPosC[2] = SHIFT_Z;

	//		initialPosH[0] = SHIFT_X - BALL_WIDTH / 2;
	//		initialPosH[1] = -93;
	//		initialPosH[2] = SHIFT_Z;
	//	}
	//	else
	//	{
	//		initialPosC[0] = -40 + SHIFT_X;
	//		initialPosC[1] = -93;
	//		initialPosC[2] = SHIFT_Z;

	//		initialPosH[0] = -64 + SHIFT_X;
	//		initialPosH[1] = -93;
	//		initialPosH[2] = SHIFT_Z;
	//	}
	//}
	//else if (dataRec->scenario == ROTATIONAL )
	//{
	//	initialPosC[0] = SHIFT_X + BALL_WIDTH / 2;
	//	initialPosC[1] = -93;
	//	initialPosC[2] = SHIFT_Z;

	//	initialPosH[0] = SHIFT_X - BALL_WIDTH / 2;
	//	initialPosH[1] = -93;
	//	initialPosH[2] = SHIFT_Z;
	//}
	//else if (dataRec->scenario == MIXED )
	//{
	//	initialPosC[0] = -61 + SHIFT_X;
	//	initialPosC[1] = -93;
	//	initialPosC[2] = SHIFT_Z;

	//	initialPosH[0] = -85 + SHIFT_X;
	//	initialPosH[1] = -93;
	//	initialPosH[2] = SHIFT_Z;

	//}

	if ((firstTime == true) && ((effect->keepHapticLoop) == true))
	{

		//fp.open("force_acc_vel.txt");
		firstTime = false;

		//t = time(NULL);	elapsed = (long)t;
		/*sec_init();*/
		runs = 1;
		runsPerTrial = 1;
		totalConflictTime = 0;
		trialNo = 1;
		errorTime = 0;
		gameStart = false;
		target1Achieved = 0;
		target2Achieved = 0;

		graphCtrller->cip.setVelocity(Vector(0, 0, 0));
		graphCtrller->hip.setVelocity(Vector(0, 0, 0));

		graphCtrller->handleH.setVelocity(Vector(0, 0, 0));
		graphCtrller->handleC.setVelocity(Vector(0, 0, 0));

		ball->setVelocity(Vector(0, 0, 0));
		ball2->setVelocity(Vector(0, 0, 0));
	}

	//Not the first time
	//Onur Her þey bu if'in içinde oluyor
	else if ((effect->keepHapticLoop) == true)
	{
		giveWarning = 0;
		giveWarningG = 0;

		if (dataRec->scenario == STRAIGHT)
		{
			if (trialNo <= PRACTICE_COUNT)
			{
				dataRec->cond = 1;
			}
			else
			{

				dataRec->cond = PERMS_cond[dataRec->perm][(trialNo - PRACTICE_COUNT) % 7];
			}
			if (dataRec->perm == 7)
			{
				dataRec->practice_mode_trials = 6; //mode trial
				dataRec->NUM_TRIALS_PER_COND = 6;
				dataRec->cond = 1;
			}
		}
		else if (dataRec->scenario == MIXED)
		{
			if (trialNo <= PRACTICE_COUNT)
			{
				dataRec->cond = 1;
			}
			else
			{

				dataRec->cond = PERMS_MIXED_COND[dataRec->perm][(trialNo - PRACTICE_COUNT) % 10];
			}

			if (dataRec->perm == 7)
			{
				dataRec->practice_mode_trials = 6;
				dataRec->NUM_TRIALS_PER_COND = 6;
				dataRec->cond = 1;
			}

		}

		if (trialNo == (dataRec->NUM_TRIALS_PER_COND))
		{
			if (dataRec->scenario == STRAIGHT)
			{
				if ((runsPerTrial >= 58700) || (5 - (graphCtrller->boardGr->targetCount / 1000) < 2) || (5 - (graphCtrller->boardGr2->targetCount / 1000) < 2))
				{
					giveWarningG = 2;
				}
				else
				{
					giveWarningG = 0;
				}
			}
			else
			{
				if ((runsPerTrial >= 118700) || (5 - (graphCtrller->boardGr->targetCount / 1000) < 2) || (5 - (graphCtrller->boardGr2->targetCount / 1000) < 2))
				{
					giveWarningG = 2;
				}
				else
				{
					giveWarningG = 0;
				}
			}
		}
		//int aaa = PERMS[dataRec->perm][trialNo%7];


		//if(runs%100==0)
		//	cout<<"perm  "<<dataRec->perm<<"   t "<<trialNo<<"  cond "<<dataRec->cond<<endl;

		/*	graphCtrller->handleH.getPosition().getValue(hhpX, hhpY, hhpZ);
		graphCtrller->handleC.getPosition().getValue(hcpX, hcpY, hcpZ);*/

		if ((runs == 2) && (trialNo == 1))
		{
			firstPosCip[0] = cpX;
			firstPosCip[1] = cpY;
			firstPosCip[2] = cpZ;

			firstPosHip[0] = hpX;
			firstPosHip[1] = hpY;
			firstPosHip[2] = hpZ;
		}

		if ((runs >= 2) && (runs <= timeToGoInit + wait) && (trialNo == 1))
		{
			graphCtrller->boardGr->setInit(false);
			graphCtrller->boardGr2->setInit(false);

			gameStart = false;
			forceFB1 = initialC(1);
			forceFB2 = initialC(2);

			angle_rot = 0.0;
			giveWarning = 1;
		}
		if ((runs == timeToGoInit + wait + 5) && (trialNo == 1))
		{
			graphCtrller->ballGr->setVelocity(Vector(0, 0, 0));
			graphCtrller->ballGr->setAngVelocity(0.0f);
			//CIGIL::
			graphCtrller->ballGr->setAcceleration(Vector(0, 0, 0));
			graphCtrller->ballGr->ball->setAngAcceleration(0.0f);


		}

		boundaryforceHX = 0;
		boundaryforceCX = 0;
		boundaryforceHZ = 0;
		boundaryforceCZ = 0;

		fhw[0] = 0;
		fhw[2] = 0;
		fcw[0] = 0;
		fcw[2] = 0;

		giveWarningW = 0;
		giveWarningB = 0;

		frictionX[0] = 0.0f, frictionX[1] = 0.0f;
		frictionZ[0] = 0.0f, frictionZ[1] = 0.0f;


		//CIGIL 
		graphCtrller->ballGr->ball->setCollision(false);

		xwall[0][0] = wallX1;
		xwall[0][1] = wallZ11;
		xwall[0][2] = wallZ12;

		xwall[1][0] = wallX2;
		xwall[1][1] = wallZ21;
		xwall[1][2] = wallZ22;

		xwall[2][0] = wallX3;
		xwall[2][1] = wallZ31;
		xwall[2][2] = wallZ32;

		Vector wallDisp;
		float x_distance;
		Vector wallCollide;
		Vector force2, force1; //forces of agents without friction

		/*

		if(thereIsWall)
		{
		wallCollide = collX(Vector(ball->posX,ball->posY,ball->posZ));
		float wallx1 = wallCollide[0];
		float wallz1 = wallCollide[1];
		float wallz2 = wallCollide[2];
		ballPoint = xwallColl(wallx1,0,wallz1,wallz2,ballPoint,angle_rot);
		graphCtrller->ballGr->setPosition(ballPoint);
		}
		*/


		//CIGIL
		graphCtrller->ballGr->ball->setCollision(false);
		float upperLimitAngle, lowerLimitAngle;

		// Initialization over
		if (((runs > timeToGoInit + wait) && (trialNo == 1)) || (trialNo != 1))
		{
			if (dataRec->scenario == ROTATIONAL)
			{
				if (dataRec->cond == FULL_CONFLICT)
				{
					LIMIT_ANGLE[dataRec->cond][0] = -60.0f / 180 * PI;//
					LIMIT_ANGLE[dataRec->cond][1] = 60.0f / 180 * PI;//63	
				}
				else
				{
					LIMIT_ANGLE[dataRec->cond][0] = -55.0f / 180 * PI;
					LIMIT_ANGLE[dataRec->cond][1] = 55.0f / 180 * PI;
				}


				lowerLimitAngle = LIMIT_ANGLE[dataRec->cond][0];
				upperLimitAngle = LIMIT_ANGLE[dataRec->cond][1];
			}


			graphCtrller->boardGr->setInit(true);
			graphCtrller->boardGr2->setInit(true);
			gameStart = true;

			if (runs == timeToGoInit + wait + 1)
			{
				runsPerTrial = 0;
				totalConflictTime = 0;
			}

			force1[0] = forceFB1[0];//-f_forceX;
			force1[2] = forceFB1[2];//-f_forceZ;
			force2[0] = forceFB2[0];//-f_forceX;
			force2[2] = forceFB2[2];//-f_forceZ;


			//Onur 
			/*
			Hangi force hangi forcea bakalým

				forceFB1 = hhp -> left device
				forceFB2 = cpx -> algorithm
			*/

			////onur 
			////change the right haptic force with my algorithm

			//force2[0] = Algorithm_force.at(0);
			//force2[2] = Algorithm_force.at(2);
			////


			// AYSE: deleting
			//angle_rot = 
			graphCtrller->ballGr->calculateAngleBallGr(force2, force1);


			////CIGIL: check if there is an angular collision
			////if there is not an angular collision, assign new angle
			//if(!graphCtrller->ballGr->ball->isAngleCollide())
			//{
			//	graphCtrller->ballGr->setAngleBallGr(angle_rot);
			//	graphCtrller->ballGr2->setAngleBallGr(angle_rot);
			//}

		}//if( ( (runs > timeToGoInit+wait) && (trialNo==1) ) || (trialNo!=1) ) bunun bitiþi
		ballPosNext = calcBallPos();
		//CIGIL
		// check the angle of the board 
		angle_rot = graphCtrller->ballGr->getAngleBallGr();
		angle_rotVel = graphCtrller->ballGr->ball->getAngleVel();
		angle_rotAcc = graphCtrller->ballGr->ball->getAngleAcc();
		float boardAngle = fmod(angle_rot, 2 * PI);
		float prevBoardAngle = boardAngle;

		graphCtrller->ballGr->setPosition(ballPosNext);

		if (dataRec->scenario == MIXED)
		{
			if (ballPosNext[0] >= (wallX1 - wallWidth*0.5 - BALL_WIDTH*0.5))
			{
				graphCtrller->ballGr->setPosition(xwallColl(wallX1, 0, wallZ11, wallZ12, ballPosNext, angle_rot, wallWidth, 1));
			}
			else
			{
				if (abs(ballPosNext[2] - wallZ31) >= abs(ballPosNext[2] - wallZ22))
				{
					graphCtrller->ballGr->setPosition(xwallColl(wallX2, 0, wallZ21, wallZ22, ballPosNext, angle_rot, wallWidth23, 2));
				}
				else
				{
					graphCtrller->ballGr->setPosition(xwallColl(wallX3, 0, wallZ31, wallZ32, ballPosNext, angle_rot, wallWidth23, 3));
				}
			}


		}


		if (dataRec->scenario == ROTATIONAL)
		{
			if (dataRec->cond != FULL_CONFLICT)
			{
				graphCtrller->ballGr->setPosition(xwallColl(wallX1_rot, 0, wallZ11_rot, wallZ12_rot, ballPosNext, angle_rot, boundaryThickness, 0));

			}

			else
			{
				/*	float wallx1_col,wallz1_col,wallz2_col;
				if(abs(ballPosNext[2]- wallZ21_rot)>abs(ballPosNext[2]- wallZ12_rot))
				{
				wallx1_col = wallX1_rot;
				wallz1_col = wallZ11_rot;
				wallz2_col = wallZ12_rot;
				}
				else
				{
				wallx1_col = wallX2_rot;
				wallz1_col = wallZ21_rot;
				wallz2_col = wallZ22_rot;

				}

				graphCtrller->ballGr->setPosition(xwallColl(wallx1_col,0,wallz1_col,wallz2_col,ballPosNext,angle_rot,boundaryThickness));
				*/
			}

		}





		//TODO: Correct this
		if (((runs > timeToGoInit + wait) && (trialNo == 1)) || (trialNo != 1))
		{
			if ((dataRec->scenario == ROTATIONAL) && (dataRec->cond == FULL_CONFLICT))
			{
				if (boardAngle >= upperLimitAngle)
				{
					angle_rot = upperLimitAngle;
					errorTime++;

					graphCtrller->ballGr->ball->setAngleCollision(true);

					if (graphCtrller->ballGr->getAngVelocity() > 0)
					{
						graphCtrller->ballGr->setAngVelocity(-0.0001f);
						graphCtrller->ballGr->setAngAcc(-0.00010f);
					}
				}
				else if (boardAngle <= lowerLimitAngle)
				{
					angle_rot = lowerLimitAngle;

					errorTime++;
					graphCtrller->ballGr->ball->setAngleCollision(true);

					if (graphCtrller->ballGr->getAngVelocity() < 0)
					{
						graphCtrller->ballGr->setAngVelocity(0.0001f);
						graphCtrller->ballGr->setAngAcc(0.00010f);
					}
				}
				else
				{
					graphCtrller->ballGr->ball->setAngleCollision(false);
				}


			}
		}


		// Initialization over?

		// TODO: do not directly set the position here... 
		// CIGIL: It does not work
		if ((dataRec->scenario == ROTATIONAL) && (dataRec->cond == FULL_CONFLICT))
		{
			boundaryColl_full(ballPosNext);
		}
		else
		{

			graphCtrller->ballGr->setPosition(boundaryColl(angle_rot, ballPosNext));
		}
		graphCtrller->ballGr->getPosition().getValue(bpX, bpY, bpZ);





		//Onur 

		//kalamar is actually a counter
		//change coordinate system between to game
		//if (kalamar % 1000 == 0){
		//cout << "bpX is: " << bpX  <<  " bpY is: " << 0  << " bpZ is: " << bpZ << "\n" ;
		/*Dont know why its not working
		//std::vector<float>  newPos = baseChange2D.translateNewPos(bpX,bpZ);
		*/
		//}



		graphCtrller->boardGr->setTrial(trialNo);

		float target1X, target2X, target1Z, target2Z;
		float targetAngle1, targetAngle2;

		if (dataRec->scenario == STRAIGHT)
		{
			if (dataRec->perm == 7)
			{
				target1X = PRACTICE_TARGET_POS_A1_S[trialNo % 2][0];
				target2X = PRACTICE_TARGET_POS_A2_S[trialNo % 2][0];

				target1Z = PRACTICE_TARGET_POS_A1_S[trialNo % 2][2];
				target2Z = PRACTICE_TARGET_POS_A2_S[trialNo % 2][2];

				targetAngle1 = PRACTICE_TARGET_ANGLE_A1[trialNo % 2];
				targetAngle2 = PRACTICE_TARGET_ANGLE_A2[trialNo % 2];
			}
			else
			{
				if (trialNo <= PRACTICE_COUNT)
				{   //cout<<"practice"<<endl;
					target1X = PRACTICE_TARGET_POS_A1_S[trialNo % 2][0];
					target2X = PRACTICE_TARGET_POS_A2_S[trialNo % 2][0];

					target1Z = PRACTICE_TARGET_POS_A1_S[trialNo % 2][2];
					target2Z = PRACTICE_TARGET_POS_A2_S[trialNo % 2][2];

					targetAngle1 = PRACTICE_TARGET_ANGLE_A1[trialNo % 2];
					targetAngle2 = PRACTICE_TARGET_ANGLE_A2[trialNo % 2];
				}
				else
				{

					target1X = TARGET_POS_A1[(trialNo - PRACTICE_COUNT) % 7][0];
					target2X = TARGET_POS_A2[(trialNo - PRACTICE_COUNT) % 7][0];

					target1Z = TARGET_POS_A1[(trialNo - PRACTICE_COUNT) % 7][2];
					target2Z = TARGET_POS_A2[(trialNo - PRACTICE_COUNT) % 7][2];

					targetAngle1 = TARGET_ANGLE_A1[(trialNo - PRACTICE_COUNT) % 7];
					targetAngle2 = TARGET_ANGLE_A2[(trialNo - PRACTICE_COUNT) % 7];
				}
			}
		}
		else if (dataRec->scenario == MIXED)
		{
			if (dataRec->perm == 7)
			{
				target1X = PRACTICE_TARGET_POS_A1_M[trialNo % 2][0];
				target2X = PRACTICE_TARGET_POS_A2_M[trialNo % 2][0];

				target1Z = PRACTICE_TARGET_POS_A1_M[trialNo % 2][2];
				target2Z = PRACTICE_TARGET_POS_A2_M[trialNo % 2][2];

				targetAngle1 = PRACTICE_TARGET_ANGLE_A1[trialNo % 2];
				targetAngle2 = PRACTICE_TARGET_ANGLE_A2[trialNo % 2];

			}
			else
			{

				if (trialNo <= PRACTICE_COUNT)
				{
					target1X = PRACTICE_TARGET_POS_A1_M[trialNo % 2][0];
					target2X = PRACTICE_TARGET_POS_A2_M[trialNo % 2][0];

					target1Z = PRACTICE_TARGET_POS_A1_M[trialNo % 2][2];
					target2Z = PRACTICE_TARGET_POS_A2_M[trialNo % 2][2];

					targetAngle1 = PRACTICE_TARGET_ANGLE_A1[trialNo % 2];
					targetAngle2 = PRACTICE_TARGET_ANGLE_A2[trialNo % 2];
				}
				else
				{

					target1X = TARGET_POS_A1[(trialNo - PRACTICE_COUNT) % 10][0];
					target2X = TARGET_POS_A2[(trialNo - PRACTICE_COUNT) % 10][0];

					target1Z = TARGET_POS_A1[(trialNo - PRACTICE_COUNT) % 10][2];
					target2Z = TARGET_POS_A2[(trialNo - PRACTICE_COUNT) % 10][2];

					targetAngle1 = TARGET_ANGLE_A1[(trialNo - PRACTICE_COUNT) % 10];
					targetAngle2 = TARGET_ANGLE_A2[(trialNo - PRACTICE_COUNT) % 10];
				}
			}
		}

		//if (dataRec->cond == PARTIAL_CONFLICT) // alternating targets at right and left
		//{
		//	target1X = TARGET_POS_A1[trialNo%6][0];
		//	target2X = TARGET_POS_A2[trialNo%6][0];

		//	target1Z = TARGET_POS_A1[trialNo%6][2];
		//	target2Z = TARGET_POS_A2[trialNo%6][2];

		//	targetAngle1 = TARGET_ANGLE_A1[trialNo%2];
		//	targetAngle2 = TARGET_ANGLE_A2[trialNo%2];
		//}
		//else if (dataRec->cond == FULL_CONFLICT) // alternating targets at opposite ends, followed by a target at middle 
		//{
		//	target1X = TARGET_POS_A1[trialNo%4][0];
		//	target2X = TARGET_POS_A2[trialNo%4][0];

		//	target1Z = TARGET_POS_A1[trialNo%4][2];
		//	target2Z = TARGET_POS_A2[trialNo%4][2];

		//	targetAngle1 = TARGET_ANGLE_A1[trialNo%4];
		//	targetAngle2 = TARGET_ANGLE_A2[trialNo%4];
		//}
		//else if (dataRec->cond == SINGLE_BLIND_A1 || dataRec->cond == SINGLE_BLIND_A2) // only one target 
		//{
		//	target1X = TARGET_POS_A1[trialNo%6][0];
		//	target2X = TARGET_POS_A2[trialNo%6][0];

		//	target1Z = TARGET_POS_A1[trialNo%6][2];
		//	target2Z = TARGET_POS_A2[trialNo%6][2];

		//	targetAngle1 = TARGET_ANGLE_A1[trialNo%2];
		//	targetAngle2 = TARGET_ANGLE_A2[trialNo%2];
		//}
		//else if (dataRec->cond == HARMONY) // both see the same targets
		//{
		//	target1X = TARGET_POS_A1[trialNo%4][0];
		//	target2X = TARGET_POS_A2[trialNo%4][0];

		//	target1Z = TARGET_POS_A1[trialNo%4][2];
		//	target2Z = TARGET_POS_A2[trialNo%4][2];

		//	targetAngle1 = TARGET_ANGLE_A1[trialNo%2];
		//	targetAngle2 = TARGET_ANGLE_A2[trialNo%2];
		//}

		targetPosA1 = Vector(target1X, 0, target1Z);
		targetPosA2 = Vector(target2X, 0, target2Z);

		targetAngleA1 = targetAngle1;
		targetAngleA2 = targetAngle2;

		if (((runs > timeToGoInit + wait) && (trialNo == 1)) || (trialNo != 1))
		{
			// do checks for hip and cip here so that this is a parking task
			// horizontally out of target area
			if (bpX > target1X + TARGET_REACH_EPS_X || bpX < target1X - TARGET_REACH_EPS_X)
			{
				graphCtrller->boardGr->setArrived(false);
				target1Achieved = 0;
				graphCtrller->boardGr->targetCount = 0;

				setTargetText1("  ");
			}
			// horizontally in, but vertically out of target area
			else if (bpZ > target1Z + TARGET_REACH_EPS_Z || bpZ < target1Z - TARGET_REACH_EPS_Z)
			{
				graphCtrller->boardGr->setArrived(false);
				target1Achieved = 0;
				graphCtrller->boardGr->targetCount = 0;
				setTargetText1("  ");
			}
			// within target area and nearly aligned 
			else if (abs(boardAngle - targetAngle1) < HORIZONTAL_ALIGN_EPS)
			{
				graphCtrller->boardGr->setArrived(true);
				target1Achieved = 1;
				graphCtrller->boardGr->targetCount++;


				sprintf(str, "Wait: %3d", 5 - (graphCtrller->boardGr->targetCount / 1000));
				setTargetText1(str);

				// if the target is acquired or the time 
				if (graphCtrller->boardGr->targetCount >= TIME_TO_WAIT_ON_TARGET)
				{
					trialNo++;

					// AYSE: deal with double incrementation in coinciding targets
					if (dataRec->scenario != ROTATIONAL && target1X == target2X && target1Z == target2Z)
						trialNo--;

					if (dataRec->scenario == ROTATIONAL && targetAngle1 == targetAngle2)
						trialNo--;

					errorTime = 0;
					runsPerTrial = 0;
					totalConflictTime = 0;
					setTargetText1("  ");
					graphCtrller->boardGr->targetCount = 0;
				}
			}
			else
			{
				graphCtrller->boardGr->setArrived(false);
				target1Achieved = 0;
				graphCtrller->boardGr->targetCount = 0;
				setTargetText1("  ");
			}

			// horizontally out of target area
			if (bpX > target2X + TARGET_REACH_EPS_X || bpX < target2X - TARGET_REACH_EPS_X)
			{
				graphCtrller->boardGr2->setArrived(false);
				target2Achieved = 0;
				graphCtrller->boardGr2->targetCount = 0;
				setTargetText2("  ");
			}
			// horizontally in, vertically out of target area
			else if (bpZ > target2Z + TARGET_REACH_EPS_Z || bpZ < target2Z - TARGET_REACH_EPS_Z)
			{
				graphCtrller->boardGr2->setArrived(false);
				target2Achieved = 0;
				graphCtrller->boardGr2->targetCount = 0;
				setTargetText2("  ");
			}
			// within target area and nearly aligned 
			else if (abs(boardAngle - targetAngle2) < HORIZONTAL_ALIGN_EPS)
			{
				graphCtrller->boardGr2->setArrived(true);
				target2Achieved = 1;
				graphCtrller->boardGr2->targetCount++;


				sprintf(str, "Wait: %3d", 5 - (graphCtrller->boardGr2->targetCount / 1000));
				setTargetText2(str);

				if (graphCtrller->boardGr2->targetCount >= TIME_TO_WAIT_ON_TARGET)
				{
					trialNo++;
					errorTime = 0;
					runsPerTrial = 0;
					totalConflictTime = 0;
					setTargetText2("  ");
					graphCtrller->boardGr2->targetCount = 0;
				}
			}
			else
			{
				graphCtrller->boardGr2->setArrived(false);
				target2Achieved = 0;
				graphCtrller->boardGr2->targetCount = 0;
				setTargetText2("  ");
			}

			if (dataRec->scenario == STRAIGHT)
			{
				if (PERMS[dataRec->perm][(trialNo - PRACTICE_COUNT) % 7] == CONDITIONAL_HARMONY)
				{
					if ((abs(bpX - target1X) < 5) && (runsPerTrial <= 5000))
					{
						trialNo = trialNo + 1;
						errorTime = 0;
						runsPerTrial = 0;
						totalConflictTime = 0;
						setTargetText2("  ");
						graphCtrller->boardGr2->targetCount = 0;
						setTargetText1("  ");
						graphCtrller->boardGr->targetCount = 0;
					}
				}
			}

			// do not bound the springs by the board
			// make them extendable to a certain maximum amount...
			graphCtrller->handleH.getPosition().getValue(hhpX, hhpY, hhpZ);
			graphCtrller->handleC.getPosition().getValue(hcpX, hcpY, hcpZ);

			float spring_angle_h, spring_dist_h;
			float spring_angle_c, spring_dist_c;

			float spring_distX_h = hpX - hhpX;
			float spring_distZ_h = hpZ - hhpZ;

			float spring_distX_c = cpX - hcpX;
			float spring_distZ_c = cpZ - hcpZ;

			spring_angle_h = atan2(spring_distX_h, spring_distZ_h);
			spring_dist_h = sqrt(spring_distX_h*spring_distX_h + spring_distZ_h*spring_distZ_h);

			spring_angle_c = atan2(spring_distX_c, spring_distZ_c);
			spring_dist_c = sqrt(spring_distX_c*spring_distX_c + spring_distZ_c*spring_distZ_c);

			/*	hpX = hpX / 2;
			hpZ = hpZ / 2;

			cpX = cpX / 2;
			cpZ = cpZ / 2;*/

			if (spring_dist_h >= MAX_SPRING_DIST)
			{
				hpX = hhpX + MAX_SPRING_DIST * sin(spring_angle_h);
				hpZ = hhpZ + MAX_SPRING_DIST * cos(spring_angle_h);
			}

			if (spring_dist_c >= MAX_SPRING_DIST)
			{
				cpX = hcpX + MAX_SPRING_DIST * sin(spring_angle_c);
				cpZ = hcpZ + MAX_SPRING_DIST * cos(spring_angle_c);
			}


		}// if( ( (runs >timeToGoInit+wait)&&(trialNo==1) ) || (trialNo!=1)) bunun bitiþi

	//fp << hpX << ", " << hvX << ", " << cpX << ", " << cvX << ", ";

		graphCtrller->hip.setPosition(Point(hpX, hpY, hpZ), runs);
		graphCtrller->cip.setPosition(Point(cpX, cpY, cpZ), runs);

		//fp << graphCtrller->hip.getVelocity()[0] << ", " << graphCtrller->cip.getVelocity()[0] << endl;


		//CIGIL:21 subat
		//

		if (graphCtrller->boardGr->getInit())
		{
			scoreTime += 1;
			setScoreText(scoreTime);

			setTrialText(graphCtrller->boardGr->getTrial());
		}
		else
		{
			setScoreText(-30);
		}


		if ((giveWarningW == 1) || (giveWarningB == 1))
		{
			effect->hitWalls = true;
		}
		else
		{
			effect->hitWalls = false;
		}

		if (giveWarning == 1)
		{
			setMessageText("Please hold the device lightly and get ready!Initialization!!! ");
		}

		else
		{
			setMessageText("  ");
		}


		if (giveWarningG == 2)
		{
			setGameOverText(" Game Over ");
		}
		else
		{
			setGameOverText(" ");
		}

		graphCtrller->ballGr->ball->setCollision(false);

		//////FRICTION	

		//CIGIL: delete friction force of agents
		//f_forceX = 0.0f; 
		//f_forceY = 0.0f; 
		//f_forceZ = 0.0f;
		//Vector frictionForce;
		//if( ((runs>timeToGoInit+wait+2)&&(trialNo==1)) ||(trialNo!=1))
		//{
		//	frictionForce = calculateFriction();
		//	f_forceX = frictionForce[0];
		//	f_forceY = frictionForce[1];
		//	f_forceZ = frictionForce[2];

		//	//if(abs(ball->velX) > 0.0f)
		//	//{
		//	//	f_forceX = - ( ball->getMass() * G * COEFFICIENT_FRICTION_STATIC * sign(ball->velZ) ) // static friction
		//	//			   - ( COEFFICIENT_FRICTION_DYNAMIC * ball->velZ );
		//	//	//graphCtrller->ballGr->getMass()*gravitation*friction*1*sign(ball->velX);
		//	//}
		//
		//	//if(abs(ball->velZ)> 0.0f)
		//	//{
		//	//	f_forceZ = - ( ball->getMass() * G * COEFFICIENT_FRICTION_STATIC * sign(ball->velZ) ) // static friction
		//	//			   - ( COEFFICIENT_FRICTION_DYNAMIC * ball->velZ );
		//	//}
		//}



		//if(runs%1000==0)
		//{
		//	cout<<f_forceX<<endl;
		//	cout<<f_forceZ<<endl<<endl;
		//}


		//hhpX = ball->posX-(BALL_WIDTH+IP_RADIUS)*0.5*cos(graphCtrller->ballGr->getAngleBallGr());
		//hhpZ = ball->posZ+(BALL_WIDTH+IP_RADIUS)*0.5*sin(graphCtrller->ballGr->getAngleBallGr());

		//hcpX = ball->posX+(BALL_WIDTH+IP_RADIUS)*0.5*cos(graphCtrller->ballGr->getAngleBallGr());
		//hcpZ = ball->posZ-(BALL_WIDTH+IP_RADIUS)*0.5*sin(graphCtrller->ballGr->getAngleBallGr());

		graphCtrller->handleH.getPosition().getValue(hhpX, hhpY, hhpZ);
		graphCtrller->handleC.getPosition().getValue(hcpX, hcpY, hcpZ);

		graphCtrller->handleH.getVelocity().getValue(hhvX, hhvY, hhvZ);
		graphCtrller->handleC.getVelocity().getValue(hcvX, hcvY, hcvZ);

		graphCtrller->cip.getVelocity().getValue(cvX, cvY, cvZ);
		graphCtrller->hip.getVelocity().getValue(hvX, hvY, hvZ);
		Vector  wVelH, wVelC;
		wVelH = graphCtrller->hip.ip->getWindowVel();
		wVelC = graphCtrller->cip.ip->getWindowVel();


		/*	if( ( (runs>timeToGoInit+wait)&&(trialNo==1) ) || (trialNo!=1))
		{
		fp  << hipvX   <<" " << hipvZ  <<" "
		<< cipvX   <<" " << cipvZ  <<" "
		<< hvX     <<" " << hvZ    <<" "
		<< cvX     <<" " << cvZ    <<" "
		<< wVelH[0]<<" " <<wVelH[2]<<" "
		<< wVelC[0]<<" " <<wVelC[2]<<" "
		<< hpX     <<" " << hpZ    <<" "
		<< cpX     <<" " << cpZ     <<" "
		<< (hhpX- hpX)*kpHN     <<" " << (hhpZ - hpZ)*kpHN     <<" "
		<< (hcpX- cpX)*kpHN     <<" " << (hcpZ - cpZ)*kpHN     <<" "
		<< (hhvX- hvX)*kdN     <<" " << (hhvZ - hvZ)*kdN     <<" "
		<< (hcvX- cvX)*kdN     <<" " << (hcvZ - cvZ)*kdN     <<" "
		<< hhpX     <<" " << hhpZ    <<" "
		<< hcpX     <<" " << hcpZ     <<" "
		<<endl;
		}*/
		//fp	<< forceBall[0] << ", " << forceBall[2] << ", "
		//	<< accX << ", "  << accZ << ", "
		//	<< bvX << ", "   << bvZ << ", "
		//	<< hvX << ", "   << hvZ << ", "
		//	<< cvX << ", "   << cvZ << ", "
		//	<< hhvX << ", "  << hhvZ << ", "
		//	<< hcvX << ", "  << hcvZ << ", "
		//	<< kdN*(hvX-hhvX) << ", "   <<kdN*(hvZ-hhvZ) << ", "<<endl;



		if (((runs > timeToGoInit + wait) && (trialNo == 1)) || (trialNo != 1))
		{

			/*HIP*/
			forceFB1 = Vector(((hhpX - hpX)*kpHN + (hhvX - hvX)*kdN),
				0,
				((hhpZ - hpZ)*kpHN + (hhvZ - hvZ)*kdN));

			fOnHip = forceFB1;

			/*CIP*/
			/*	if(runs % Interpolation_time == 0)
			{
			prevForceX = nextForceX;
			prevForceY = nextForceY;
			prevForceZ = nextForceZ;

			forceFB2[0] = (hcpX- cpX)*kpCN + (hcvX-cvX)*kdN;
			forceFB2[1] = 0;
			forceFB2[2] = (hcpZ- cpZ)*kpCN + (hcvZ-cvZ)*kdN ;

			nextForceX =forceFB2[0];
			nextForceY =forceFB2[1];
			nextForceZ =forceFB2[2];
			}*/

			//forceFB2[0] = prevForceX + (((nextForceX - prevForceX)/Interpolation_time) * fmod(runs,double(Interpolation_time)) ) ;
			//forceFB2[1] = prevForceY + (((nextForceY - prevForceY)/Interpolation_time) * fmod(runs,double(Interpolation_time))) ;
			//forceFB2[2] = prevForceZ + (((nextForceZ - prevForceZ)/Interpolation_time) * fmod(runs,double(Interpolation_time))) ;
			forceFB2[0] = (hcpX - cpX)*kpCN + (hcvX - cvX)*kdN;
			forceFB2[1] = 0;
			forceFB2[2] = (hcpZ - cpZ)*kpCN + (hcvZ - cvZ)*kdN;
			fOnCip = forceFB2;
		}


		if (((runs > timeToGoInit + wait) && (trialNo == 1)) || (trialNo != 1))
		{

			// do we have conflict between agents?
			if ((forceFB1.length() > 0.5) && (forceFB2.length() > 0.5)
				&& (abs(graphCtrller->ballGr->ball->velX) < 0.2) && (abs(graphCtrller->ballGr->ball->velZ) < 0.2)
				&& (abs(graphCtrller->ballGr->getAngVelocity()) < 0.07)
				&& (graphCtrller->boardGr->targetCount == 0) && (graphCtrller->boardGr2->targetCount == 0))
			{
				conflictCounter++;
				totalConflictTime++;
			}
			else
			{
				conflictCounter = 0;
			}

			// display information on screen
			if (conflictCounter >= 5000)
			{
				setTargetText1("CONFLICT!");
				setTargetText2("CONFLICT!");
			}
			if (dataRec->scenario == STRAIGHT)
			{
				if (runsPerTrial >= STRAIGHT_TIME_UP)

				{
					char strW[40];
					sprintf(strW, "Time is up! Last%3d", (60 - runsPerTrial / 1000));
					setWarningText(strW);
				}
				else
				{
					setWarningText(" ");
				}

			}
			else if (dataRec->scenario == MIXED)
			{
				if (runsPerTrial >= MIXED_TIME_UP)
				{
					char strW2[40];
					sprintf(strW2, "Time is up! Last%3d", (120 - runsPerTrial / 1000));
					setWarningText(strW2);
				}
				else
				{
					setWarningText(" ");
				}

			}

			// skip trial if it takes too long
			int TIME_TO_COMPLETE_TRIAL;
			if (dataRec->scenario == STRAIGHT)
				TIME_TO_COMPLETE_TRIAL = TIME_TO_COMPLETE_TRIAL_STRAIGHT;
			else if (dataRec->scenario == ROTATIONAL)
				TIME_TO_COMPLETE_TRIAL = TIME_TO_COMPLETE_TRIAL_ROTATIONAL;
			else if (dataRec->scenario == MIXED)
				TIME_TO_COMPLETE_TRIAL = TIME_TO_COMPLETE_TRIAL_MIXED;



			if (runsPerTrial > TIME_TO_COMPLETE_TRIAL)
			{
				trialNo++;
				errorTime = 0;
				runsPerTrial = 0;
				totalConflictTime = 0;
			}


			force1[0] = forceFB1[0] - f_forceX;
			force1[2] = forceFB1[2] - f_forceZ;

			//Onur
			//Ikýnci haptiðin forcu
			//Buna benim algoritmamýn ürettiði forcu ekleyelim ? 
			force2[0] = forceFB2[0] - f_forceX;
			force2[2] = forceFB2[2] - f_forceZ;






			if (!graphCtrller->ballGr->ball->isAngleCollide())
			{
				/*if(dataRec->scenario == ROTATIONAL)
				{
				graphCtrller->boardGr->setHit(false);
				}*/
				// TODO: AYSE: is this correct????
				//Onur
				// Change the force2 for my algorithm 


				//force2[0] = Algorithm_force.at(0);
				//force2[2] = Algorithm_force.at(2);
				//Onur
				graphCtrller->ballGr->calculateAngleBallGr(force2, force1);
				graphCtrller->ballGr->calculateAngleBallGr(force2, force1);

			}
			//else
			//{
			//	if(dataRec->scenario == ROTATIONAL)
			//	{
			//		/*graphCtrller->boardGr->setHit(true);*/
			//		graphCtrller->ballGr->setAngleBallGr(angle_rot);
			//		graphCtrller->ballGr2->setAngleBallGr(angle_rot);
			//	}
			//}

		}

	}//else if((effect->keepHapticLoop) == true)// Bunun bitiþi burasý Her þey bu if'in içinde oluyor


	hduVector3Dd position;
	HDdouble kStiffness;


	//Onur Burdan sonrasý backforce uyguluyor.



	hdMakeCurrentDevice(effect->hHD1);
	hdGetDoublev(HD_CURRENT_POSITION, position);
	hdGetDoublev(HD_NOMINAL_MAX_STIFFNESS, &kStiffness);


	float fMag = forceFB1.length();
	if (fMag > FORCE_LIMIT)
	{

		forceFB1 = (forceFB1 / fMag) * FORCE_LIMIT;
	}
	forceFB1 = F_SCALE_FACTOR * forceFB1;

	/* AYSE: Render two simple horizontal planes to contstrain user movement to x-z plane. */
	/* the planes are not stiff */
	if (position[1] < HORIZONTAL_PLANE_L) // ground
	{
		forceFB1[1] = kStiffness * (HORIZONTAL_PLANE_L - position[1]);
	}
	else if (position[1] > HORIZONTAL_PLANE_U) // ceiling
	{
		forceFB1[1] = kStiffness * (HORIZONTAL_PLANE_U - position[1]);
	}

	fOnHip_scaled = forceFB1;
	// send all forces to device
	//Onur comment backforce for Haptic 1
	//HDdouble fToD1[3] = { forceFB1[0], forceFB1[1], forceFB1[2] };
	HDdouble fToD1[3] = { 0, 0, 0 };


	effect->PreventWarmMotors(hduVector3Dd(fToD1[0], fToD1[1], fToD1[2]));

	//Onur comment backforce for Left  Haptic hhd1
	//hdSetDoublev(HD_CURRENT_FORCE, fToD1);
	hdSetDoublev(HD_CURRENT_FORCE, fToD1);
	hdScheduleSynchronous(QueryMotorTemp, aMotorTemp,
		HD_DEFAULT_SCHEDULER_PRIORITY);

	// AYSE SERVER CODE commented
	//hdMakeCurrentDevice(effect->hHD2);
	//hdGetDoublev(HD_CURRENT_POSITION, position);
	//hdGetDoublev(HD_NOMINAL_MAX_STIFFNESS, &kStiffness);

	fMag = forceFB2.length();
	if (fMag > FORCE_LIMIT)
	{
		forceFB2 = (forceFB2 / fMag) * FORCE_LIMIT;
	}
	forceFB2 = F_SCALE_FACTOR * forceFB2;

	position = server->positionData;

	if (position[1] < HORIZONTAL_PLANE_L) // ground
	{
		forceFB2[1] = kStiffness * (HORIZONTAL_PLANE_L - position[1]);
	}
	else if (position[1] > HORIZONTAL_PLANE_U) // ceiling
	{
		forceFB2[1] = kStiffness * (HORIZONTAL_PLANE_U - position[1]);
	}



	fOnCip_scaled = forceFB2;

	time_t currentTime;
	currentTime = time(NULL);
	//print forces
	//if( ( (runs>timeToGoInit+wait)&&(trialNo==1) ) || (trialNo!=1))
	//{
	//fp << currentTime << " " << forceFB2[0]   <<" " << forceFB2[1]  <<" "<<forceFB2[2]<<endl;
	//}
	// send all forces to device
	HDdouble fToD2[3] = { forceFB2[0], forceFB2[1], forceFB2[2] };

	// send forces to the client
	server->timestamp = currentTime;


	//Onur comment backforce for Right Haptic hhd2
	//server->forceData = hduVector3Dd(forceFB2[0], forceFB2[1], forceFB2[2]);

	server->forceData = hduVector3Dd(0, 0, 0);
	server->update();




	//	effect->PreventWarmMotors( hduVector3Dd(fToD2[0], fToD2[1], fToD2[2]) );  
	//	hdSetDoublev(HD_CURRENT_FORCE, fToD2);

	//hdScheduleSynchronous(QueryMotorTemp, aMotorTemp,
	//	HD_DEFAULT_SCHEDULER_PRIORITY);


	//HDdouble fToD[3] = { (*computedForces)[0], (*computedForces)[1], (*computedForces)[2] };
	runs++;	cntF++;
	runsPerTrial++;



	if (/*(firstTime == false) && */((effect->keepHapticLoop) == true))
	{
		graphCtrller->hip.getPosition().getValue(hpX, hpY, hpZ);
		graphCtrller->hip.getVelocity().getValue(hvX, hvY, hvZ);
		graphCtrller->cip.getPosition().getValue(cpX, cpY, cpZ);
		graphCtrller->cip.getVelocity().getValue(cvX, cvY, cvZ);



		if (effect->update_stylus_position(myP1) == -1)
		{
			return HD_CALLBACK_DONE;
		}

		if (gameStart)
		{

			dataRec->tick->push_back(getTicks());

			dataRec->errorT->push_back(errorTime);
			dataRec->runsCnt->push_back(runs);
			dataRec->conflictCnt->push_back(totalConflictTime);
			dataRec->runsPerTrialCnt->push_back(runsPerTrial);
			dataRec->trialID->push_back(trialNo);
			dataRec->target1_ACHIEVED->push_back(target1Achieved);
			dataRec->target2_ACHIEVED->push_back(target2Achieved);
			dataRec->targetAgent1->push_back(targetPosA1);
			dataRec->targetAgent2->push_back(targetPosA2);
			dataRec->targetAngleAgent1->push_back(targetAngleA1);
			dataRec->targetAngleAgent2->push_back(targetAngleA2);


			dataRec->timeScore->push_back(scoreTime);
			dataRec->ballP->push_back(Vector(ball->posX, 0, ball->posZ));
			dataRec->ballV->push_back(Vector(ball->velX, 0, ball->velZ));
			dataRec->ballA->push_back(Vector(ball->accX, 0, ball->accZ));
			dataRec->angularPosition->push_back(angle_rot);//ball->angle);
			dataRec->angularVelocity->push_back(angle_rotVel);//ball->angle);
			dataRec->angularAcc->push_back(angle_rot);//ball->angle);
			dataRec->cipP->push_back(Vector(cpX, 0, cpZ));
			dataRec->cipV->push_back(Vector(cvX, 0, cvZ));
			dataRec->hipP->push_back(Vector(hpX, 0, hpZ));
			dataRec->hipV->push_back(Vector(hvX, 0, hvZ));


			dataRec->fOnCIP->push_back(fOnCip);
			dataRec->fOnHIP->push_back(fOnHip);

			dataRec->fOnCIP_SCALED->push_back(fOnCip_scaled);
			dataRec->fOnHIP_SCALED->push_back(fOnHip_scaled);

			dataRec->forceResistance->push_back(fResistance);
			dataRec->forceFriction->push_back(fFriction);
		}

	}
	return HD_CALLBACK_CONTINUE;
}




void HapticCallBack::stopHaptics()
{
	printf("getTicks()=> %ld\n", getTicks());

	server->sendTerminationPackets();
	hdStopScheduler();
	for (int i = 0; i < callbackHandlers.size(); i++)
	{
		hdUnschedule(callbackHandlers.at(i));
		callbackHandlers.pop_back();
	}
	hdDisableDevice(effect->hHD1/*hdGetCurrentDevice()*/);
	//fp.close();

	//dataRec->writeMeToFile();
}

int HapticCallBack::update_stylus_position(Point pt)
{
	SbVec3f myVec;
	Point bPos;
	bPos = graphCtrller->ballGr->getPosition();
	// this uses ball position for tilt
	// comment next line to use hip position for tilt
	pt = bPos;

	myVec[0] = pt[0];
	myVec[1] = pt[1];
	myVec[2] = pt[2];

	myNewTranslation01 = myVec;

	if (myStylusFlag == false)
	{
		float bpX, bpY, bpZ, cpX, cpY, cpZ, hpX, hpY, hpZ;

		graphCtrller->cip.getPosition().getValue(cpX, cpY, cpZ);
		graphCtrller->hip.getPosition().getValue(hpX, hpY, hpZ);


		myNewTranslation = myVec;

		// set the coordinates for the styli on screen
		SbMat myma = { 1, 0, 0, 0,
			0,	1,	0,	0,
			0,	0,	1,	0,
			myVec[0],	0,	0,	1 };
		SbMat myma2 = { 1, 0, 0, 0,
			0,	1,	0,	0,
			0,	0,	1,	0,
			0,	0,	myVec[2],	1 };
		stylusTransMatrix1.setValue(myma);
		stylusTransMatrix2.setValue(myma2);
		int index;

		if (isGameOver() == 1)
		{
			if (effect->curTrial == dataRec->NUM_TRIALS_PER_COND)
			{
				effect->keepHapticLoop = false;

				printf("haptics is stopped\n");

				myStylusFlag = false;
				//Sleep(200);
				effect->stopHaptics();
				free(gServoMotorTemp);
				free(aMotorTemp);
				SoundPlayerThread::aTimer->Stop();
				SoundPlayerThread::aTimer->Enabled = false;

				//Sleep(100);
				exit(0);
			}
		}

		if (effect->isGameFinished)
		{
			effect->curTrial++;
			runs = 0;

			isGameFinished = false;
			sec_init();
		}

		myStylusFlag = true;
	}
	return 1;
}

// AYSE: returns true if all targets in scene are hit 
int HapticCallBack::isGameOver()
{
	int gameOver = 1;

	return gameOver;
}

/*******************************************************************************
Print motor temperature utility.
*******************************************************************************/
void HapticCallBack::PrintMotorTemp(const HDdouble *aMotorTemp, HDint nNumMotors)
{
	HDint i;

	/*   printf("Motor Temperature:");
	for (i = 0; i < nNumMotors; i++)
	{
	printf(" %f", aMotorTemp[i]);
	}*/

	if (bRampDownForces)
	{
		for (i = 0; i < nNumMotors; i++)
		{
			printf(" %f", aMotorTemp[i]);
		}
		printf(" Ramping Down Forces\n");
	}
	else if (bRampUpForces)
	{
		for (i = 0; i < nNumMotors; i++)
		{
			printf(" %f", aMotorTemp[i]);
		}
		printf(" Ramping Up Forces\n");
	}
}

/*******************************************************************************
Scales the force down, taking into account the motor temperature.
*******************************************************************************/
void HapticCallBack::PreventWarmMotors(hduVector3Dd force)
{
	static const HDdouble kRampDownTurnOn = 0.7;
	static const HDdouble kRampDownTurnOff = 0.5;
	static const HDdouble kRampDownRate = 0.999;
	static const HDdouble kRampUpRate = 1.001;
	static HDdouble fAdaptiveClamp = DBL_MAX;

	HDdouble kMaxContinuousForce;
	HDdouble fForceMag;
	HDdouble fMaxTemp = 0;
	HDint i;

	/* Determine if any of the motors are above the temperature threshold.
	All temperature values are normalized between room temperature and the
	overheat temperature. */
	hdGetDoublev(HD_MOTOR_TEMPERATURE, gServoMotorTemp);

	for (i = 0; i < gNumMotors; i++)
	{
		if (gServoMotorTemp[i] > fMaxTemp)
		{
			fMaxTemp = gServoMotorTemp[i];
		}
	}

	fForceMag = hduVecMagnitude(force);

	if (!bRampDownForces && fMaxTemp > kRampDownTurnOn)
	{
		bRampDownForces = TRUE;
		bRampUpForces = FALSE;
		fAdaptiveClamp = fForceMag;
	}
	else if (bRampDownForces && fMaxTemp < kRampDownTurnOff)
	{
		bRampDownForces = FALSE;
		bRampUpForces = TRUE;
	}

	if (bRampDownForces)
	{
		/* Determine a force clamp magnitude that will gradually bring the force
		towards the continuous max force magnitude.  SensAble claims that the
		device can operate continuously at that force without overheating. */
		hdGetDoublev(HD_NOMINAL_MAX_CONTINUOUS_FORCE, &kMaxContinuousForce);

		fAdaptiveClamp *= kRampDownRate;
		fAdaptiveClamp = max(fAdaptiveClamp, kMaxContinuousForce);
	}
	else if (bRampUpForces)
	{
		if (fForceMag < fAdaptiveClamp)
		{
			bRampUpForces = FALSE;
			fAdaptiveClamp = DBL_MAX;
		}

		fAdaptiveClamp *= kRampUpRate;
	}

	/* The HD API presently does not provide a way to convert a cartesian
	force to motor torques, so we cannot adjust the motor torques directly
	to cool just the warm motor.  Therefore, we need to impose an overall
	clamp on the force magnitude in the cartesian domain. */
	if (fForceMag > fAdaptiveClamp)
	{
		hduVecNormalizeInPlace(force);
		hduVecScaleInPlace(force, fAdaptiveClamp);
	}
}

// Check wall collision. 
// the center of the wall in x axis is wallx1, 
// wallz1 and wallz2 are the extreme positions for the z axis 
// it returns the position of the ball corrected after the collision, if exists
Vector xwallColl(float wallx1, float wallx2, float wallz1, float wallz2, Vector ballPosNext, float angle, float wWidth, int wallID)
{
	Vector nonCollidingBallPos;
	float bpX, bpY, bpZ;// current ball position
	float bvX, bvY, bvZ; //current ball velocity
	float baX, baY, baZ; //current ball acceleration

	float bppX, bppY, bppZ; //next ball position


	float Bpnts[48][3]; //coordinates of next points at ball (48 points)
	float bnow[48][3];	//coordinates of current points at ball

	float wallx11 = wallx1 + 0.5*wWidth; // maximum position of the wall in x axis
	float wallx12 = wallx1 - 0.5*wWidth;	// minimum position of the wall in x axis


	graphCtrller->ballGr->getPosition().getValue(bpX, bpY, bpZ); //cigil bppx

	bppX = ballPosNext[0];
	bppY = ballPosNext[1];
	bppZ = ballPosNext[2];

	graphCtrller->ballGr->getVelocity().getValue(bvX, bvY, bvZ);
	graphCtrller->ballGr->getAcceleration().getValue(baX, baY, baZ);

	float lenArray[48][3];
	createLen(lenArray);


	for (int j = 0; j < 48; j++)
	{
		Bpnts[j][0] = bppX + (lenArray[j][0] * cos(angle)) + (lenArray[j][2] * sin(angle));//next
		Bpnts[j][2] = bppZ + (lenArray[j][2] * cos(angle)) - (lenArray[j][0] * sin(angle));
		bnow[j][0] = bpX + (lenArray[j][0] * cos(angle)) + (lenArray[j][2] * sin(angle));
		bnow[j][2] = bpZ + (lenArray[j][2] * cos(angle)) - (lenArray[j][0] * sin(angle));
	}

	int collPtId_x = 0;	// last colliding point id in x-axis 
	int collPtId_z = 0;	// last colliding point id in z-axis 

	graphCtrller->ballGr->ball->setCollision(false);


	//coefficients of line1 and line2 
	//for example line1 is a1*x+b1*z=c1
	float a1 = wallz1 - wallz2;
	float b1 = -(wallx12 - wallx11);
	float c1 = wallx11*a1 + wallz2*b1;
	float a2 = -(wallz1 - wallz2);
	float b2 = -(wallx12 - wallx11);
	float c2 = wallx11 * a2 + wallz1 *b2;



	for (int i = 0; i < 48; i++) {
		if ((Bpnts[i][0] < (boundary[0] + SHIFT_X - boundaryThickness)) && (Bpnts[i][0] > SHIFT_X - (boundary[0] - boundaryThickness)))
		{
			if ((Bpnts[i][0] <= wallx1) && (Bpnts[i][0] >= (wallx1 - 0.5*wWidth)) && (Bpnts[i][2] <= wallz1) && (Bpnts[i][2] >= wallz2))
			{
				graphCtrller->ballGr->ball->setCollision(true);
				collPtId_x = i;

			}

		}
	}
	int collisionDir_x = NO_COLLISION;
	int collisionDir_z = NO_COLLISION;

	//// number of points penetrating into boundary regions 
	int region1 = 0, region2 = 0, region3 = 0, region4 = 0;

	/*       ___region3__
	|			  |
	|			  |
	region1		region2
	|			  |
	|			  |
	|__region4___ |
	*/

	for (int i = 0; i < 48; i++) {
		if ((Bpnts[i][0] < (SHIFT_X + boundary[0] - boundaryThickness)) && (Bpnts[i][0] > SHIFT_X - (boundary[0] - boundaryThickness)))
		{
			if ((Bpnts[i][0] <= wallx1 + 0.5*wWidth) && (Bpnts[i][0] >= wallx1 - 0.5*wWidth) && (Bpnts[i][2] <= wallz1) && (Bpnts[i][2] >= wallz2))
			{
				if ((bnow[i][0] <= wallx1 + 0.5*wWidth) && (bnow[i][0] >= wallx1 - 0.5*wWidth) && (bnow[i][2] <= wallz1) && (bnow[i][2] >= wallz2))
				{

					//collision = 1;   CIGIL
					graphCtrller->ballGr->ball->setCollision(true);

					float future_x = Bpnts[i][0];//future
					float future_z = Bpnts[i][2];//future

					float current_x = bnow[i][0]; //now
					float current_z = bnow[i][2];//now


					if ((((a1*future_x) + (b1*future_z)) < c1) &&
						(((a2*future_x) + (b2*future_z)) > c2) &&
						(((a1*current_x) + (b1*current_z)) < c1) &&
						(((a2*current_x) + (b2*current_z)) > c2))
					{
						region1++;
						//collision = 1;
						graphCtrller->ballGr->ball->setCollision(true);

						collPtId_x = i;

					}
					else if ((((a1*future_x) + (b1*future_z)) > c1) && (((a2*future_x) + (b2*future_z)) < c2) && (((a1*current_x) + (b1*current_z)) > c1) && (((a2*current_x) + (b2*current_z)) < c2))
					{
						region2++;
						graphCtrller->ballGr->ball->setCollision(true);

						collPtId_x = i;

					}
					else if ((((a1*future_x) + (b1*future_z)) > c1) && (((a2*future_x) + (b2*future_z)) > c2) && (((a1*current_x) + (b1*current_z)) > c1) && (((a2*current_x) + (b2*current_z)) > c2))
					{
						region3++;
						graphCtrller->ballGr->ball->setCollision(true);
						collPtId_z = i;
					}
					else if ((((a1*future_x) + (b1*future_z)) < c1) && (((a2*future_x) + (b2*future_z)) < c2) && (((a1*current_x) + (b1*current_z)) < c1) && (((a2*current_x) + (b2*current_z)) < c2))
					{
						region4++;
						graphCtrller->ballGr->ball->setCollision(true);
						collPtId_z = i;
					}

					else if ((((a1*future_x) + (b1*future_z)) < c1) && (((a2*future_x) + (b2*future_z)) == c2) && (((a1*current_x) + (b1*current_z)) < c1) && (((a2*current_x) + (b2*current_z)) == c2))
					{
						region1++; region4++;
						graphCtrller->ballGr->ball->setCollision(true);
						collPtId_x = i; collPtId_z = i;
					}
					else if ((((a1*future_x) + (b1*future_z)) > c1) && (((a2*future_x) + (b2*future_z)) == c2) && (((a1*current_x) + (b1*current_z)) > c1) && (((a2*current_x) + (b2*current_z)) == c2))
					{
						region3++; region2++;
						graphCtrller->ballGr->ball->setCollision(true);
						collPtId_x = i; collPtId_z = i;
					}
					else if ((((a1*future_x) + (b1*future_z)) == c1) && (((a2*future_x) + (b2*future_z)) > c2) && (((a1*current_x) + (b1*current_z)) == c1) && (((a2*current_x) + (b2*current_z)) > c2))
					{
						region1++;
						region3++;
						graphCtrller->ballGr->ball->setCollision(true);
						collPtId_x = i; collPtId_z = i;
					}
					else if ((((a1*future_x) + (b1*future_z)) == c1) && (((a2*future_x) + (b2*future_z)) < c2) && (((a1*current_x) + (b1*current_z)) == c1) && (((a2*current_x) + (b2*current_z)) < c2))
					{
						region2++;
						region4++;
						graphCtrller->ballGr->ball->setCollision(true);
						collPtId_x = i; collPtId_z = i;

					}
					else if ((((a1*future_x) + (b1*future_z)) == c1) && (((a2*future_x) + (b2*future_z)) < c2) && (((a1*current_x) + (b1*current_z)) == c1) && (((a2*current_x) + (b2*current_z)) == c2))
					{
						region1++;
						region2++;
						region4++;
						region3++;
						graphCtrller->ballGr->ball->setCollision(true);
						graphCtrller->ballGr->ball->setCollision(true);
						collPtId_x = i; collPtId_z = i;
					}


				}
			}
		}
	}


	if (graphCtrller->ballGr->ball->isCollide())
	{
		//since it is wall collision forcetypes=2
		crunsw++;
		errorTime++;
		setforces(region1, region2, region3, region4, 2);//18 eylul

		giveWarningW = 1;
		graphCtrller->boardGr->setHitObstacle(true);

		if (dataRec->scenario == MIXED)
		{
			if (wallID == 1)
			{
				graphCtrller->wall1->setHit(true);
			}
			else  if (wallID == 2)
			{
				graphCtrller->wall2->setHit(true);
				//cout<<"collision2"<<endl;
			}
			else  if (wallID == 3)
			{
				graphCtrller->wall3->setHit(true);
				//cout<<"collision3"<<endl;
			}
		}
	}
	else
	{
		fhw[0] = 0.0f;
		fcw[0] = 0.0f;
		fhw[2] = 0.0f;
		fcw[2] = 0.0f;

		/*	fhw_arr[0][0]=0.0f;
		fcw_arr[0][0]=0.0f;
		fhw_arr[0][2]=0.0f;
		fcw_arr[0][2]=0.0f;

		fhw_arr[1][0]=0.0f;
		fcw_arr[1][0]=0.0f;
		fhw_arr[1][2]=0.0f;
		fcw_arr[1][2]=0.0f;*/

		graphCtrller->wall1->setHit(false);
		graphCtrller->wall2->setHit(false);
		graphCtrller->wall3->setHit(false);
		graphCtrller->boardGr->setHitObstacle(false);
		giveWarningW = 0;
		crunsw = 0;
	}






	if ((region1 > 0) && (region1 > region2))
	{
		collisionDir_x = COLLISION_LEFT;  //1
	}
	else if ((region2 > 0) && (region2 > region1))
	{
		collisionDir_x = COLLISION_RIGHT;
	}

	if ((region3 > 0) && (region3 > region4))
	{
		collisionDir_z = COLLISION_UP;
	}
	else if ((region4 > 0) && (region4 > region3))
	{
		collisionDir_z = COLLISION_DOWN;
	}



	for (int j = 0; j < 48; j++)
	{

		if ((collisionDir_x == COLLISION_LEFT) && (collPtId_x == j) && (graphCtrller->ballGr->ball->isCollide()))
		{
			nonCollidingBallPos[0] = (wallx1 - 0.5*wWidth) + ((-lenArray[j][0] * cos(angle)) + (-lenArray[j][2] * sin(angle)));
		}
		else if ((collisionDir_x == COLLISION_RIGHT) && (collPtId_x == j) && (graphCtrller->ballGr->ball->isCollide()))
		{
			nonCollidingBallPos[0] = wallx1 + 0.5*wWidth + ((-lenArray[j][0] * cos(angle)) + (-lenArray[j][2] * sin(angle)));
		}
		else if ((collisionDir_z == COLLISION_UP) && (collPtId_z == j) && (graphCtrller->ballGr->ball->isCollide()))
		{
			nonCollidingBallPos[2] = (wallz1)+((-lenArray[j][2] * cos(angle)) - (-lenArray[j][0] * sin(angle)));
		}
		else if ((collisionDir_z == COLLISION_DOWN) && (collPtId_z == j) && (graphCtrller->ballGr->ball->isCollide()))
		{
			nonCollidingBallPos[2] = (wallz2)+((-lenArray[j][2] * cos(angle)) - (-lenArray[j][0] * sin(angle)));
		}
		else if ((collisionDir_x == COLLISION_LEFT) && (collisionDir_z == COLLISION_UP) && (collPtId_x == j) && (collPtId_z == j) && (graphCtrller->ballGr->ball->isCollide()))
		{
			nonCollidingBallPos[0] = (wallx1 - 0.5*wWidth) + ((-lenArray[j][0] * cos(angle)) + (-lenArray[j][2] * sin(angle)));
			nonCollidingBallPos[2] = (wallz1)+((-lenArray[j][2] * cos(-angle)) - (-lenArray[j][0] * sin(-angle)));
		}
		else if ((collisionDir_x == COLLISION_LEFT) && (collisionDir_z == COLLISION_DOWN) && (collPtId_x == j) && (graphCtrller->ballGr->ball->isCollide()) && (collPtId_z == j))
		{
			nonCollidingBallPos[0] = (wallx1 - 0.5*wWidth) + ((-lenArray[j][0] * cos(angle)) + (-lenArray[j][2] * sin(angle)));
			nonCollidingBallPos[2] = (wallz2)+((-lenArray[j][2] * cos(angle)) - (-lenArray[j][0] * sin(angle)));

		}
		else if ((collisionDir_x == COLLISION_RIGHT) && (collisionDir_z == COLLISION_UP) && (collPtId_x == j) && (graphCtrller->ballGr->ball->isCollide()) && (collPtId_z == j))
		{
			nonCollidingBallPos[0] = (wallx1 + 0.5*wWidth) + ((-lenArray[j][0] * cos(angle)) + (-lenArray[j][2] * sin(angle)));
			nonCollidingBallPos[2] = (wallz1)+((-lenArray[j][2] * cos(angle)) - (-lenArray[j][0] * sin(angle)));

		}
		else if ((collisionDir_x == COLLISION_RIGHT) && (collisionDir_z == COLLISION_DOWN) && (collPtId_x == j) && (graphCtrller->ballGr->ball->isCollide()) && (collPtId_z == j))
		{
			nonCollidingBallPos[0] = (wallx1 + 0.5*wWidth) + ((-lenArray[j][0] * cos(angle)) + (-lenArray[j][2] * sin(angle)));
			nonCollidingBallPos[2] = (wallz2)+((-lenArray[j][2] * cos(angle)) - (-lenArray[j][0] * sin(angle)));
		}

		else if ((!graphCtrller->ballGr->ball->isCollide()) && (collisionDir_x == NO_COLLISION) && (collisionDir_z == NO_COLLISION) && (collPtId_x == 0) && (collPtId_z == 0))
		{
			nonCollidingBallPos[0] = bpX;
			nonCollidingBallPos[1] = bpY;
			nonCollidingBallPos[2] = bpZ;
		}
	}

	if ((region1 != 0) && (region1 > region2))
	{
		if ((bvX > 0))
		{
			bvX = -0.001f;
			baX = -0.001f;
			graphCtrller->ballGr->setVelocity(Vector(bvX, bvY, bvZ));
			graphCtrller->ballGr->setAcceleration(Vector(baX, baY, baZ));
		}

	}

	else if ((region2 != 0) && (region2 > region1))
	{
		if ((bvX < 0))
		{
			bvX = 0.001f;
			baX = 0.001f;
			graphCtrller->ballGr->setVelocity(Vector(bvX, bvY, bvZ));
			graphCtrller->ballGr->setAcceleration(Vector(baX, baY, baZ));
		}
	}

	if ((region3 != 0) && (region3 > region4))
	{
		if ((bvZ < 0))
		{
			bvZ = 0.001f;
			baZ = 0.001f;
			graphCtrller->ballGr->setVelocity(Vector(bvX, bvY, bvZ));
			graphCtrller->ballGr->setAcceleration(Vector(baX, baY, baZ));
		}
	}

	else if ((region4 != 0) && (region4 > region3))
	{
		if ((bvZ > 0))
		{
			bvZ = -0.001f;
			baZ = -0.001f;
			graphCtrller->ballGr->setVelocity(Vector(bvX, bvY, bvZ));
			graphCtrller->ballGr->setAcceleration(Vector(baX, baY, baZ));
		}
	}


	nonCollidingBallPos[1] = bpY;//ballPosNext[1];

	if (collPtId_x == 0)
	{
		nonCollidingBallPos[0] = bpX;//ballPosNext[0];
	}
	if (collPtId_z == 0)
	{
		nonCollidingBallPos[2] = bpZ;//ballPosNext[2];
	}

	return nonCollidingBallPos;
}


// find the closest wall
Vector collX(Vector ball)
{
	Vector answer;
	float xdistance = abs(ball[0] - xwall[0][0]);
	for (int i = 0; i < wall_n; i++)
	{
		if ((ball[2] < xwall[i][1] + BALL_DEPTH) && (ball[2] > xwall[i][2] - BALL_DEPTH))
		{
			if (xdistance >= abs(ball[0] - xwall[i][0]))
			{
				xdistance = abs(ball[0] - xwall[i][0]);
				answer[0] = xwall[i][0];
				answer[1] = xwall[i][1];
				answer[2] = xwall[i][2];
				wall_hit = i + 1;
			}
		}
	}
	return answer;
}




Vector calcBallPos()
{
	OnurRuns++;
	// IP positions	
	float cpX, cpY, cpZ, hpX, hpY, hpZ;
	// IP positions
	float cvX = 0, cvY = 0, cvZ = 0, hvX = 0, hvY = 0, hvZ = 0;

	// handle positions
	float hcpX = 0, hcpY = 0, hcpZ = 0, hhpX = 0, hhpY = 0, hhpZ = 0;
	// handle velocities
	float hcvX = 0, hcvY = 0, hcvZ = 0, hhvX = 0, hhvY = 0, hhvZ = 0;

	// ball position and velocity
	float bpX, bpY, bpZ, bvX = 0, bvY = 0, bvZ = 0;





	float ballMass;
	float static_friction_magnitude, kinetic_friction_magnitude;

	float accX = 0, accY = 0, accZ = 0;

	Vector forceBall;
	forceBall[0] = 0.0f;
	forceBall[1] = 0.0f;
	forceBall[2] = 0.0f;


	float angle_rot = graphCtrller->ballGr->getAngleBallGr();

	graphCtrller->cip.getPosition().getValue(cpX, cpY, cpZ);
	graphCtrller->hip.getPosition().getValue(hpX, hpY, hpZ);

	graphCtrller->cip.getVelocity().getValue(cvX, cvY, cvZ);
	graphCtrller->hip.getVelocity().getValue(hvX, hvY, hvZ);

	graphCtrller->ballGr->getPosition().getValue(bpX, bpY, bpZ);
	graphCtrller->ballGr->getVelocity().getValue(bvX, bvY, bvZ);
















	ballMass = graphCtrller->ballGr->getMass();


	// calculate handle positions 
	hhpX = bpX - (BALL_WIDTH)*0.5*cos(angle_rot);
	hhpZ = bpZ + (BALL_WIDTH)*0.5*sin(angle_rot);

	hcpX = bpX + (BALL_WIDTH)*0.5*cos(angle_rot);
	hcpZ = bpZ - (BALL_WIDTH)*0.5*sin(angle_rot);


	graphCtrller->handleC.getVelocity().getValue(hcvX, hcvY, hcvZ);
	/*Ýlk yöntem
		(if (OnurRuns == 1) {
		error1x = 0;
		error1x = 0;
		error1y = 0;
		error1z = 0;

		error2x = 0;
		error2y = 0;
		error2z = 0;

		double Xsrc_max = 82;	//Onur Code x axis max up-down
		double Xsrc_min = 0;	//Onur Code x axis min up-down
		double Ysrc_max = 200;	//Onur Code y axis max left-right
		double Ysrc_min = 0;	//Onur Code y axis min left-right



		double Xres_min = -100; //MazeGame x axis min left-right
		double Xres_max = +100;	//MazeGame x axis max left-right
		double Zres_min = -41;	//MazeGame z axis min up-down
		double Zres_max = +41;	//MazeGame z axis max up-down



		double res_z = ((path.dtargetsX.at(path.dtargetsX.size() - 1) - Xsrc_min) / (Xsrc_max - Xsrc_min) * (Zres_max - Zres_min) + Zres_min);
		double res_x = ((path.dtargetsY.at(path.dtargetsY.size() - 1) - Ysrc_min) / (Ysrc_max - Ysrc_min) * (Xres_max - Xres_min) + Xres_min);



		initX = res_x;
		initY = 0;
		initZ = res_z;

		dx = (initX - hcpX) / (10000);
		dy = (initY - hcpY) / (10000);
		dz = (initZ - hcpZ) / (10000);

		desiredX = hcpX;
		desiredY = hcpY;
		desiredZ = hcpZ;
	}

	error1x = error2x;
	error1y = error2y;
	error1z = error2z;

	desiredX += dx;
	desiredY += dy;
	desiredZ += dz;

	error2x = desiredX - hcpX;
	error2y = desiredY - hcpY;
	error2z = desiredZ - hcpZ;


	Algorithm_force[0] = kpHN * error2x + kdN * (error2x - error1x);
	Algorithm_force[1] = 0;
	Algorithm_force[2] = kpHN * error2z + kdN * (error2z - error1z);


	*/

	//Onur
	//Dont even think about to change without saving somewhere 
	//Code work right,2 times checked
	//Dönüþüm Maze Gameden onurun koda
	/*
	double Xsrc_min = -100; // MazeGame x axis min left-right
	double Xsrc_max = +100;	// MazeGame x axis max left-right
	double Zsrc_min = -41;	// MazeGame z axis min up-down
	double Zsrc_max = +41;	// MazeGame z axis max up-down



	double Xres_max = 82;	// Onur Code x axis max up-down
	double Xres_min = 0;	// Onur Code x axis min up-down
	double Yres_max = 200;	// Onur Code y axis max left-right
	double Yres_min = 0;	// Onur Code y axis min left-right




	double res_x = ((hcpZ)-Zsrc_min) / (Zsrc_max - Zsrc_min) * (Xres_max - Xres_min) + Xres_min;
	double res_y = ((hcpX)-Xsrc_min) / (Xsrc_max - Xsrc_min) * (Yres_max - Yres_min) + Yres_min;
	*/

	//Onurun koddan maze game'e


	/*Çalýþmýyor bütün yol için
	double Xsrc_max = 82;	//Onur Code x axis max up-down
	double Xsrc_min = 0;	//Onur Code x axis min up-down
	double Ysrc_max = 200;	//Onur Code y axis max left-right
	double Ysrc_min = 0;	//Onur Code y axis min left-right



	double Xres_min = -100; //MazeGame x axis min left-right
	double Xres_max = +100;	//MazeGame x axis max left-right
	double Zres_min = -41;	//MazeGame z axis min up-down
	double Zres_max = +41;	//MazeGame z axis max up-down

	graphCtrller->handleC.getVelocity().getValue(hcvX, hcvY, hcvZ);



	/*Çalýþmýyor bütün yol için
	double euclidean_distance = 999999999;
	double x_ref = 999999999;
	double res_z;
	double res_x;
	double min_res_z;
	double min_res_x;
	int j = 0;

	for (int i = 0; i < path.dtargetsX.size(); i++) {
		if (path.isVisited.at(i) == true) {
			continue;
		}
		res_z = (	(path.dtargetsX.at(i)	-Xsrc_min) / (Xsrc_max - Xsrc_min) * (Zres_max - Zres_min) + Zres_min);
		res_x = (	(path.dtargetsY.at(i)	-Ysrc_min) / (Ysrc_max - Ysrc_min) * (Xres_max - Xres_min) + Xres_min);

		euclidean_distance = sqrt(pow((look_ahead_pos_x - res_x), 2) + pow((look_ahead_pos_z - res_z), 2));

		if (euclidean_distance < 15) {
			path.isVisited.at(i) = true;
			continue;
		}


		if (euclidean_distance < x_ref) {

			x_ref = euclidean_distance;
			j = i;
			min_res_x = res_x;
			min_res_z = res_z;
		}


	}
	*/

	//Çalýþmýyor son hedef için

	double Xsrc_max = 82;	//Onur Code x axis max up-down 
	double Xsrc_min = 0;	//Onur Code x axis min up-down
	double Ysrc_max = 200;	//Onur Code y axis max left-right
	double Ysrc_min = 0;	//Onur Code y axis min left-right



	double Xres_min = -100; //MazeGame x axis min left-right 
	double Xres_max = +100;	//MazeGame x axis max left-right
	double Zres_min = -41;	//MazeGame z axis min up-down
	double Zres_max = +41;	//MazeGame z axis max up-down



	double look_ahead_pos_x;
	double look_ahead_pos_z;
	double look_ahead_time = 0.001;


	look_ahead_pos_x = hcpX + (look_ahead_time * hcvX);
	look_ahead_pos_z = hcpZ + (look_ahead_time * hcvZ);


	double res_z = ((path.dtargetsX.at(path.dtargetsX.size() - 1) - Xsrc_min) / (Xsrc_max - Xsrc_min) * (Zres_max - Zres_min) + Zres_min);
	double res_x = ((path.dtargetsY.at(path.dtargetsY.size() - 1) - Ysrc_min) / (Ysrc_max - Ysrc_min) * (Xres_max - Xres_min) + Xres_min);

	Algorithm_force.at(0) = 0.8 * (look_ahead_pos_x - res_x);
	Algorithm_force.at(2) = 0.8 * (look_ahead_pos_z - res_z);








	if (runs % 1000 == 0) {
		cout << "hcX is:  " << hcpX << "\t hcvZ is: " << hcpZ << "\n";
		cout << "res_x is:" << res_x << "\tres_z is:" << res_z << "\n";
		//	cout << "res_x is: " << path.dtargetsX.at(j) << "res_z is: " << path.dtargetsY.at(j) << "\n";
		cout << "algorithm bpx force is: " << Algorithm_force[0] << "algorithm bpz force is: " << Algorithm_force[2] << "\n\n\n";
		//cout << "lefthaptic bpx force is: " << kpHN*(hpX-hhpX) + kdN*(hvX-hhvX) << " left haptic bpz force is: "  << kpHN*(hpZ-hhpZ) + kdN*(hvZ-hhvZ) << "\n\n\n"; 
	}

	cpX = +(Algorithm_force.at(0) / kpHN) - hcpX;
	cpZ = +(Algorithm_force.at(2) / kpHN) - hcpZ;


	//Onur Left haptic working right haptic is my algorithm
	forceBall[0] = kpHN*(hpX - hhpX) + kdN*(hvX - hhvX)
		+  /*deviceden gelen force u 0 la yerine benim algorithmamýn ürettiði forcu koy*/ kpHN*(cpX - hcpX) + kdN*(cvX - hcvX) /*Algorithm_force.at(0)*/ + fcw[0] + boundaryforceCX;

	forceBall[2] = kpHN*(hpZ - hhpZ) + kdN*(hvZ - hhvZ)
		+/*Onur Right(Device HHD2 ) deviceden gelen force u 0 la yerine benim algorithmamýn ürettiði forcu koy */ kpHN*(cpZ - hcpZ) + kdN*(cvZ - hcvZ) /*Algorithm_force.at(2)*/ + fcw[2] + boundaryforceCZ;
	//Onur


	// Force'un eski hali
	//forceBall[0] = kpHN*(hpX-hhpX) + kdN*(hvX-hhvX) 
	//+ kpHN*(cpX-hcpX) + kdN*(cvX-hcvX)+fcw[0] + boundaryforceCX;

	//forceBall[2] = kpHN*(hpZ-hhpZ) + kdN*(hvZ-hhvZ)
	//+kpHN*(cpZ-hcpZ) + kdN*(cvZ-hcvZ)+fcw[2] + boundaryforceCZ; 

	//Onur


	fResistance[0] = fcw[0] + boundaryforceCX;
	fResistance[2] = fcw[2] + boundaryforceCZ;

	//graphCtrller->cip.setPosition(Point(cpX, cpY, cpZ), runs);

	graphCtrller->handleH.setPosition(Point(hhpX, hhpY, hhpZ), runs);
	graphCtrller->handleC.setPosition(Point(hcpX, hcpY, hcpZ), runs);

	graphCtrller->handleH.getVelocity().getValue(hvX, hvY, hvZ);
	graphCtrller->handleC.getVelocity().getValue(hcvX, hcvY, hcvZ);

	//fp	<< forceBall[0] << ", " << forceBall[2] << ", ";

	static_friction_magnitude = ballMass * G * COEFFICIENT_FRICTION_STATIC;
	kinetic_friction_magnitude = ballMass * G * COEFFICIENT_FRICTION_STATIC;

	if (abs(bvX) == 0)
	{
		if (abs(forceBall[0]) >= abs(static_friction_magnitude * cos(angle_rot)))
		{
			fFriction[0] = -(abs(static_friction_magnitude * cos(angle_rot)) * SSIGN(bvX));
			forceBall[0] += -(abs(static_friction_magnitude * cos(angle_rot)) * SSIGN(bvX)); // static friction, when velocity is zero 
		}
		else
		{
			fFriction[0] = -forceBall[0];
			forceBall[0] = 0.0f;
		}

	}
	else
	{
		fFriction[0] = -(abs(kinetic_friction_magnitude *cos(angle_rot)) * SSIGN(bvX));
		forceBall[0] += -(abs(kinetic_friction_magnitude *cos(angle_rot)) * SSIGN(bvX));// kinetic friction, when there is a translation and velocity is nonzero					 			   
	}

	if (abs(bvZ) == 0)
	{
		if (abs(forceBall[2]) >= abs(static_friction_magnitude * sin(angle_rot)))
		{
			fFriction[2] = -(abs(static_friction_magnitude *sin(angle_rot)) * SSIGN(bvZ));
			forceBall[2] += -(abs(static_friction_magnitude *sin(angle_rot)) * SSIGN(bvZ)); // static friction
		}
		else
		{
			fFriction[2] = -forceBall[2];
			forceBall[2] = 0.0f;
		}
	}
	else
	{
		fFriction[2] = -(abs(kinetic_friction_magnitude *sin(angle_rot))* SSIGN(bvZ));
		forceBall[2] += -(abs(kinetic_friction_magnitude *sin(angle_rot))* SSIGN(bvZ));	// static friction
	}








	accX = forceBall[0] / ballMass;//* .0010;
	accZ = forceBall[2] / ballMass;//* .0010;

	bvX += accX * DELTA_T;
	bvZ += accZ * DELTA_T;

	graphCtrller->ballGr->setVelocity(Vector(bvX, bvY, bvZ));
	graphCtrller->ballGr->setAcceleration(Vector(accX, accY, accZ));

	float fnet = sqrt(forceBall[0] * forceBall[0] + forceBall[2] * forceBall[2]);
	float vnet = sqrt(bvX*bvX + bvZ*bvZ);
	float anet = sqrt(accX*accX + accZ*accZ);

	// don't translate in rotational scenario CIGIL
	/*if (dataRec->scenario == ROTATIONAL)
	{
	bvX = 0;
	bvZ = 0;
	}*/

	bpX += bvX * DELTA_T + 0.5 * accX * pow(DELTA_T, 2);
	bpZ += bvZ * DELTA_T + 0.5 * accZ * pow(DELTA_T, 2);


	//if (runs % 1000 == 0)
	//{		
	//	cout << "X - F_after = " << forceBall[0] << " a = " << accX << " v = " << bvX << endl;
	//	cout << "Z - F_after = " << forceBall[2] << " a = " << accZ << " v = " << bvZ << endl << endl;
	//}

	//fp	<< forceBall[0] << ", " << forceBall[2] << ", "
	//	<< accX << ", "  << accZ << ", "
	//	<< bvX << ", "   << bvZ << ", "
	//	<< hvX << ", "   << hvZ << ", "
	//	<< cvX << ", "   << cvZ << ", "
	//	<< hhvX << ", "  << hhvZ << ", "
	//	<< hcvX << ", "  << hcvZ << ", "
	//	<< kdN*(hvX-hhvX) << ", "   <<kdN*(hvZ-hhvZ) << ", "<<endl;


	return Point(bpX, bpY, bpZ);
}




void setforces(int r11, int r22, int r33, int r44, int forcetypes)
{
	//CIGIL: increase magnitude of constant force
	float constantf = 1.0f;
	float constantfb = 1.0f;
	float repeat = 500.0f;

	int repeat2 = 500;

	//boundary reaction force
	if (forcetypes == 1)
	{
		if ((r44 > 0) && (r44 > r33))
		{
			boundaryforceHZ = constantfb*(r44 - r33);
			boundaryforceCZ = constantfb*(r44 - r33);
		}
		else if ((r33 > 0) && (r33 > r44))
		{
			boundaryforceCZ = -constantfb*(r33 - r44);
			boundaryforceHZ = -constantfb*(r33 - r44);
		}


		if ((r11 > 0) && (r11 > r22))
		{
			boundaryforceHX = constantfb*(r11 - r22);
			boundaryforceCX = constantfb*(r11 - r22);
		}
		else if ((r22 > 0) && (r22 > r11))
		{
			boundaryforceCX = -constantfb*(r22 - r11);
			boundaryforceHX = -constantfb*(r22 - r11);
		}

	}



	//wall reaction force
	if ((forcetypes == 2))
	{
		if ((r44 > 0) && (r44 > r33))
		{
			fhw[2] = -constantf*(r44 - r33);
			fcw[2] = -constantf*(r44 - r33);
		}
		else if ((r33 > 0) && (r33 > r44))
		{
			fhw[2] = constantf*(r33 - r44);
			fcw[2] = constantf*(r33 - r44);
		}


		if ((r11 > 0) && (r11 > r22))
		{
			fhw[0] = -constantf*(r11 - r22);
			fcw[0] = -constantf*(r11 - r22);
		}
		else if ((r22 > 0) && (r22 - r11))
		{
			fhw[0] = constantf*(r22 - r11);
			fcw[0] = constantf*(r22 - r11);
		}

	}

}
void calcForceFromBoardRot(Point magPt, Vector &fBoard)
{
	fBoard[0] = -magPt[0] / (boundary[0] - boundaryThickness) * maxForceFromBoard;
	fBoard[1] = 0;
	fBoard[2] = -magPt[2] / (boundary[2] - boundaryThickness) * maxForceFromBoard;
}

float InvSqrt(float x)
{
	float xhalf = 0.5f*x;
	int i = *(int*)&x;
	i = 0x5f3759df - (i >> 1);
	x = *(float*)&i;
	return x*(1.5f - xhalf*x*x);
}



Vector boundaryColl_full(Vector ballPosNext)
{
	Vector nonCollidingBallPos;

	float bpX, bpY, bpZ; // current ball position 
	float bvX, bvY, bvZ; // current ball velocity 
	float baX, baY, baZ; // current ball acceleration 

	float bpX_next, bpY_next, bpZ_next; // next ball position 

	int r1 = 0, r2 = 0, r3 = 0, r4 = 0;

	int collPtId_z = 0;	// last colliding point id in z-axis 
	int collisionDir_z = NO_COLLISION; // the direction of collision in z-axis

	int collPtId_x = 0;
	int collisionDir_x = NO_COLLISION;

	graphCtrller->ballGr->getPosition().getValue(bpX, bpY, bpZ);
	graphCtrller->ballGr->getVelocity().getValue(bvX, bvY, bvZ);
	graphCtrller->ballGr->getAcceleration().getValue(baX, baY, baZ);



	bpX_next = ballPosNext[0];
	bpY_next = ballPosNext[1];
	bpZ_next = ballPosNext[2];

	int safe_region = .4;



	if ((bpX >= (SHIFT_X + safe_region)) && (bpX_next >= (SHIFT_X + safe_region)))
	{
		nonCollidingBallPos[0] = SHIFT_X + safe_region;
		graphCtrller->ballGr->ball->setCollision(true);


		if (bvX > 0)
		{
			bvX = -0.001;		baX = -0.001;
			graphCtrller->ballGr->setVelocity(Vector(bvX, bvY, bvZ));
			graphCtrller->ballGr->setAcceleration(Vector(baX, baY, baZ));
		}
	}
	else if ((bpX <= SHIFT_X - safe_region) && (bpX_next <= SHIFT_X - safe_region))
	{

		nonCollidingBallPos[0] = SHIFT_X - safe_region;
		graphCtrller->ballGr->ball->setCollision(true);


		if (bvX < 0)
		{
			bvX = 0.001;
			baX = 0.001;
			graphCtrller->ballGr->setVelocity(Vector(bvX, bvY, bvZ));
			graphCtrller->ballGr->setAcceleration(Vector(baX, baY, baZ));
		}
	}

	if ((bpZ >= (SHIFT_Z + safe_region)) && (bpZ_next >= (SHIFT_Z + safe_region)))
	{
		nonCollidingBallPos[2] = SHIFT_Z + safe_region;
		graphCtrller->ballGr->ball->setCollision(true);

		if (bvZ > 0)
		{
			bvZ = -0.001;		baZ = -0.001;
			graphCtrller->ballGr->setVelocity(Vector(bvX, bvY, bvZ));
			graphCtrller->ballGr->setAcceleration(Vector(baX, baY, baZ));
		}
	}
	else if ((bpZ <= SHIFT_Z - safe_region) && (bpZ_next <= SHIFT_Z - safe_region))
	{

		nonCollidingBallPos[2] = SHIFT_Z - safe_region;
		graphCtrller->ballGr->ball->setCollision(true);


		if (bvZ < 0)
		{
			bvZ = 0.001;		baZ = 0.001;
			graphCtrller->ballGr->setVelocity(Vector(bvX, bvY, bvZ));
			graphCtrller->ballGr->setAcceleration(Vector(baX, baY, baZ));
		}
	}
	return nonCollidingBallPos;
}







// Checks for collisions with boundaries 
Vector boundaryColl(float angle, Vector ballPosNext)
{
	Vector nonCollidingBallPos;

	float bpX, bpY, bpZ; // current ball position 
	float bvX, bvY, bvZ; // current ball velocity 
	float baX, baY, baZ; // current ball acceleration 

	float bpX_next, bpY_next, bpZ_next; // next ball position 


	float Bpnts[48][3]; // points on ball (object) edges used for collision detection 
	// 8 on short edge, 16 on long edge 
	float bnow[48][3];  // next points on ball edges
	float lenArray[48][3]; // distance to object center 

	int r1 = 0, r2 = 0, r3 = 0, r4 = 0; // number of points penetrating into boundary regions 
	/*            ____r4____
	r1 ||__________||r2
	r3
	*/
	int collPtId_z = 0;	// last colliding point id in z-axis 
	int collisionDir_z = NO_COLLISION; // the direction of collision in z-axis

	int collPtId_x = 0;
	int collisionDir_x = NO_COLLISION;

	graphCtrller->ballGr->getPosition().getValue(bpX, bpY, bpZ);
	graphCtrller->ballGr->getVelocity().getValue(bvX, bvY, bvZ);
	graphCtrller->ballGr->getAcceleration().getValue(baX, baY, baZ);
	graphCtrller->cip.getPosition().getValue(cpX, cpY, cpZ);


	bpX_next = ballPosNext[0];
	bpY_next = ballPosNext[1];
	bpZ_next = ballPosNext[2];



	createLen(lenArray); // initialize the length array

	for (int j = 0; j < 48; j++)
	{
		Bpnts[j][0] = bpX_next + (lenArray[j][0] * cos(angle)) + (lenArray[j][2] * sin(angle));
		Bpnts[j][2] = bpZ_next + (lenArray[j][2] * cos(angle)) - (lenArray[j][0] * sin(angle));
		bnow[j][0] = bpX + (lenArray[j][0] * cos(angle)) + (lenArray[j][2] * sin(angle));
		bnow[j][2] = bpZ + (lenArray[j][2] * cos(angle)) - (lenArray[j][0] * sin(angle));
	}


	for (int j = 0; j < 48; j++) {
		if ((Bpnts[j][0] >= (SHIFT_X + boundary[0] - boundaryThickness)) && (bnow[j][0] >= (SHIFT_X + boundary[0] - boundaryThickness)))
		{
			r2++;
			collisionDir_x = COLLISION_RIGHT;
			graphCtrller->ballGr->ball->setCollision(true);
			collPtId_x = j;

			if (bvX > 0)
			{
				bvX = -0.001;		baX = -0.001;
				graphCtrller->ballGr->setVelocity(Vector(bvX, bvY, bvZ));
				graphCtrller->ballGr->setAcceleration(Vector(baX, baY, baZ));
			}
		}
		else if ((Bpnts[j][0] <= SHIFT_X - (boundary[0] - boundaryThickness)) && (bnow[j][0] <= SHIFT_X - (boundary[0] - boundaryThickness)))
		{
			r1++;
			collisionDir_x = COLLISION_LEFT;
			graphCtrller->ballGr->ball->setCollision(true);
			collPtId_x = j;

			if (bvX < 0)
			{
				bvX = 0.001;
				baX = 0.001;
				graphCtrller->ballGr->setVelocity(Vector(bvX, bvY, bvZ));
				graphCtrller->ballGr->setAcceleration(Vector(baX, baY, baZ));
			}
		}

		if ((Bpnts[j][2] >= (SHIFT_Z + boundary[2] - boundaryThickness)) && (bnow[j][2] >= (SHIFT_Z + boundary[2] - boundaryThickness)))
		{
			r3++;
			collisionDir_z = COLLISION_DOWN;
			graphCtrller->ballGr->ball->setCollision(true);
			collPtId_z = j;

			if (bvZ > 0)
			{
				bvZ = -0.001;		baZ = -0.001;
				graphCtrller->ballGr->setVelocity(Vector(bvX, bvY, bvZ));
				graphCtrller->ballGr->setAcceleration(Vector(baX, baY, baZ));
			}
		}
		else if ((Bpnts[j][2] <= SHIFT_Z - (boundary[2] - boundaryThickness)) && (bnow[j][2] <= SHIFT_Z - (boundary[2] - boundaryThickness)))
		{
			r4++;
			collisionDir_z = COLLISION_UP;
			graphCtrller->ballGr->ball->setCollision(true);
			collPtId_z = j;

			if (bvZ < 0)
			{
				bvZ = 0.001;		baZ = 0.001;
				graphCtrller->ballGr->setVelocity(Vector(bvX, bvY, bvZ));
				graphCtrller->ballGr->setAcceleration(Vector(baX, baY, baZ));
			}
		}

	}

	// handle recurring collisions
	for (int j = 0; j < 48; j++)
	{
		int i = j;

		if ((collisionDir_x == COLLISION_RIGHT) && (collPtId_x == j) && (graphCtrller->ballGr->ball->isCollide()))
		{
			nonCollidingBallPos[0] = (SHIFT_X + boundary[0] - boundaryThickness) -
				((lenArray[i][0] * cos(angle)) + (lenArray[i][2] * sin(angle)));
		}
		else if ((collisionDir_x == COLLISION_LEFT) && (collPtId_x == j) && (graphCtrller->ballGr->ball->isCollide()))
		{
			nonCollidingBallPos[0] = SHIFT_X - (boundary[0] - boundaryThickness) -
				((lenArray[i][0] * cos(angle)) + (lenArray[i][2] * sin(angle)));
		}

		if ((collisionDir_z == COLLISION_DOWN) && (collPtId_z == i) && (graphCtrller->ballGr->ball->isCollide()))
		{
			nonCollidingBallPos[2] = SHIFT_Z + boundary[2] - boundaryThickness -
				((lenArray[i][2] * cos(angle)) - (lenArray[i][0] * sin(angle)));
		}
		else if ((collisionDir_z == COLLISION_UP) && (collPtId_z == i) && (graphCtrller->ballGr->ball->isCollide()))
		{
			nonCollidingBallPos[2] = SHIFT_Z - (boundary[2] - boundaryThickness) -
				((lenArray[i][2] * cos(angle)) - (lenArray[i][0] * sin(angle)));
		}

		if ((!graphCtrller->ballGr->ball->isCollide()) && (collPtId_z == 0) && (collisionDir_z == NO_COLLISION) &&
			(collPtId_x == 0) && (collisionDir_x == NO_COLLISION))
		{
			nonCollidingBallPos[0] = bpX;
			nonCollidingBallPos[2] = bpZ;
		}
	}


	nonCollidingBallPos[1] = bpY;

	if (collPtId_x == 0)
	{
		nonCollidingBallPos[0] = bpX;
	}

	if (collPtId_z == 0)
	{
		nonCollidingBallPos[2] = bpZ;
	}



	if (graphCtrller->ballGr->ball->isCollide())
	{
		errorTime++;
		setforces(r1, r2, r3, r4, 1);
		crunsb++;
		graphCtrller->boardGr->setHit(true);
		graphCtrller->ballGr->ball->setCollision(true);
		giveWarningB = 1;
	}
	else
	{
		boundaryforceCX = 0.0f;
		boundaryforceCZ = 0.0f;
		boundaryforceHX = 0.0f;
		boundaryforceHZ = 0.0f;

		graphCtrller->boardGr->setHit(false);
		graphCtrller->ballGr->ball->setCollision(false);
		giveWarningB = 0;

		crunsb = 0;
	}

	return nonCollidingBallPos;
}

Vector calculateFriction()
{
	int repeat = 50000;
	f_forceY = 0.0f;
	float massBall = graphCtrller->ballGr->ball->getMass();
	float ballVelX = graphCtrller->ballGr->ball->velX;
	float ballVelZ = graphCtrller->ballGr->ball->velZ;

	if (runs % repeat == 0)
	{
		if (abs(ballVelX) > 0.0f)
		{
			frictionX[0] = -(massBall * G * COEFFICIENT_FRICTION_KINETIC * SSIGN(ballVelX)) // static friction
				- (COEFFICIENT_FRICTION_DYNAMIC *  ballVelX);
			//graphCtrller->ballGr->getMass()*gravitation*friction*1*sign(ball->velX);
		}

		if (abs(ballVelZ) > 0.0f)
		{
			frictionZ[0] = -(massBall * G * COEFFICIENT_FRICTION_KINETIC * SSIGN(ballVelZ)) // static friction
				- (COEFFICIENT_FRICTION_DYNAMIC *  ballVelZ);
		}
	}
	else if (runs % repeat == (repeat - 1))
	{
		if (abs(ballVelX) > 0.0f)
		{
			frictionX[1] = -(massBall * G * COEFFICIENT_FRICTION_KINETIC * SSIGN(ballVelX)) // static friction
				- (COEFFICIENT_FRICTION_DYNAMIC * ballVelX);
			//graphCtrller->ballGr->getMass()*gravitation*friction*1*sign(ball->velX);
		}

		if (abs(ballVelZ) > 0.0f)
		{
			frictionZ[1] = -(massBall * G * COEFFICIENT_FRICTION_KINETIC * SSIGN(ballVelZ)) // static friction
				- (COEFFICIENT_FRICTION_DYNAMIC *  ballVelZ);
		}
	}

	float diffrenceX = frictionX[1] - frictionX[0];

	f_forceX = frictionX[0] + (diffrenceX / repeat * fmod(runs, (float)repeat));

	float diffrenceZ = frictionZ[1] - frictionZ[0];

	f_forceZ = frictionZ[0] + (diffrenceZ / repeat * fmod(runs, (float)repeat));

	return Vector(f_forceX, f_forceY, f_forceZ);


}
void createLen(float lenArray[48][3])
{
	float beta = atan2(BALL_DEPTH, BALL_WIDTH);
	float teta = atan2(BALL_DEPTH*0.25, BALL_WIDTH*0.5);
	float h2 = sqrt((BALL_DEPTH*0.25)*(BALL_DEPTH*0.25) + (BALL_WIDTH*0.5)*(BALL_WIDTH*0.5));
	float h = sqrt(BALL_WIDTH*BALL_WIDTH + BALL_DEPTH*BALL_DEPTH)*0.5;

	lenArray[0][0] = -h*cos(beta);
	lenArray[0][2] = -h*sin(beta);

	lenArray[1][0] = -h*cos(beta);
	lenArray[1][2] = h*sin(beta);

	lenArray[2][0] = h*cos(beta);
	lenArray[2][2] = h*sin(beta);

	lenArray[3][0] = h*cos(beta);
	lenArray[3][2] = -h*sin(beta);

	lenArray[4][0] = 0.0f;
	lenArray[4][2] = -BALL_DEPTH*0.5;

	lenArray[5][0] = 0;
	lenArray[5][2] = BALL_DEPTH*0.5;

	lenArray[6][0] = -BALL_WIDTH*0.5;
	lenArray[6][2] = 0.0f;

	lenArray[7][0] = BALL_WIDTH*0.5;
	lenArray[7][2] = 0.0f;

	lenArray[8][0] = h2*cos(teta);
	lenArray[8][2] = h2*sin(teta);

	lenArray[9][0] = -h2*cos(teta);
	lenArray[9][2] = h2*sin(teta);

	lenArray[10][0] = -h2*cos(teta);
	lenArray[10][2] = -h2*sin(teta);

	lenArray[11][0] = -h2*cos(teta);
	lenArray[11][2] = h2*sin(teta);

	lenArray[12][0] = BALL_WIDTH*0.25;
	lenArray[12][2] = -BALL_DEPTH / 2;

	lenArray[13][0] = BALL_WIDTH*0.25;
	lenArray[13][2] = BALL_DEPTH / 2;

	lenArray[14][0] = -BALL_WIDTH*0.25;
	lenArray[14][2] = -BALL_DEPTH / 2;

	lenArray[15][0] = -BALL_WIDTH*0.25;
	lenArray[15][2] = (BALL_DEPTH / 2);

	lenArray[16][0] = BALL_WIDTH / 8;
	lenArray[16][2] = (-BALL_DEPTH / 2);

	lenArray[17][0] = -BALL_WIDTH / 8;
	lenArray[17][2] = (-BALL_DEPTH / 2);

	lenArray[18][0] = -BALL_WIDTH / 8;
	lenArray[18][2] = (BALL_DEPTH / 2);

	lenArray[19][0] = BALL_WIDTH / 8;
	lenArray[19][2] = (BALL_DEPTH / 2);

	lenArray[20][0] = BALL_WIDTH / 8 * 3;
	lenArray[20][2] = -BALL_DEPTH*0.5;

	lenArray[21][0] = -BALL_WIDTH / 8 * 3;
	lenArray[21][2] = -BALL_DEPTH*0.5;

	lenArray[22][0] = -BALL_WIDTH / 8 * 3;
	lenArray[22][2] = BALL_DEPTH*0.5;

	lenArray[23][0] = BALL_WIDTH / 8 * 3;
	lenArray[23][2] = BALL_DEPTH*0.5;

	lenArray[24][0] = BALL_WIDTH / 16;
	lenArray[24][2] = -BALL_DEPTH / 2;

	lenArray[25][0] = BALL_WIDTH / 16;
	lenArray[25][2] = BALL_DEPTH / 2;

	lenArray[26][0] = -BALL_WIDTH / 16;
	lenArray[26][2] = -BALL_DEPTH / 2;

	lenArray[27][0] = -BALL_WIDTH / 16;
	lenArray[27][2] = BALL_DEPTH / 2;

	lenArray[28][0] = BALL_WIDTH / 16 * 3;
	lenArray[28][2] = -BALL_DEPTH / 2;

	lenArray[29][0] = BALL_WIDTH / 16 * 3;
	lenArray[29][2] = BALL_DEPTH / 2;

	lenArray[30][0] = -BALL_WIDTH / 16 * 3;
	lenArray[30][2] = -BALL_DEPTH / 2;

	lenArray[31][0] = -BALL_WIDTH / 16 * 3;
	lenArray[31][2] = BALL_DEPTH / 2;

	lenArray[32][0] = BALL_WIDTH / 16 * 5;
	lenArray[32][2] = -BALL_DEPTH / 2;

	lenArray[33][0] = BALL_WIDTH / 16 * 5;
	lenArray[33][2] = BALL_DEPTH / 2;

	lenArray[34][0] = -BALL_WIDTH / 16 * 5;
	lenArray[34][2] = -BALL_DEPTH / 2;

	lenArray[35][0] = -BALL_WIDTH / 16 * 5;
	lenArray[35][2] = BALL_DEPTH / 2;

	lenArray[36][0] = BALL_WIDTH / 16 * 7;
	lenArray[36][2] = -BALL_DEPTH / 2;

	lenArray[37][0] = BALL_WIDTH / 16 * 7;
	lenArray[37][2] = BALL_DEPTH / 2;

	lenArray[38][0] = -BALL_WIDTH / 16 * 7;
	lenArray[38][2] = -BALL_DEPTH / 2;

	lenArray[39][0] = -BALL_WIDTH / 16 * 7;
	lenArray[39][2] = BALL_DEPTH / 2;

	lenArray[40][0] = -BALL_WIDTH / 2;
	lenArray[40][2] = -BALL_DEPTH / 8;

	lenArray[41][0] = BALL_WIDTH / 2;
	lenArray[41][2] = -BALL_DEPTH / 8;

	lenArray[42][0] = -BALL_WIDTH / 2;
	lenArray[42][2] = BALL_DEPTH / 8;

	lenArray[43][0] = -BALL_WIDTH / 2;
	lenArray[43][2] = BALL_DEPTH / 8;

	lenArray[44][0] = -BALL_WIDTH / 2;
	lenArray[44][2] = -BALL_DEPTH / 8 * 3;

	lenArray[45][0] = BALL_WIDTH / 2;
	lenArray[45][2] = -BALL_DEPTH / 8 * 3;

	lenArray[46][0] = -BALL_WIDTH / 2;
	lenArray[46][2] = BALL_DEPTH / 8 * 3;

	lenArray[47][0] = BALL_WIDTH / 2;
	lenArray[47][2] = BALL_DEPTH / 8 * 3;

}
Vector initialC(int type)
{
	Vector firstPos, position;
	Vector cur_pos, inc;

	if (type == 1)
	{
		graphCtrller->hip.getPosition().getValue(position[0], position[1], position[2]);
		firstPos = firstPosHip;
		totErrC = totErrorHip;
		initialPos[0] = initialPosH[0];
		initialPos[1] = initialPosH[1];
		initialPos[2] = initialPosH[2];
	}
	else if (type == 2)
	{
		firstPos = firstPosCip;
		graphCtrller->cip.getPosition().getValue(position[0], position[1], position[2]);
		totErrC = totErrorCip;
		initialPos[0] = initialPosC[0];
		initialPos[1] = initialPosC[1];
		initialPos[2] = initialPosC[2];
	}

	if (runs == 2)
	{
		for (int i = 0; i < 3; i++)
		{
			firstPos[i] = position[i];
		}

	}
	else if (runs < timeToGoInit + 1)
	{
		for (int i = 0; i < 3; i++) {
			inc[i] = (initialPos[i] - firstPos[i]) / (timeToGoInit);
			cur_pos[i] = firstPos[i] + inc[i] * (runs - 2);
			errorC[i] = cur_pos[i] - position[i];
			totErrC[i] = totErrC[i] + errorC[i];
			forceC[i] = kp * errorC[i] + ki * totErrC[i];
		}
	}
	else if (runs < timeToGoInit + wait)
	{

		for (int i = 0; i < 3; i++) {
			cur_pos[i] = initialPos[i];
			errorC[i] = cur_pos[i] - position[i];
			totErrC[i] = totErrC[i] + errorC[i];
			forceC[i] = kp * errorC[i] + ki * totErrC[i];
		}
		initialPosAcquired = 1;
	}

	forceC[1] = 0;

	return forceC;
}




void serverLoop(void * arg)
{
	while (true)
	{
		server->update();

		if (server->clientConnected() && runs == 0)
		{
			// initialize phantom servo loop 
			if (effect->keepHapticLoop == false)
			{
				HDErrorInfo error;
				effect->keepHapticLoop = true;

				effect->initialize_phantom(effect->keepHapticLoop);
				printf("Game started!\n");

				// begin frame at the beginning of each servo loop
				callbackHandlers.push_back(hdScheduleAsynchronous(BeginFrameCallback, (void*)0, HD_MAX_SCHEDULER_PRIORITY));
				// end frame at the end of each servo loop
				callbackHandlers.push_back(hdScheduleAsynchronous(EndFrameCallback, (void*)0, HD_MIN_SCHEDULER_PRIORITY));

				//callbackHandlers.push_back( hdScheduleAsynchronous(ComputeForceCallback, computedForces, HD_DEFAULT_SCHEDULER_PRIORITY) );
				callbackHandlers.push_back(hdScheduleAsynchronous(MyHapticLoop, (void*)0, HD_DEFAULT_SCHEDULER_PRIORITY));
				//callbackHandlers.push_back( hdScheduleAsynchronous(DutyCycleCallback, (void*)0, HD_MIN_SCHEDULER_PRIORITY) );

				hdSetSchedulerRate(SCHEDULER_RATE);
				hdStartScheduler();


				SoundPlayerThread::run();

				if (HD_DEVICE_ERROR(error = hdGetError()))
				{
					//hduPrintError(stderr, &error, "Failed to start the scheduler");
					printf("Failed to start the scheduler");
				}

			}
			return;
		}
	}
}

//Vector xwallColl_full(float wallx1,float wallx2,float wallz1,float wallz2,float wallx1f,float wallx2f,float wallz1f,float wallz2f,Vector ballPosNext,float angle,float wWidth)
//{	
//	Vector nonCollidingBallPos;
//	float bpX, bpY, bpZ;// current ball position
//	float bvX,bvY,bvZ; //current ball velocity
//	float baX,baY,baZ; //current ball acceleration
//
//	float bppX, bppY, bppZ; //next ball position
//
//
//	float Bpnts[48][3]; //coordinates of next points at ball (48 points)
//	float bnow[48][3];	//coordinates of current points at ball
//
//	float wallx11=wallx1+0.5*wWidth; // maximum position of the wall in x axis
//	float wallx12=wallx1-0.5*wWidth;	// minimum position of the wall in x axis
//
//	float wallx11f = wallx1f+0.5*wWidth; // maximum position of the wall in x axis
//	float wallx12f = wallx1f-0.5*wWidth;	// minimum position of the wall in x axis
//
//
//	graphCtrller->ballGr->getPosition().getValue(bpX, bpY, bpZ); //cigil bppx
//
//	bppX=ballPosNext[0];
//	bppY=ballPosNext[1];
//	bppZ=ballPosNext[2];
//
//	graphCtrller->ballGr->getVelocity().getValue(bvX,bvY,bvZ);
//	graphCtrller->ballGr->getAcceleration().getValue(baX,baY,baZ);
//
//	float lenArray[48][3];
//	createLen(lenArray);
//
//
//	for(int j=0;j<48;j++)
//	{
//		Bpnts[j][0]=bppX+(lenArray[j][0]*cos(angle))+(lenArray[j][2]*sin(angle));//next
//		Bpnts[j][2]=bppZ+(lenArray[j][2]*cos(angle))-(lenArray[j][0]*sin(angle));
//		bnow[j][0]=bpX+(lenArray[j][0]*cos(angle))+(lenArray[j][2]*sin(angle));
//		bnow[j][2]=bpZ+(lenArray[j][2]*cos(angle))-(lenArray[j][0]*sin(angle));
//	}
//
//	int collPtId_x = 0;	// last colliding point id in x-axis 
//	int collPtId_z = 0;	// last colliding point id in z-axis 
//
//	graphCtrller->ballGr->ball->setCollision(false);
//
//
//	//coefficients of line1 and line2 
//	//for example line1 is a1*x+b1*z=c1
//	float a1 = wallz1-wallz2;
//	float b1 = - (wallx12-wallx11);
//	float c1 = wallx11*a1 + wallz2*b1;
//	float a2 = - (wallz1 - wallz2);
//	float b2 = - (wallx12-wallx11);
//	float c2 = wallx11 * a2 + wallz1 *b2;
//
//	float a1f = wallz1f-wallz2f;
//	float b1f = - (wallx12f-wallx11f);
//	float c1f = wallx11*a1f + wallz2*b1f;
//	float a2f = - (wallz1f - wallz2f);
//	float b2f = - (wallx12f-wallx11f);
//	float c2f = wallx11f * a2f + wallz1f *b2f;
//
//
//
//	for (int i=0;i<48;i++){
//		if((Bpnts[i][0]<(boundary[0]+SHIFT_X-boundaryThickness))&& (Bpnts[i][0]> SHIFT_X-(boundary[0]-boundaryThickness)))
//		{
//			if( (Bpnts[i][0] <= wallx1) && (Bpnts[i][0] >= (wallx1-0.5*wWidth)) && (Bpnts[i][2] <= wallz1) && (Bpnts[i][2] >= wallz2) )
//			{  
//				graphCtrller->ballGr->ball->setCollision(true);
//				collPtId_x=i;
//
//			}
//
//		}
//	}
//	int collisionDir_x =  NO_COLLISION;
//	int collisionDir_z =  NO_COLLISION;
//
//	//// number of points penetrating into boundary regions 
//	int region1 = 0, region2 = 0, region3 = 0, region4 = 0;
//	int region5 = 0, region6 = 0, region7 = 0, region8 = 0;
//
//	/*        ___region3__
//	|			  |
//	|			  |
//	region1		region2
//	|			  |
//	|			  |
//	|__region4___|		 
//	*/
//
//	for (int i=0; i<48; i++){
//		if ( (Bpnts[i][0] < (SHIFT_X+boundary[0]-boundaryThickness)) && (Bpnts[i][0]> SHIFT_X-(boundary[0]-boundaryThickness)) )
//		{
//			if ( (Bpnts[i][0] <= wallx1+0.5*wWidth) && (Bpnts[i][0]>=wallx1-0.5*wWidth) && (Bpnts[i][2]<=wallz1) && (Bpnts[i][2] >= wallz2) )
//			{ 
//				if ( (bnow[i][0] <= wallx1+0.5*wWidth) && (bnow[i][0] >= wallx1-0.5*wWidth) && (bnow[i][2]<=wallz1) && (bnow[i][2] >= wallz2) )
//				{
//
//					//collision = 1;   CIGIL
//					graphCtrller->ballGr->ball->setCollision(true);
//
//					float future_x = Bpnts[i][0];//future
//					float future_z = Bpnts[i][2];//future
//
//					float current_x = bnow[i][0]; //now
//					float current_z = bnow[i][2];//now
//
//
//					if( ( ((a1*future_x)+(b1*future_z)) < c1 ) && 
//						( ((a2*future_x)+(b2*future_z)) > c2 ) &&
//						( ((a1*current_x)+(b1*current_z)) < c1 ) && 
//						( ((a2*current_x)+(b2*current_z)) > c2 ))
//					{   
//						region1++;
//						//collision = 1;
//						graphCtrller->ballGr->ball->setCollision(true);
//
//						collPtId_x = i;
//
//					} 
//					else if( ( ((a1*future_x)+(b1*future_z)) > c1 )&& ( ((a2*future_x)+(b2*future_z)) < c2 ) && ( ((a1*current_x)+(b1*current_z)) > c1 )&& ( ((a2*current_x)+(b2*current_z)) < c2 ))
//					{   
//						region2++;
//						graphCtrller->ballGr->ball->setCollision(true);
//
//						collPtId_x=i;
//
//					} 
//					else if( ( ((a1*future_x)+(b1*future_z)) > c1 )&& ( ((a2*future_x)+(b2*future_z)) > c2 ) &&( ((a1*current_x)+(b1*current_z)) > c1 )&& ( ((a2*current_x)+(b2*current_z)) > c2 ))
//					{   
//						region3++;
//						graphCtrller->ballGr->ball->setCollision(true);
//						collPtId_z=i;
//					} 
//					else if( ( ((a1*future_x)+(b1*future_z)) < c1 )&& ( ((a2*future_x)+(b2*future_z)) < c2 )&&( ((a1*current_x)+(b1*current_z)) < c1 )&& ( ((a2*current_x)+(b2*current_z)) < c2 )  )
//					{  
//						region4++;
//						graphCtrller->ballGr->ball->setCollision(true);
//						collPtId_z=i;
//					} 
//
//					else if ( (((a1*future_x)+(b1*future_z)) < c1 )&& ( ((a2*future_x)+(b2*future_z)) == c2 )&&(((a1*current_x)+(b1*current_z)) <c1 )&& ( ((a2*current_x)+(b2*current_z)) == c2 ) )
//					{  
//						region1++; region4++;
//						graphCtrller->ballGr->ball->setCollision(true);
//						collPtId_x=i;collPtId_z=i;
//					}
//					else if ( (((a1*future_x)+(b1*future_z)) > c1 )&& ( ((a2*future_x)+(b2*future_z)) == c2 )&& (((a1*current_x)+(b1*current_z)) > c1 )&& ( ((a2*current_x)+(b2*current_z)) == c2 ) )
//					{
//						region3++; region2++;
//						graphCtrller->ballGr->ball->setCollision(true);
//						collPtId_x=i;collPtId_z=i;
//					}
//					else if(( ((a1*future_x)+(b1*future_z)) == c1 )&& ( ((a2*future_x)+(b2*future_z)) > c2 )&&( ((a1*current_x)+(b1*current_z)) == c1 )&& ( ((a2*current_x)+(b2*current_z)) > c2 ) )
//					{  
//						region1++; 
//						region3++;
//						graphCtrller->ballGr->ball->setCollision(true);
//						collPtId_x=i;collPtId_z=i;
//					}
//					else if ( (((a1*future_x)+(b1*future_z)) == c1 )&& ( ((a2*future_x)+(b2*future_z)) < c2 )&&(((a1*current_x)+(b1*current_z)) == c1 )&& ( ((a2*current_x)+(b2*current_z)) < c2 ) )
//					{  
//						region2++; 
//						region4++;
//						graphCtrller->ballGr->ball->setCollision(true);
//						collPtId_x=i;collPtId_z=i;
//
//					}
//
//
//					if( ( ((a1f*future_x)+(b1f*future_z)) < c1f ) && 
//						( ((a2f*future_x)+(b2f*future_z)) > c2f ) &&
//						( ((a1f*current_x)+(b1f*current_z)) < c1f ) && 
//						( ((a2f*current_x)+(b2f*current_z)) > c2f ))
//					{   
//						region5++;
//						//collision = 1;
//						graphCtrller->ballGr->ball->setCollision(true);
//
//						collPtId_x = i;
//
//					} 
//					else if( ( ((a1f*future_x)+(b1f*future_z)) > c1f )&& ( ((a2f*future_x)+(b2f*future_z)) < c2f ) && ( ((a1f*current_x)+(b1f*current_z)) > c1f )&& ( ((a2f*current_x)+(b2f*current_z)) < c2f ))
//					{   
//						region6++;
//						graphCtrller->ballGr->ball->setCollision(true);
//
//						collPtId_x=i;
//
//					} 
//					else if( ( ((a1f*future_x)+(b1f*future_z)) > c1f )&& ( ((a2f*future_x)+(b2f*future_z)) > c2f ) &&( ((a1f*current_x)+(b1f*current_z)) > c1f )&& ( ((a2f*current_x)+(b2f*current_z)) > c2f ))
//					{   
//						region7++;
//						graphCtrller->ballGr->ball->setCollision(true);
//						collPtId_z=i;
//					} 
//					else if( ( ((a1f*future_x)+(b1f*future_z)) < c1f )&& ( ((a2f*future_x)+(b2f*future_z)) < c2f )&&( ((a1f*current_x)+(b1f*current_z)) < c1f )&& ( ((a2f*current_x)+(b2f*current_z)) < c2f )  )
//					{  
//						region8++;
//						graphCtrller->ballGr->ball->setCollision(true);
//						collPtId_z=i;
//					} 
//
//					else if ( (((a1f*future_x)+(b1f*future_z)) < c1f )&& ( ((a2f*future_x)+(b2f*future_z)) == c2f )&&(((a1f*current_x)+(b1f*current_z)) <c1f )&& ( ((a2f*current_x)+(b2f*current_z)) == c2f ) )
//					{  
//						region5++; region8++;
//						graphCtrller->ballGr->ball->setCollision(true);
//						collPtId_x=i;collPtId_z=i;
//					}
//					else if ( (((a1f*future_x)+(b1f*future_z)) > c1f )&& ( ((a2f*future_x)+(b2f*future_z)) == c2f )&& (((a1f*current_x)+(b1f*current_z)) > c1f )&& ( ((a2f*current_x)+(b2f*current_z)) == c2f ) )
//					{
//						region7++; region6++;
//						graphCtrller->ballGr->ball->setCollision(true);
//						collPtId_x=i;collPtId_z=i;
//					}
//					else if(( ((a1f*future_x)+(b1f*future_z)) == c1f )&& ( ((a2f*future_x)+(b2f*future_z)) > c2f )&&( ((a1f*current_x)+(b1f*current_z)) == c1f )&& ( ((a2f*current_x)+(b2f*current_z)) > c2f ) )
//					{  
//						region5++; 
//						region7++;
//						graphCtrller->ballGr->ball->setCollision(true);
//						collPtId_x=i;collPtId_z=i;
//					}
//					else if ( (((a1f*future_x)+(b1f*future_z)) == c1f )&& ( ((a2f*future_x)+(b2f*future_z)) < c2f )&&(((a1f*current_x)+(b1f*current_z)) == c1f )&& ( ((a2f*current_x)+(b2f*current_z)) < c2f) )
//					{  
//						region6++; 
//						region8++;
//						graphCtrller->ballGr->ball->setCollision(true);
//						collPtId_x=i;collPtId_z=i;
//
//					}
//
//					else if ( (((a1*future_x)+(b1*future_z)) == c1 )&& ( ((a2*future_x)+(b2*future_z)) < c2 )&&(((a1*current_x)+(b1*current_z)) == c1 )&& ( ((a2*current_x)+(b2*current_z)) == c2 ) )
//					{
//						region1++;
//						region2++;
//						region4++;
//						region3++;
//						graphCtrller->ballGr->ball->setCollision(true);
//						graphCtrller->ballGr->ball->setCollision(true);
//						collPtId_x=i; collPtId_z=i;
//					}
//
//
//				}
//			}
//		}
//	}
//	if(graphCtrller->ballGr->ball->isCollide())
//	{
//		//since it is wall collision forcetypes=2
//		crunsw++;
//		setforces(region1,region2,region3,region4,2);//18 eylul
//	}
//	else
//	{
//		fhw[0]=0.0f;
//		fcw[0]=0.0f;
//		fhw[2]=0.0f;
//		fcw[2]=0.0f;
//
//		/*	fhw_arr[0][0]=0.0f;
//		fcw_arr[0][0]=0.0f;
//		fhw_arr[0][2]=0.0f;
//		fcw_arr[0][2]=0.0f;
//
//		fhw_arr[1][0]=0.0f;
//		fcw_arr[1][0]=0.0f;
//		fhw_arr[1][2]=0.0f;
//		fcw_arr[1][2]=0.0f;*/
//
//		graphCtrller->wall1->setHit(false);
//		graphCtrller->boardGr->setHitObstacle(false);
//		giveWarningW=0;
//		crunsw=0;
//	}
//	if(dataRec->scenario == MIXED)
//	{
//		if( (graphCtrller->ballGr->ball->isCollide()) && (wall_hit == 1) )
//		{
//			graphCtrller->wall1->setHit(true);
//			giveWarningW = 1;
//		}
//		else if (!graphCtrller->ballGr->ball->isCollide())
//		{
//			graphCtrller->wall1->setHit(false);
//			giveWarningW = 0;
//		}
//	}
//	else if(dataRec->scenario == ROTATIONAL)
//	{
//
//		if( (graphCtrller->ballGr->ball->isCollide()))
//		{	
//			giveWarningW = 1;
//			graphCtrller->boardGr->setHitObstacle(true);
//		}
//		else if (!graphCtrller->ballGr->ball->isCollide())
//		{
//			graphCtrller->boardGr->setHitObstacle(false);
//			giveWarningW=0;
//		}
//
//	}
//	//else if ( (graphCtrller->ballGr->ball->isCollide()) && (wall_hit==2) )
//	//{
//	//	graphCtrller->wall2->setHit(true);
//	//	giveWarningW=1;
//	//}
//	//else if ( (graphCtrller->ballGr->ball->isCollide()) && (wall_hit == 3) )
//	//{
//	//	graphCtrller->wall3->setHit(true);
//	//	giveWarningW=1;
//	//}
//
//
//	int wallNo=0;
//
//	if( (region1 > 0) && (region1 > region2) )
//	{
//		collisionDir_x = COLLISION_LEFT;  //1
//		wallNo = 1;
//	}
//	else if ( (region2 > 0) && (region2 > region1) )
//	{
//		collisionDir_x = COLLISION_RIGHT;
//		wallNo = 1;
//	}
//
//	if( (region3 > 0) && (region3 > region4) )
//	{
//		collisionDir_z = COLLISION_UP;
//		wallNo = 1;
//	}
//	else if ( (region4 > 0) && (region4 > region3) )
//	{
//		collisionDir_z = COLLISION_DOWN;
//		wallNo = 1;
//	}
//
//	if( (region5 > 0) && (region5 > region6) )
//	{
//		collisionDir_x = COLLISION_LEFT;  //1
//		wallNo = 2;
//	}
//	else if ( (region6 > 0) && (region6 > region5) )
//	{
//		collisionDir_x = COLLISION_RIGHT;
//		wallNo = 2;
//	}
//
//	if( (region7 > 0) && (region7 > region8) )
//	{
//		collisionDir_z = COLLISION_UP;
//		wallNo = 2;
//	}
//	else if ( (region8 > 0) && (region8 > region7) )
//	{
//		collisionDir_z = COLLISION_DOWN;
//		wallNo = 2;
//	}
//	if((region4 > 0) && (region7 > 0) )
//	{
//		wallNo = 12;
//	}
//
//	float wallx1_eq, wallz1_eq, wallz2_eq;
//	if(wallNo==1)
//	{
//		wallx1_eq = wallx1;
//		wallz1_eq = wallz1;
//		wallz2_eq = wallz2;
//	}
//	else if(wallNo==2)
//	{
//		wallx1_eq = wallx1f;
//		wallz1_eq = wallz1f;
//		wallz2_eq = wallz2f;
//	}
//
//
//
//for ( int j=0; j < 48; j++ )
//{
//
//	if ( (collisionDir_x == COLLISION_LEFT) && (collPtId_x == j) && (graphCtrller->ballGr->ball->isCollide()) )
//	{
//		nonCollidingBallPos[0]=(wallx1_eq-0.5*wWidth)+((-lenArray[j][0]*cos(angle))+(-lenArray[j][2]*sin(angle)));
//	}
//	else if ( (collisionDir_x == COLLISION_RIGHT) && (collPtId_x == j) && (graphCtrller->ballGr->ball->isCollide()) )
//	{
//		nonCollidingBallPos[0]=wallx1_eq + 0.5*wWidth+((-lenArray[j][0]*cos(angle))+(-lenArray[j][2]*sin(angle)));
//	}
//	else if ( (collisionDir_z == COLLISION_UP) && (collPtId_z == j) && (graphCtrller->ballGr->ball->isCollide()) )
//	{
//		nonCollidingBallPos[2]=(wallz1_eq)+((-lenArray[j][2]*cos(angle))-(-lenArray[j][0]*sin(angle)));
//	}
//	else if ( (collisionDir_z == COLLISION_DOWN) && (collPtId_z == j) && (graphCtrller->ballGr->ball->isCollide()) )
//	{
//		nonCollidingBallPos[2]=(wallz2_eq)+((-lenArray[j][2]*cos(angle))-(-lenArray[j][0]*sin(angle)));
//	}
//	else if( (collisionDir_x == COLLISION_LEFT ) && (collisionDir_z == COLLISION_UP) && (collPtId_x == j) && (collPtId_z == j) && (graphCtrller->ballGr->ball->isCollide()) )
//	{   
//		nonCollidingBallPos[0]=(wallx1_eq-0.5*wWidth)+((-lenArray[j][0]*cos(angle))+(-lenArray[j][2]*sin(angle)));
//		nonCollidingBallPos[2]=(wallz1_eq)+((-lenArray[j][2]*cos(-angle))-(-lenArray[j][0]*sin(-angle)));
//	}
//	else if ( (collisionDir_x == COLLISION_LEFT ) && (collisionDir_z == COLLISION_DOWN) && (collPtId_x == j) && (graphCtrller->ballGr->ball->isCollide()) && (collPtId_z == j) )
//	{   
//		nonCollidingBallPos[0]=(wallx1_eq-0.5*wWidth)+((-lenArray[j][0]*cos(angle))+(-lenArray[j][2]*sin(angle)));
//		nonCollidingBallPos[2]=(wallz2_eq)+((-lenArray[j][2]*cos(angle))-(-lenArray[j][0]*sin(angle)));
//
//	}
//	else if ( (collisionDir_x == COLLISION_RIGHT) && (collisionDir_z == COLLISION_UP) && (collPtId_x == j) && (graphCtrller->ballGr->ball->isCollide())&& (collPtId_z == j) )
//	{   
//		nonCollidingBallPos[0]=(wallx1_eq+0.5*wWidth)+((-lenArray[j][0]*cos(angle))+(-lenArray[j][2]*sin(angle)));
//		nonCollidingBallPos[2]=(wallz1_eq)+((-lenArray[j][2]*cos(angle))-(-lenArray[j][0]*sin(angle)));
//
//	}
//	else if ( (collisionDir_x == COLLISION_RIGHT) && (collisionDir_z == COLLISION_DOWN) && (collPtId_x == j) && (graphCtrller->ballGr->ball->isCollide()) && (collPtId_z == j) )
//	{   
//		nonCollidingBallPos[0]=(wallx1_eq+0.5*wWidth)+((-lenArray[j][0]*cos(angle))+(-lenArray[j][2]*sin(angle)));
//		nonCollidingBallPos[2]=(wallz2_eq)+((-lenArray[j][2]*cos(angle))-(-lenArray[j][0]*sin(angle)));
//	}
//
//	else if ( (!graphCtrller->ballGr->ball->isCollide()) && (collisionDir_x == NO_COLLISION) && (collisionDir_z == NO_COLLISION) && (collPtId_x == 0) && (collPtId_z == 0) )
//	{
//		nonCollidingBallPos[0]=bpX;
//		nonCollidingBallPos[1]=bpY;
//		nonCollidingBallPos[2]=bpZ;
//	}
//}
//int region1_eq, region2_eq,region3_eq,region4_eq;
//if(wallNo==1)
//{
//	region1_eq = region1;
//	region2_eq = region2;
//	region3_eq = region3;
//	region4_eq = region4;
//}
//else if(wallNo == 2)
//
//{
//	region1_eq = region5;
//	region2_eq = region6;
//	region3_eq = region7;
//	region4_eq = region8;
//}
//
//if ((region1_eq != 0)&&(region1_eq > region2_eq))
//{
//	if((bvX > 0)) 
//	{
//		bvX = -0.001f;
//		baX = -0.001f;
//		graphCtrller->ballGr->setVelocity(Vector(bvX,bvY,bvZ));
//		graphCtrller->ballGr->setAcceleration(Vector(baX,baY,baZ));
//	}
//
//}
//
//else if((region2_eq != 0)&&(region2_eq > region1_eq))
//{
//	if((bvX < 0)) 
//	{
//		bvX = 0.001f;
//		baX = 0.001f;
//		graphCtrller->ballGr->setVelocity(Vector(bvX,bvY,bvZ));
//		graphCtrller->ballGr->setAcceleration(Vector(baX,baY,baZ));
//	}
//}
//
//if ((region3_eq != 0)&&(region3_eq > region4_eq))
//{
//	if((bvZ < 0)) 
//	{
//		bvZ = 0.001f;
//		baZ = 0.001f;
//		graphCtrller->ballGr->setVelocity(Vector(bvX,bvY,bvZ));
//		graphCtrller->ballGr->setAcceleration(Vector(baX,baY,baZ));
//	}
//}
//
//else if ((region4_eq != 0)&&(region4_eq > region3_eq))
//{
//	if((bvZ > 0)) 
//	{
//		bvZ = -0.001f;
//		baZ = -0.001f;
//		graphCtrller->ballGr->setVelocity(Vector(bvX,bvY,bvZ));
//		graphCtrller->ballGr->setAcceleration(Vector(baX,baY,baZ));
//	}
//}
//
//
//nonCollidingBallPos[1]=bpY;//ballPosNext[1];
//
//if(collPtId_x == 0)
//{
//	nonCollidingBallPos[0]=bpX;//ballPosNext[0];
//}
//if(collPtId_z == 0)
//{
//	nonCollidingBallPos[2]=bpZ;//ballPosNext[2];
//}
//
//if(wallNo == 12) 
//{
//	nonCollidingBallPos[0]=bpX;//ballPosNext[1];
//	nonCollidingBallPos[1]=bpY;//ballPosNext[1];
//	nonCollidingBallPos[2]=bpZ;//ballPosNext[1];
//	cout<<"wallNo12"<<endl;
//	if (graphCtrller->ballGr->getAngleBallGr()<0)
//	{
//		graphCtrller->ballGr->setAngleBallGr(-63.0f/180*PI);
//		if(graphCtrller->ballGr->getAngVelocity() <0)
//		{
//			graphCtrller->ballGr->setAngVelocity(0.0001f);
//			graphCtrller->ballGr->setAngAcc(0.00010f);
//		}
//	}
//	else if (graphCtrller->ballGr->getAngleBallGr()>0)
//	{
//		graphCtrller->ballGr->setAngleBallGr(63.0f/180*PI);
//		if(graphCtrller->ballGr->getAngVelocity() >0)
//		{
//			graphCtrller->ballGr->setAngVelocity(-0.0001f);
//			graphCtrller->ballGr->setAngAcc(-0.00010f);
//		}
//	}
//}
//
//return nonCollidingBallPos;
//}
