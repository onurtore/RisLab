/**************************************************************************
Developed by: Ozgur Oguz and Ayse Kucukyilmaz 
Purpose: The server loop for haptic.
Date: Jul. 22, 2009
**************************************************************************/

// INCLUDE FILES
#include "Public.h"
#include "MathCB.h"
#include "GraphicsController.h"
#include "HapticCallBack.h"
#include "HapticInterfacePoint.h"
#include "BgTimer.h"
#include <float.h>
#include "DataRecord.h"
// timestamp için eklendi.
#include <ctime>

// EXTERNED VARIABLES
extern HapticCallBack *effect;
extern GraphicsController *graphCtrller;

extern float ballRadius;

// AYSE: rename for clearness 
//float boardHeight;
//float boardWidth;
//float boardDepth;
Vector boundary;
float boundaryThickness;
//float boundaryHeight;

static HDdouble *gServoMotorTemp = 0;
static HDint gNumMotors;
static HDdouble *aMotorTemp = 0;

SbVec3f *computedForces = new SbVec3f(0,0,0);
static HDdouble fFromBall[3] = {0, 0, 0}, fSprFromBtoN[3] = {0, 0, 0},
				fFromNtoC[3] = {0, 0, 0}, fFromNtoH[3] = {0, 0, 0};
float cpX, cpY, cpZ, cvX, cvY, cvZ, caX, caY, caZ;


static volatile HDboolean bRampDownForces = FALSE;
static volatile HDboolean bRampUpForces = FALSE;

extern SbMatrix stylusTransMatrix1;
extern SbMatrix stylusTransMatrix2;
extern float stylusR;
extern float stylusR2;
extern Vector stylusOffset;
extern Vector stylusOffset2;
extern float targetRadius;
extern Point targetCenters				[TARGET_COUNT];
extern Point keyframeGoalCenters		[KEYFRAME_COUNT]; // the keyframe order will change in each game!
extern Point masterKeyframeGoalCenters	[KEYFRAME_COUNT];

// the keyframe order will change in each game!
#ifdef PLAYONCE
int keyframeGoalOrders	[NUM_TRIALS_PER_COND][TARGET_COUNT] = {	{0,1,2,3} };
#else
int keyframeGoalOrders	[NUM_TRIALS_PER_COND][TARGET_COUNT] = {	{0,1,2,3},	
																{1,3,2,0},
																{2,0,1,3}//,
																/*{2,1,0,3},
																{1,2,3,0},
																{2,0,3,1},
																{0,3,1,2},
																*/
																//{1,3,0,2},
																//{3,2,1,0}/*,
																//{3,1,2,0}*/
															  /*{0,1,2,3},	
																{0,1,2,3},
																{0,1,2,3},
																{0,1,2,3},
																{0,1,2,3},
																{0,1,2,3},
																{0,1,2,3},
																{0,1,2,3},
																{0,1,2,3},
																{0,1,2,3}*/ };

#endif

extern bool myStylusFlag;
extern SbMatrix boardRotateMatrixX;
extern SbMatrix boardRotateMatrixZ;
// path visualization vars from graphics.cpp
extern SoCoordinate3	*pathBCoor;
extern SoLineSet		*pathBLines;
extern SoSeparator		*scoreTimeSep;
extern SoTransform		*mySTStrTrans;

// var for path vis
int ballPathPtCnt = 0, ballPathUpdCnt = 0;

// GLOBALS
SbVec3f new_probe_tip;
//SbVec3f new_probe_tip2;

float P[3], P2[3], V[3], V2[3];
bool playSndFlag = false; //AYSE: used for sound playback only on role exchange moments

Vector ctrlForce;
Point desiredBallPos;

/* ozgur - */
static long int elapsed = 0, elapsedTurn, runs = 0, cntF = 0;//The variables in these two lines
static time_t t;	       		// are used to calculate the server rate.
float maxForceFromBoard = 2.0f; // for each axis

/* AYSE - implement score as timing */
static int  scoreTimeHistory [NUM_TRIALS_PER_COND];
static int  scoreTime					= 0;
static int  runsInPits					= 0;
static int  ticksInPits					= 0;
static int  runsOnTargets				= 0;
static int  ticksOnTargets				= 0;
static int  startTickForPits			= 0;
static int  startTickForTargets			= 0;
static bool pitTickCounterStarted[OBSTACLE_COUNT][NUM_OBS_CORNERS-1]	= {false};
static bool targetTickCounterStarted	= false;


float angleX, angleZ, positionX, positionZ, velocityX, velocityZ, accX, accZ;

Vector targetVelocity[TARGET_COUNT];

int contactCyl		[TARGET_COUNT];
// AYSE: add keyframes
int contactKeyframe	[KEYFRAME_COUNT];

Point goalPt, goalPtFraction;
int goalId, prevGoalId; // AYSE: remove, itersSinceHit = 0;

// new model
const int ctrlTimestep = 100;			// 100 ms - for calculating the force to control the ball trajectory
const float o_gravity = 9.81f/10000.0f;	// gravity constant cm/ms^2
float kpBN = 0.5f, kdBN = 0.0015f; //0.5f, kdBN = 0.0015f; //kpBN = 0.1/*0.000000900f*/,	kdBN = 0.86f;/*0.0015500f*///86f;
float kpCT = 0.0000050f,	kdCT = 0.003500f;//kdCT = 0.00199000f;
	

//#ifdef PHANTOM_PREMIUM
	float o_massBall = (SCHEDULER_RATE != 500) ? 0.15f : 0.4f;	// mass of the ball - kg
	float kpLow = 0.05f, kpHigh = 0.6f;// high and low values
	float kpHN = kpHigh;
	float kpCN = kpLow;
//	
//	
//#else
//	float o_massBall = 0.4f;			// mass of the ball - kg
//	float kpLow = 0.05f, kpHigh = 0.5f;// high and low values
//	float kpHN = 0.150f;
//	float kpCN = 0;
//	float kpCNLow = 0.05f, kpCNHigh = kpHN;// high and low values
//#endif

/* kpBN = 0.002; kdBN = 0.22 // nice looking!*/
float o_thetaX, o_thetaZ;				// rotation angle of the board
HapticInterfacePoint *CHIP, *NHIP, *HIP;	// Controlled, Negotiated, and Real Haptic Interface Points
//Ball *ball;
extern DataRecord *dataRec;

const int SMOOTH_FREQ	 = 50;		// higher values make omni complain
const int DOWNSAMPLE_FREQ= 50;
const int HBG_VARIANCE	 = 20;
const int ROLE_INT		 = 200;		// the minimum time (in ms.) that we want the user 
// to feel the force successively above a threshold value
HDdouble fVar[3], fStd[3], fCurAvg[3];		// stores average Force On User, updates itself at each timestep

float	gaussCoef[SMOOTH_FREQ];
float	*dSampF, *dSampFx, *dSampFz;
float	thresholdF_Up[3], thresholdF_Down[3], M2x = 0, M2z = 0;
int		meanVarCnt = 0;
int		consecRolE, consecRolEBack; // AYSE: make role exchange in 2 axes. 
bool	userWantsCtrl = false;
bool	userGivesCtrl = false;
float	blendTime = BLEND_TIME;		// blend in blendTime step
float	blendAlpha = BLEND_TIME;
float	sumFs[3] = {0.0f, 0.0f, 0.0f}, sqrFs[3] = {0.0f, 0.0f, 0.0f};

int		ctrlMode = USER_CTRL;

int insideCyl[TARGET_COUNT];

extern vector <HDSchedulerHandle> callbackHandlers;

void setGoal(int index);
int findNearestUnhitCylinder();
void calcForceFromBoardRot(Point pt, Vector &fBoard);
void copyBallToCHIP();
void putInGroove();
void updatePath();
void setHIPpos();
void calcNHIPpos();

void forceBeWithUser(float plusOrMinus);

void checkRolExBack(HDdouble fX, HDdouble fZ); // AYSE: checks for both axes 
void checkRolExchange(HDdouble fX, HDdouble fZ); // AYSE: checks for both axes 
float InvSqrt (float x);
void calculateThreshold(int);
void computeFOnUserAveg(HDdouble fToD[3]);
void smoothFGauss(int callID);
void smoothFxzGauss(int callID);
void hbgGauss();
void downSampleF();
void downSampleFxz();

extern void setScoreText(int score);
extern void setCounterText(char* counter);
extern void setWarningText(char* warning);

/* AYSE: added binary lights and continuous bars for role exchange */
extern void visualizeRoleExchange();

/* AYSE: added buzz for role exchange and tremor for computer control */
/* Variables for the buzz effect during role exchange 
	and tremor effect when computer is in control of the game */
static HDint buzzFreq					= 90; /* Hz */
static HDdouble buzzAmplitude			= 0.6; /* N */
static const hduVector3Dd buzzDirection	(1,0,1);
static hduVector3Dd buzzForce;

static const HDdouble TREMOR_FREQ_DOWN	= 20;
static const HDdouble TREMOR_FREQ_UP	= 30;
static const HDdouble TREMOR_AMP_DOWN	= 0.025/*0.010*/; 
static const HDdouble TREMOR_AMP_UP		= 0.015/*0.020*/;
/* will be set randomly in a range between TREMOR_FREQ_DOWN - TREMOR_FREQ_UP Hz */
static HDdouble tremorFreq				= TREMOR_FREQ_DOWN; 
/* will be set randomly in a range between  TREMOR_AMP_DOWN - TREMOR_AMP_UP N */
static HDdouble tremorAmplitude			= TREMOR_AMP_DOWN; 
static const hduVector3Dd tremorDirection(1,0,1);
static hduVector3Dd tremorForce; 

HDdouble instRate;


HapticCallBack::HapticCallBack()
{ 
	//hbgGauss();		// calculate gaussian coeffients prior to haptic interaction
	//guidanceFlag = GUIDE_IN_GIVEN_ORDER; // not necessary for the moment
	/*dataRec = new DataRecord();*/

	curTrial = 0;
	fellInPit = false;
	waitOnTarget = false;
	pitfallCount = 0;

	scorePrintedOnScreen = false;
	
	// both styli are guided whence a guidance method is selected
	guidedStyli[BLUE]			= false;
	guidedStyli[GREEN]			= false;
	userControlledStyli[BLUE]	= true;
	userControlledStyli[GREEN]	= true;

	// set kpNC according to the condition
	if ( dataRec->cond == SC )
	{
		kpCN = kpHigh/2.0;
		kpHN = kpHigh/2.0;
	}
	else 
	{
		kpCN = 0;
	}

	// AYSE: initially buzzing and tremor is active, and no sound is played 
	if ( dataRec->cond == VHC )
	{
		buzzOn			= true;
		tremorOn		= true;
	}
	else
	{
		buzzOn			= false;
		tremorOn		= false;
	}

	soundOn			= true;
	tiltOn			= true;
	hipsOn			= false;
	scoreTextOn		= true;

	if ( dataRec->cond == VHC )
	{
		roleIconZoomOn	= true;
	}
	else
	{
		roleIconZoomOn	= false;
	}

	// initially user controls both axes
	separateAxes	= false;
	keepHapticLoop	= false;
	isGameFinished	= true;

	//	ball = new Ball;

	CHIP			= new HapticInterfacePoint;
	NHIP			= new HapticInterfacePoint;
	HIP				= new HapticInterfacePoint;

	fCurAvg[0] = 0;	fCurAvg[1] = 0;	fCurAvg[2] = 0;	
	fVar[0] = 0;	fVar[1] = 0;	fVar[2] = 0;	
	fStd[0] = 0;	fStd[1] = 0;	fStd[2] = 0;	
	thresholdF_Up[0] = 0.5;	thresholdF_Up[1] = 0.5;	thresholdF_Up[2] = 0.5;	
	thresholdF_Down[0] = 0.2;	thresholdF_Down[1] = 0.2;	thresholdF_Down[2] = 0.2;	
	consecRolE = 0;
	consecRolEBack = 0;;

	// *** initialize vars from graphics controller class *** //
	boundary[0]			= graphCtrller->boardGr->boundary[0];
	boundary[1]			= graphCtrller->boardGr->boundary[1];
	boundary[2]			= graphCtrller->boardGr->boundary[2];
	//boundaryHeight		= graphCtrller->boardGr->boundaryHeight;
	boundaryThickness	= graphCtrller->boardGr->boundaryThickness;
	//boardHeight			= graphCtrller->boardGr->floorHeight;
	//boardWidth			= graphCtrller->boardGr->floorWidth;
	//boardDepth			= graphCtrller->boardGr->floorDepth;

	 aMotorTemp = 0;
}

int HapticCallBack::initialize_phantom(bool myboolean)
{      
	// Coordinate system has origin at reset position  //
	// phantom is reset upon entry into program so put //
	// phantom in reset position when starting program //
	// get the phantom up and running //
	HDErrorInfo error;

	HHD hHD = hdInitDevice("Left");
	
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{	
		//hduPrintError(stderr, &error, "Failed to initialize haptic device");
		cout << "Failed to initialize haptic device";
		myboolean = false;
		return(-1);
	}
	/* Query the number of output DOF (i.e. num motors). */
    hdGetIntegerv(HD_OUTPUT_DOF, &gNumMotors);
    aMotorTemp = (HDdouble *) malloc(sizeof(HDdouble) * gNumMotors);
    gServoMotorTemp = (HDdouble *) malloc(sizeof(HDdouble) * gNumMotors);

    hdEnable(HD_FORCE_OUTPUT);
    hdEnable(HD_FORCE_RAMPING);
    hdEnable(HD_MAX_FORCE_CLAMPING);
	hdEnable(HD_SOFTWARE_VELOCITY_LIMIT);
	hdEnable(HD_SOFTWARE_FORCE_IMPULSE_LIMIT);
	
	//hdGetDoublev(HD_NOMINAL_MAX_CONTINUOUS_FORCE, &FORCE_LIMIT);
	hdGetFloatv(HD_NOMINAL_MAX_STIFFNESS, &KP_LIMIT);
	hdGetFloatv(HD_NOMINAL_MAX_DAMPING, &KD_LIMIT);

	cout << "FORCE_LIMIT: " << FORCE_LIMIT << endl
		 << "KP_LIMIT: " << KP_LIMIT << endl
		 << "KD_LIMIT: " << KD_LIMIT << endl;

	if ( dataRec->cond == RE || dataRec->cond == REVHC ) 
	{
		cout << "Current KP: " << 1.0 / ( 1.0 / kpHN + 1.0 / (kpBN + (1.0/(1.0/kpCT	 + 1.0/kpHigh))) ) << endl
			 << "Current KD: " << 1.0 / (1.0/kdBN + 1.0/kdCT) << endl;
	}
	else
	{
		cout << "Current KP: " << 1.0 / ( 1.0 / kpHN + 1.0 / (kpBN + (1.0/(1.0/kpCT	 + 1.0/kpCN))) ) << endl
			 << "Current KD: " << 1.0 / (1.0/kdBN + 1.0/kdCT) << endl;
	}

	cout << endl << "Phantom id: " << (int)hHD << " is initialized." << endl;

	effect->clearBuzzForce();

	return(1);
}

HDCallbackCode HDCALLBACK BeginFrameCallback(void *)
{
	hdBeginFrame(hdGetCurrentDevice());
	return HD_CALLBACK_CONTINUE;
}
HDCallbackCode HDCALLBACK EndFrameCallback(void *)
{
	hdEndFrame(hdGetCurrentDevice());
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
	if (timeElapsed > 1/(float)SCHEDULER_RATE)
	{
		assert(false && "Scheduler has exceeded scheduler rate.");
		cout << "Scheduler has exceeded scheduler rate: " << timeElapsed << endl;
	}
	return HD_CALLBACK_CONTINUE;
}


//Onur buradan  itibaren
/*******************************************************************************
 Callback that queries motor temperature.
*******************************************************************************/
HDCallbackCode HDCALLBACK QueryMotorTemp(void *pUserData)
{
    aMotorTemp = (HDdouble *) pUserData;

    hdGetDoublev(HD_MOTOR_TEMPERATURE, aMotorTemp);

    return HD_CALLBACK_DONE;
}

/*******************************************************************************
 Callback for haptic loop.
*******************************************************************************/
HDCallbackCode HDCALLBACK MyHapticLoop(void *pUserData)
{   
	// timestamp 

	SYSTEMTIME t;
	long ms;

	static Vector forceFB = Vector(0, 0, 0);
	static float tmpFX = 0, tmpFZ = 0, tmpFInerX = 0, tmpFInerZ = 0;
	
	Ball *ball = graphCtrller->ballGr->getBall();
	static bool firstTime = true;

	Point myP;
	hduVector3Dd pos;
	hdGetDoublev(HD_CURRENT_POSITION, pos);

	myP.setValue((float)pos[0], /*(float)pos[1]*/0, (float)pos[2]/**1.5f*/);		


	ctrlForce = Vector(0, 0, 0);
	graphCtrller->iPs[1].setPosition( myP );
	
	Point ballDirection;
	Point movementDirection;
	
	effect->PrintMotorTemp(aMotorTemp, gNumMotors);

	if((firstTime == true) && ((effect->keepHapticLoop) == true))
	{
		firstTime = false;

		//t = time(NULL);	elapsed = (long)t;
		/*sec_init();*/
		runs = 1;
		angleX = 0;	angleZ = 0;	

		for(int i=0; i < TARGET_COUNT; i++)	{
			contactCyl[i] = 0;
			insideCyl[i] = 0;
			targetVelocity[i] = Vector(0,0,0);
		}
		// AYSE: add keyframes
		for(int i=0; i < KEYFRAME_COUNT; i++)	{
#ifdef DO_INITIAL_FREE_TRIAL
			contactKeyframe[i] = (i-1)%keyframesPerObs == 0 ? 0 : 1; // ignore keyframes at initial data collection trial
#else
			contactKeyframe[i] = 0;
#endif
		}
				

		ctrlForce = Vector(0,0,0);	
		goalPt[0] = 0;	goalPt[1] = 0;	goalPt[2] = 0;	prevGoalId = -1; goalId = 0;
		

		ball->incrementPosition(myP);
		//cout << ball->getPosition()[0] << " " << ball->getPosition()[2] << endl;
		// AYSE: set to ball's position
		graphCtrller->iPs[0].setPosition(ball->getPosition()/*myP*/);
		graphCtrller->iPs[0].setVelocity(Vector(0, 0, 0));
		graphCtrller->iPs[2].setPosition(ball->getPosition()/*myP*/);
		graphCtrller->iPs[2].setVelocity(Vector(0, 0, 0));
	}
	else if((effect->keepHapticLoop) == true)
	{
		float hpX, hpY, hpZ, npX, npY, npZ;
		graphCtrller->iPs[1].getPosition().getValue(hpX, hpY, hpZ);
		graphCtrller->iPs[2].getPosition().getValue(npX, npY, npZ);

		P[0] = myP[0];	P[1] = myP[1];	P[2] = myP[2];

		tmpFInerX = o_gravity * sinf( o_thetaZ );
		tmpFInerZ = o_gravity * sinf( o_thetaX );

		if(ball->posX > (boundary[0]-boundaryThickness-ballRadius))
		{
			ball->setPosition( Point(boundary[0]-boundaryThickness-ballRadius, ball->posY, ball->posZ) );
			
			if(ball->velX > 0) {
				ball->velX = 0;		ball->accX = 0;
			}
		}
		else if(ball->posX < -(boundary[0]-boundaryThickness-ballRadius))
		{
			
			if (!ball->setPosition( Point(-(boundary[0]-boundaryThickness-ballRadius), ball->posY, ball->posZ) ))
				effect->userFlaggedMoment = true;
						
			if(ball->velX < 0) {
				ball->velX = 0;		ball->accX = 0;
			}
		}

		// check boundaries of the board for ball position - z-axis
		if(ball->posZ > (boundary[2]-boundaryThickness-ballRadius))
		{
			if (!ball->setPosition( Point(ball->posX, ball->posY, boundary[2]-boundaryThickness-ballRadius) ))
				effect->userFlaggedMoment = true;
			
			if(ball->velZ > 0) {
				ball->velZ = 0;		ball->accZ = 0;
			}
		} 
		else if(ball->posZ < -(boundary[2]-boundaryThickness-ballRadius))
		{
			if (!ball->setPosition( Point(ball->posX, ball->posY, -(boundary[2]-boundaryThickness-ballRadius)) ))
				effect->userFlaggedMoment = true;
			
			if(ball->velZ < 0) {
				ball->velZ = 0;		ball->accZ = 0;
			}
		}

		// AYSE: make obstacle boundaries rigid
		// AYSE: do intersection / collision detection
		// AYSE: do checks at threshold collection trial
		Point* obsCorners;
		float sinAlpha;
		float cosAlpha;
		float newPosChangeMag;
		float newXPos, newZPos;

		int keyframesPerObs = KEYFRAME_COUNT / OBSTACLE_COUNT;

		graphCtrller->rescueFromPit = false;
		goalPt = keyframeGoalCenters[goalId];
		effect->fellInPit = false;

		for(int i=0; i < OBSTACLE_COUNT; i++)
		{
			obsCorners = graphCtrller->obstacleGrs->at(i)->obstacleCornerPts;
		
			for (int j=0; j < NUM_OBS_CORNERS-1; j++)
			{	
				if ( graphCtrller->ballIntersectsLine(obsCorners[j], obsCorners[j+1]) )
				{
					graphCtrller->hitBounds[i][j] = true;
					effect->fellInPit = true;
						
					runsInPits++;
					

					if (!pitTickCounterStarted[i][j])
					{
						startTickForPits		= getTicks();
						pitTickCounterStarted[i][j]	= true;	
					}				


#ifdef DO_INITIAL_FREE_TRIAL
					if (effect->curTrial > 1)
#endif
					{
						if ( goalId % keyframesPerObs != keyframesPerObs-1 ) 
						{
							graphCtrller->rescueFromPit = true;

							if (goalId % keyframesPerObs == 0)
								goalPt = keyframeGoalCenters[goalId+2];// HACK: AYSE: targetCenters[goalId];
							else if (goalId % keyframesPerObs == 1)
							{
								goalPt = keyframeGoalCenters[goalId+1];// HACK: AYSE: targetCenters[goalId];
								contactKeyframe[goalId-1] = 0;
							}
						}
					
						else
						{
							graphCtrller->rescueFromPit = false;
							contactKeyframe[goalId] = 1;
							goalPt = keyframeGoalCenters[goalId+1];
						}
					}
#ifdef DO_INITIAL_FREE_TRIAL
					else
					{
						graphCtrller->rescueFromPit = false;
						goalPt = keyframeGoalCenters[goalId];
					}
#endif

					float x1 = obsCorners[j][0];
					float z1 = obsCorners[j][2];
					float x2 = obsCorners[j+1][0];
					float z2 = obsCorners[j+1][2];
					float x0 = ball->posX;
					float z0 = ball->posZ;
					float delta = BALL_RADIUS;

					float dx    = x2 - x1;
					float dz	= z2 - z1;
					float dist  = EUC_DIST(x1, z1, x2, z2);

					float numerator = (pow(x1-x0,2) + pow(z1-z0,2)) * (pow(dx,2) + pow(dz,2))
										- pow( (z1-z0)*dx-(x1-x0)*dz, 2 );
					
					float denominator = pow(dx,2) + pow(dz,2);
					float ratio;

					
					if (numerator >= 0 && denominator > 0)
					{
						ratio = sqrt( abs(numerator) ) / denominator;
						newXPos = x1 + dx*ratio;
						newZPos = z1 + dz*ratio;
					}
					else
					{
						ratio = 0;
						newXPos = ball->posX;
						newZPos = ball->posZ;
					}		 

					// if close to corners, let the ball off the border
					if ( ( newXPos > x1 - delta && newXPos < x1 + delta ) ||
						 ( newXPos > x2 - delta && newXPos < x2 + delta ) ) 
					{
				
						ballDirection = Point(ball->posX - ball->prevPosX,
											  0, 
											  ball->posZ - ball->prevPosZ);

						sinAlpha = (z1 - z2) / dist;
						
						cosAlpha = (x1 - x2) / dist;	
									
						newPosChangeMag = ballDirection[0] * cosAlpha +
										  ballDirection[2] * sinAlpha;
						
						newXPos = ball->prevPosX + newPosChangeMag * cosAlpha ;
						newZPos = ball->prevPosZ + newPosChangeMag * sinAlpha ;	
					}			
					if (!ball->setPosition( Point(newXPos , ball->posY, newZPos ) ))
						effect->userFlaggedMoment = true;
					

					// update score time triply if in the pits
					scoreTime += 10;
					setScoreText(scoreTime);

					ball->setVelocity( Vector(0,0,0) );
					ball->setAcceleration( Vector(0,0,0) );
				}
				else
				{
					if (pitTickCounterStarted[i][j])
					{
						ticksInPits				+= getTicks() - startTickForPits;
						pitTickCounterStarted[i][j]	 = false;	
					}

					graphCtrller->hitBounds[i][j] = false;
				}
			}
		}

		// AYSE: enable/disable tilt of the board
		if (effect->tiltOn)
		{
			angleX = o_thetaX*2;
			angleZ = o_thetaZ*2;
		}

		// AYSE: calculate force on the spring between nhip and the ball
		ctrlForce +=  Vector( -(ball->posX - npX) * kpBN - (ball->velX * kdBN),
							0,
							-(ball->posZ - npZ) * kpBN - (ball->velZ * kdBN) );
		
		// position CHIP 
		putInGroove();
		// set where the negotiation point is
		calcNHIPpos();

		// for rotating the board !!!
		float tmpFctrlX = (ctrlForce[0] / ball->getMass());
		float tmpFctrlZ = (ctrlForce[2] / ball->getMass());
		tmpFX = tmpFctrlX - tmpFInerX;
		tmpFZ = tmpFctrlZ - tmpFInerZ;

		if ( tmpFX > 1 ) tmpFX = 0.6f;	if ( tmpFX < -1 ) tmpFX = -0.6f;
		if ( tmpFZ > 1 ) tmpFZ = 0.6f;	if ( tmpFZ < -1 ) tmpFZ = -0.6f;

		o_thetaZ = asinf( tmpFX );
		o_thetaX = asinf( tmpFZ );
		
		//cout << "npx: " << npX << " hpx: " << hpX << " " 
		//	 << "npz: " << npZ << " hpz: " << hpZ << endl;
		forceFB = Vector((npX - hpX) * kpHN, 0, (npZ - hpZ) * kpHN);
	
		if (effect->tremorOn && ctrlMode == BOTH_CTRL)
			effect->generateTremorForce();


		///* AYSE: add buzzing and tremor . buzzForce is zero outside blending phase */ 
		////HDdouble fToD[3] = { forceFB[0], forceFB[1], forceFB[2] };
		forceFB[0] += buzzForce[0] + tremorForce[0];
		forceFB[1] += buzzForce[1] + tremorForce[1]; 
		forceFB[2] += buzzForce[2] + tremorForce[2];
		
		
		float fMag = forceFB.length();
		if( fMag > FORCE_LIMIT) 
		{
			forceFB = (forceFB / fMag) * FORCE_LIMIT ;
		}
		forceFB = F_SCALE_FACTOR * forceFB;
	}	

	/* AYSE: Render two simple horizontal planes to contstrain user movement to x-z plane. */
	/* the planes are not stiff */ 
	hduVector3Dd position;
	HDdouble kStiffness;

	hdGetDoublev(HD_CURRENT_POSITION, position);
	hdGetDoublev(HD_NOMINAL_MAX_STIFFNESS, &kStiffness);

	if (position[1] < -20) // ground
	{
		forceFB[1] = kStiffness * (-20 - position[1]);
	}
	else if (position[1] > 0) // ceiling
	{
		forceFB[1] = kStiffness * (0 - position[1]);
	}

	// send all forces to device
	HDdouble fToD[3] = { forceFB[0], forceFB[1], forceFB[2] };


	effect->PreventWarmMotors( hduVector3Dd(fToD[0], fToD[1], fToD[2]) );  
	hdSetDoublev(HD_CURRENT_FORCE, fToD);

	hdScheduleSynchronous(QueryMotorTemp, aMotorTemp,
									HD_DEFAULT_SCHEDULER_PRIORITY);
    
	//HDdouble fToD[3] = { (*computedForces)[0], (*computedForces)[1], (*computedForces)[2] };

	runs++;	cntF++;
	
	scoreTime += 1;//(int)(elapsed/1000); 
	setScoreText(scoreTime);

	if(/*(firstTime == false) && */((effect->keepHapticLoop) == true))
	{
		float hpX, hpY, hpZ, npX, npY, npZ;
		graphCtrller->iPs[1].getPosition().getValue(hpX, hpY, hpZ);
		graphCtrller->iPs[2].getPosition().getValue(npX, npY, npZ);
	
	
		if( dataRec->cond == RE || dataRec->cond == REVHC )
		{
#ifdef DO_INITIAL_FREE_TRIAL
			if (effect->curTrial == 1) {
				computeFOnUserAveg(fToD);
			}
			else 
#endif
			{
				computeFOnUserAveg(fToD);
				/*				x axis				**
				**									*/
				// if still computer guidance is not switched yet
				if (!effect->waitOnTarget) // no role exchange when waiting on targets!
				{
					if ( blendAlpha <= 0 ) 
					{
						checkRolExchange( fToD[0], fToD[2] ); // AYSE: make role exchange simultaneous in 2D 
					}
					else if ( blendAlpha >= blendTime )
					{
						checkRolExBack( fToD[0], fToD[2] ); // AYSE: make role exchange simultaneous in 2D 
					}
				}
				// if the user starts gaining the whole control of x axis
				if ( userWantsCtrl ) {
					ctrlMode = BLND_CTRL;

					effect->generateBuzzForce(userWantsCtrl);
					
					forceBeWithUser( 1 );
					
					if ( blendAlpha >= blendTime ) {
						userWantsCtrl = false;
						ctrlMode = USER_CTRL;

						// AYSE: no tremor when user is in control
						effect->clearTremorForce();
						// AYSE: clear buzz and play sound if necessary
						effect->clearBuzzForce();
					}
				}
				else if ( userGivesCtrl ) {
					ctrlMode = BLND_CTRL;

					effect->generateBuzzForce(userWantsCtrl);

					forceBeWithUser( -1 );		// make comp share the ctrl on x gradually
					
					if ( blendAlpha <= 0 ) {	// if blending done
						userGivesCtrl = false;	// request finished
						ctrlMode = BOTH_CTRL;
										
						// AYSE: add tremor when computer is in control
						effect->generateTremorForce();
						// AYSE: clear buzz and play sound if necessary
						effect->clearBuzzForce();
					}
				}
			}
		}

		if (effect->update_stylus_position(myP) == -1)
			return HD_CALLBACK_DONE;
		
		//timestamp
		GetSystemTime(&t);
		ms = 60*60*1000*t.wHour + 
				 60*1000*t.wMinute + 
				 1000*t.wSecond + 
				 t.wMilliseconds;

		dataRec->ts->push_back(ms);

		// record 
#ifdef RECORD_DATA
		if(effect->userFlaggedMoment)
		{
			dataRec->flags->push_back(true);
			effect->userFlaggedMoment = false;
		}
		else
		{
			dataRec->flags->push_back(false);
		}


		if(effect->waitOnTarget)
		{
			dataRec->waitOnTargetFlags->push_back(true);
		}
		else
		{
			dataRec->waitOnTargetFlags->push_back(false);
		}

		dataRec->numPitfalls->push_back(effect->pitfallCount);

		dataRec->tick->push_back(getTicks());
		dataRec->runsCnt->push_back(runs);
		dataRec->trialID->push_back(effect->curTrial);

		dataRec->timeScore->push_back(scoreTime);
		dataRec->runsInPits->push_back(runsInPits);
		dataRec->runsOnTargets->push_back(runsOnTargets);
		dataRec->ticksInPits->push_back(ticksInPits);
		dataRec->ticksOnTargets->push_back(ticksOnTargets);
		
		dataRec->ballP->push_back(Vector(ball->posX, 0, ball->posZ));		
		dataRec->ballV->push_back(Vector(ball->velX, 0, ball->velZ));	
		dataRec->ballA->push_back(Vector(ball->accX, 0, ball->accZ));

		// if ever ball's mass/radius or game's gravity would change online
		//dataRec->ballM->push_back(ball->mass);							
		//dataRec->ballR->push_back(ball->radius);							
		//dataRec->gameGravity->push_back(o_gravity);	

		//Vector *fOnBbyIner = new SbVec3f(tmpFInerX*ball->mass, 0, tmpFInerZ*ball->mass);
		//Vector *fOnBbySpr = new SbVec3f(tmpFX*ball->mass, 0, tmpFZ*ball->mass);
		//dataRec->fOnBbyIner->push_back(*fOnBbyIner);
		//dataRec->fOnBbySpr->push_back(*fOnBbySpr);
		//dataRec->fOnCHIP->push_back(fOnCHIP);
		//dataRec->fOnNbyC->push_back(fOnNbyC);
		//dataRec->fOnNbyBSpr->push_back(fOnNbySpr);
		//dataRec->fOnNbyBIner->push_back(fOnNbyBIner);
		//dataRec->fOnNbyH->push_back(fOnNbyH);

		//dataRec->chipP->push_back(Vector(CHIP->posX, 0, CHIP->posZ));	
		dataRec->hipP->push_back(Vector(hpX, 0, hpZ));	
		dataRec->nhipP->push_back(Vector(npX, 0, npZ));	
		//dataRec->targetP->push_back(Vector(goalPt[0], 0, goalPt[2]));	

		// if k's would change online!!!
		//dataRec->kpBN->push_back(kpBN);							
		//dataRec->kdBN->push_back(kdBN);							
		//dataRec->kpHN->push_back(kpHN);							
		//dataRec->kpCN->push_back(kpCN);							
		//dataRec->kpCT->push_back(kpCT);							
		//dataRec->kdCT->push_back(kdCT);

		dataRec->thetaX->push_back(o_thetaX);
		dataRec->thetaZ->push_back(o_thetaZ);
		dataRec->targetID->push_back(goalId+1);
		dataRec->fOnUser->push_back(forceFB);

		Vector *fOnNbyC		= new SbVec3f((float)fFromNtoC[0], 0, (float)fFromNtoC[2]);
		Vector *fOnNbyH		= new SbVec3f((float)fFromNtoH[0], 0, (float)fFromNtoH[2]);
		Vector *fOnNbySpr	= new SbVec3f((float)fSprFromBtoN[0], 0, (float)fSprFromBtoN[2]);
		Vector *fOnNbyBIner	= new SbVec3f((float)fFromBall[0], 0, (float)fFromBall[2]);

		Vector *fOnCHIP = new SbVec3f(ball->getMass() * caX, 0, ball->getMass() * caZ);
		Vector *chipV =  new SbVec3f(cvX, 0, cvZ);
		Vector *chipP =  new SbVec3f(cpX, 0, cpZ);
		// change Vector with HduVec ??!!
		dataRec->fOnCHIP->push_back( *fOnCHIP );
		dataRec->chipV->push_back( *chipV );
		dataRec->chipP->push_back( *chipP );

		dataRec->fOnNbyC->push_back(*fOnNbyC);
		dataRec->fOnNbyBSpr->push_back(*fOnNbySpr);
		dataRec->fOnNbyBIner->push_back(*fOnNbyBIner);
		dataRec->fOnNbyH->push_back(*fOnNbyH);

		if ( dataRec->cond == RE || dataRec->cond == REVHC )
		{
			dataRec->kpCN->push_back(kpCN);
			//dataRec->kpHN->push_back(kpHN);
			dataRec->ctrlM->push_back(ctrlMode);
			//dataRec->boardTheme->push_back(bThemeID);
		}
#endif
	}
	


	return HD_CALLBACK_CONTINUE;
}

void HapticCallBack::stopHaptics()
{
	printf("getTicks()=> %ld\n", getTicks());

	hdStopScheduler();
	for (int i = 0; i < callbackHandlers.size(); i++)
	{
		hdUnschedule(callbackHandlers.at(i));
		callbackHandlers.pop_back();
	}
	hdDisableDevice(hdGetCurrentDevice());

#ifdef RECORD_DATA
	if (targetTickCounterStarted)
	{
		ticksOnTargets			+= getTicks() - startTickForTargets;
		targetTickCounterStarted = false;	
	}

	for (int i = 0; i < OBSTACLE_COUNT; i++)
	{
		for (int j = 0; j < NUM_OBS_CORNERS; j++)
		{
			if (pitTickCounterStarted[i][j])
			{
				ticksInPits				+= getTicks() - startTickForPits;
				pitTickCounterStarted[i][j]	 = false;
			}
		}
	}
	dataRec->writeMeToFile();
#endif
	
	
}

int HapticCallBack::update_stylus_position(Point pt)
{
	SbVec3f myVec;

	myVec[0] = pt[0];
	myVec[1] = pt[1];
	myVec[2] = pt[2];

	myNewTranslation01 = myVec;

	if(myStylusFlag == false)
	{
		float bpX, bpY, bpZ, cpX, cpY, cpZ, hpX, hpY, hpZ;
		Point bPos;
		graphCtrller->iPs[0].getPosition().getValue(cpX, cpY, cpZ);
		graphCtrller->iPs[1].getPosition().getValue(hpX, hpY, hpZ);
		bPos = graphCtrller->ballGr->getPosition();

		myNewTranslation = myVec;

		// set the coordinates for the styli on screen
		SbMat myma = {1, 0, 0, 0,
			0,	1,	0,	0,
			0,	0,	1,	0,
			myVec[0],	0,	0,	1};
		SbMat myma2 = {1, 0, 0, 0,
			0,	1,	0,	0,
			0,	0,	1,	0,
			0,	0,	myVec[2],	1};
		stylusTransMatrix1.setValue(myma);
		stylusTransMatrix2.setValue(myma2);

		// set the coordinates for the floor on screen
		SbRotation myRotX(Vector(1,0,0), angleX);
		SbRotation myRotZ(Vector(0,0,-1), angleZ);
		boardRotateMatrixX.setRotate(myRotX);
		boardRotateMatrixZ.setRotate(myRotZ);

		int index;
		int keyframesPerObs = KEYFRAME_COUNT / OBSTACLE_COUNT;

		for(int i=0; i < KEYFRAME_COUNT; i++) // AYSE: targetCount 
		{ 
			// AYSE
			if (i == goalId) 
			{	
				// HACK: AYSE: if this is not a target 
				if ( i % keyframesPerObs != 1 )
				{
					Point* obsCorners = graphCtrller->obstacleGrs->at(i/keyframesPerObs)->obstacleCornerPts;
					Ball *ball = graphCtrller->ballGr->getBall();

					float x1 = obsCorners[0][0];
					float z1 = obsCorners[0][2];
					float x2 = obsCorners[NUM_OBS_CORNERS-1][0];
					float z2 = obsCorners[NUM_OBS_CORNERS-1][2];

					// HACK: AYSE: if we passed through the entrance
					if( graphCtrller->ballIntersectsLine(obsCorners[0], obsCorners[NUM_OBS_CORNERS-1])
						&& graphCtrller->ballIntersectsLine(obsCorners[NUM_OBS_CORNERS-1], obsCorners[0]) )
					{
						// if only we are not in a pit 
						if (graphCtrller->rescueFromPit == false)
						{
							contactKeyframe[i] = 1;
						}

						//AYSE: remove itersSinceHit = 0;
					}

					// HACK: AYSE: but somehow we reached a target?
					if ( goalId % keyframesPerObs == 0 )
					{
						
						int index = (goalId)/keyframesPerObs;
						if((keyframeGoalCenters[goalId+1] - bPos).length() < (targetRadius + ballRadius)) 
						{
							contactKeyframe[goalId] = 1;
						}
					}
					
				}
				// HACK: AYSE: is this a target?
				else 
				{
					// HACK: AYSE: did we hit a keyframe
					index = (i-1)/keyframesPerObs;
					//if((targetCenters[i] - bPos).length() < (targetRadius + ballRadius)) 
					if((keyframeGoalCenters[i] - bPos).length() < (targetRadius + ballRadius)) 
					{	
						runsOnTargets++;
						if (!targetTickCounterStarted)
						{
							startTickForTargets = getTicks();
							targetTickCounterStarted = true;	
						}
						insideCyl[index]++;

						int remaining = floor( (float) (TIME_REQ_ON_TARGET - insideCyl[index]) / 10.0f );
						char counter[20];
						sprintf_s(counter, "Wait on target: %d", remaining );
						effect->waitOnTarget = true;

						setCounterText(counter);
						
						// AYSE: if we stayed on the target long enough	
						if (insideCyl[index] >= TIME_REQ_ON_TARGET)
						{
							contactCyl[index] = 1;
							// if score info showing time is also over zero counter text	
							setCounterText(" ");
							effect->waitOnTarget = false;
							// AYSE: add keyframes
							contactKeyframe[i] = 1;
							//AYSE: remove itersSinceHit = 0;
						}
					}
					else // AYSE: not touching target
					{
						if (targetTickCounterStarted)
						{
							ticksOnTargets			+= getTicks() - startTickForTargets;
							targetTickCounterStarted = false;	
						}

						if (insideCyl[index] != 0)
						{
							setCounterText(" ");
							effect->waitOnTarget = false;
						}
						insideCyl[index] = 0;
					}
				}
			}
		}

		
		//if (effect->scorePrintedOnScreen && scoreTime / 1000 > TIME_SCORE_INFO_VALID)
		//{
		//	setCounterText(" ");
		//	setWarningText(" ");
		//	scorePrintedOnScreen = false;
		//}

		if (isGameOver() != -1)
		{
			//AYSE: erase score information text after TIME_SCORE_INFO_VALID passes
			if (effect->scorePrintedOnScreen && scoreTime / 1000 > TIME_SCORE_INFO_VALID)
			{
				setCounterText(" ");
				setWarningText(" ");
				effect->scorePrintedOnScreen = false;
			}
		}
		else
		{
			if (effect->scorePrintedOnScreen == false)
			{
				char counter[30];
				sprintf_s(counter, "Time score: %d", scoreTime/1000);
				effect->scorePrintedOnScreen = true;
				setCounterText(counter);

				scoreTimeHistory[curTrial-1] = scoreTime;
					
				if (curTrial > 3 && scoreTimeHistory[curTrial-4] > scoreTimeHistory[curTrial-3] && 
						scoreTimeHistory[curTrial-3] > scoreTimeHistory[curTrial-2] && scoreTimeHistory[curTrial-2] > scoreTime)
				{
					setWarningText("Excellent!");
				}
				else if (curTrial > 2 && scoreTimeHistory[curTrial-3] >= scoreTimeHistory[curTrial-2] && scoreTimeHistory[curTrial-2] >= scoreTime)
				{
					setWarningText("Much better!");
				}
				else if(curTrial == 1 || scoreTimeHistory[curTrial-2] >= scoreTime )
				{
					//if ( (float)(runsInPits * 10) / scoreTime > 0.25 )
					//	setWarningText("Good Job! Try to prevent pits.");	
					//else if (scoreTime > 45000) // ~60 sec is what the computer does on his own
					//	setWarningText("Good Job! Try to go faster.");
					//else
					setWarningText("Good Job!");
				}
				else
				{
					setWarningText("You can do better.");	
					//if ( (float)(runsInPits * 10) / scoreTime > 0.25 )
					//	setWarningText("Try to prevent pits.");	
					//else
					//	setWarningText("Try to go faster.");
				}		
				
				
					
				scoreTime			= 0;
				runsInPits			= 0;
				ticksInPits			= 0;
				runsOnTargets		= 0;
				ticksOnTargets		= 0;
				startTickForPits	= 0;
				startTickForTargets = 0;
			}

			if (effect->curTrial == NUM_TRIALS_PER_COND)
			{
				setWarningText("Thank you!");
				cout << "really ended" << endl;
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
		if (contactKeyframe[KEYFRAME_COUNT-1] == 1)
		{
			
			// Restart game
			for (int j = 0; j < TARGET_COUNT; j++)
			{
				contactCyl[j] = 0;		
				insideCyl[j] = 0;
			}	

			// AYSE: release keyframes
			for (int j = 0; j < KEYFRAME_COUNT; j++)
			{
				contactKeyframe[j] = 0;
			}

			isGameFinished = true;
		}
		

		if (effect->isGameFinished) {
			//// AYSE: change level if needed
			//int level = graphCtrller->getLevel();

			//if (level < LEVEL_HARDEST)
			//	graphCtrller->setLevel(level+1);

			effect->guidedStyli[BLUE] = true;
			effect->guidedStyli[GREEN] = true;
			//if (effect->curTrial == 1) {
			//#ifdef USE_F_MAG
			//					smoothFGauss(1);
			//					downSampleF();
			//#else
			//					smoothFxzGauss(1);
			//					downSampleFxz();
			//#endif
			//				}

			
			effect->curTrial++;
			runs = 0;
			if (dataRec->cond == RE || dataRec->cond == REVHC)
			{
				if (effect->curTrial > 1) 
				{
					calculateThreshold(1);	
				}
			}			
			
			// update keyframe goal centers
			for (int j = 0; j < TARGET_COUNT; j++)
			{
				int newGoalId = keyframeGoalOrders[effect->curTrial-1][j];
				
				keyframeGoalCenters[j*3]	= masterKeyframeGoalCenters[newGoalId*3];
				keyframeGoalCenters[j*3+1]	= masterKeyframeGoalCenters[newGoalId*3+1];
				keyframeGoalCenters[j*3+2]	= masterKeyframeGoalCenters[newGoalId*3+2];
			}

			for (int j = 0; j < TARGET_COUNT; j++)
			{
				int newGoalId = keyframeGoalOrders[effect->curTrial-1][j];

				graphCtrller->targetGrs->insert(graphCtrller->targetGrs->begin()+j, graphCtrller->masterTargetGrs->at(newGoalId));
			}
			for (int j = 0; j < TARGET_COUNT; j++)
			{
				graphCtrller->targetGrs->pop_back();
			}

			for (int j = 0; j < OBSTACLE_COUNT; j++)
			{
				int newGoalId = keyframeGoalOrders[effect->curTrial-1][j];

				graphCtrller->obstacleGrs->insert(graphCtrller->obstacleGrs->begin()+j, graphCtrller->masterObstacleGrs->at(newGoalId));
			}
			for (int j = 0; j < OBSTACLE_COUNT; j++)
			{
				graphCtrller->obstacleGrs->pop_back();
			}


			prevGoalId = -1;
			goalId = 0;
			goalPt = keyframeGoalCenters[goalId];// HACK: AYSE: targetCenters[goalId];
			
			isGameFinished = false;
			sec_init();
		}
		else 
		{
			for (int j = 0; j < KEYFRAME_COUNT; j++) 
			{
				// AYSE remove if (contactCyl[j] == 0)	{
				if (contactKeyframe[j] == 0) // iterate till you find a zero
				{
					if (j != goalId) 
					{
						prevGoalId  = j-1; // j > 0 ? j-1: -1;
						goalId      = j;

						goalPt	= keyframeGoalCenters[goalId];// HACK: AYSE: targetCenters[goalId];
					}
					break;
				}
			}
		}		

		

		myStylusFlag = true;
	}
	return 1;
}

/* AYSE: generates force for buzzing effect to be activated when role exchange occurs
 * The force is stored in variable  buzzForce
 * its direction, amplitude and frequency are given respectively in variables 
 * buzzDirection, buzzAmplitude and buzzFreq.
 *
 * buzzForce is always added to net force, so we need to call clearBuzzForce()
 * when no buzzing is needed.
 *
 * buzz is active if buzzOn is true and computer takes control.
 *
 */
void HapticCallBack::generateBuzzForce(bool userWantsControl)
{
	if (buzzOn && !userWantsControl)
	{
		static HDdouble timer = 0;

		/* Use the reciprocal of the instantaneous rate as a timer. */
		hdGetDoublev(HD_INSTANTANEOUS_UPDATE_RATE, &instRate);

		timer += 1.0 / instRate;

		/* Apply a sinusoidal force in the direction of motion. */
		hduVecScale(buzzForce, buzzDirection, buzzAmplitude * sin(timer * buzzFreq));
	}
	else
	{
		clearBuzzForce();
	}
}

/* AYSE: zeros buzzForce since this is always added to total force on device */
void HapticCallBack::clearBuzzForce()
{   
	hduVecScale(buzzForce, buzzDirection, 0);
}


/* AYSE: generates force for tremor effect to be activated when computer is in control
 * The force is stored in variable tremorForce
 * its direction, amplitude and frequency are given respectively in variables 
 * tremorDirection, tremorAmplitude and tremorFreq
 */
void HapticCallBack::generateTremorForce()
{
	if (tremorOn && (ctrlMode == BOTH_CTRL /*|| ctrlMode == BOTH_CTRL*/) )
	{
		static HDdouble timer = 0;
		tremorFreq = ( (double)rand() / (double)RAND_MAX ) * TREMOR_FREQ_UP 
						+ TREMOR_FREQ_DOWN;

		tremorAmplitude = ( (double)rand() / (double)RAND_MAX ) * TREMOR_AMP_UP 
						+ TREMOR_AMP_DOWN;

		/* Use the reciprocal of the instantaneous rate as a timer. */
		hdGetDoublev(HD_INSTANTANEOUS_UPDATE_RATE, &instRate);

		timer += 2.0 / instRate;

		/* Apply a sinusoidal force in the direction of motion. */
		hduVecScale(tremorForce, tremorDirection, tremorAmplitude * sin(timer * tremorFreq));
	}
	else
	{
		clearTremorForce();
	}
}

/* AYSE: zeros buzzForce since this is always added to total force on device */
void HapticCallBack::clearTremorForce()
{   
	hduVecScale(tremorForce, tremorDirection, 0);
}

// AYSE: returns true if all targets in scene are hit 
int HapticCallBack::isGameOver()
{
	int gameOver = 1;
	// AYSE: keyframe check
	
	if (contactKeyframe[KEYFRAME_COUNT-1] == 0)
	{
		gameOver = -1;
	}

	for (int i = 0; i < KEYFRAME_COUNT-1; i++)
	{
		if (contactKeyframe[i] == 0)
		{
			gameOver = 0;
			break;
		}
	}	
		
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

void putInGroove()
{
	float hpX, hpY, hpZ;
	float chipTz, chipTx;
	Vector chipForce;
	Ball *ball;
	ball = graphCtrller->ballGr->getBall();
	chipForce = Vector(0, 0, 0);
	bool potentialAttraction = false;

	if (effect->curTrial > 1)
	{
		int keyframesPerObs = KEYFRAME_COUNT / OBSTACLE_COUNT;

		// AYSE: add gravitational wells when close enough to obstacles
		// disable gravitational wells if control is fully on user
		if (ctrlMode != USER_CTRL) // || ctrlMode != USER_CTRL
		{
			for (int i = 0; i < graphCtrller->obstacleGrs->size(); i++)
			{
				ObstacleGraphics *obsGr = graphCtrller->obstacleGrs->at(i);
				// if close enough to the obstacle center activate obstacle to act like a gravity well
				float dist = obsGr->distance(ball->getPosition());

				// AYSE: many goals goalId == i
				if( (goalId)/keyframesPerObs == i && (goalId)%keyframesPerObs != keyframesPerObs-1 && dist < CLOSE_ENOUGH )
				{
					graphCtrller->setActiveObstacle(i);
					potentialAttraction = false;
					break;
				}
				else
				{
					graphCtrller->resetActiveObstacles();
				}
			}
		}
		else
		{
			graphCtrller->resetActiveObstacles();
		}
	}

	chipTx = o_thetaX;	chipTz = o_thetaZ;


	float fpX, fvX, fpZ, fvZ;
	graphCtrller->iPs[0].getPosition().getValue(cpX, cpY, cpZ);
	graphCtrller->iPs[0].getVelocity().getValue(cvX, cvY, cvZ);
	graphCtrller->iPs[1].getPosition().getValue(hpX, hpY, hpZ);

	fpX = - (cpX - goalPt[0]) * kpCT;
	fvX = - cvX * kdCT;
	fpZ = - (cpZ - goalPt[2]) * kpCT;
	fvZ = - cvZ * kdCT;

	if(potentialAttraction == false)
	{
		// computer control
		if (effect->guidedStyli[BLUE])
			chipForce[0] += fpX + fvX;

		if (effect->guidedStyli[GREEN])
			chipForce[2] += fpZ + fvZ;
	}
	
	float bM = ball->getMass();
	
	// for rotating the board !!!
	float tmpFX, tmpFZ;
	
	tmpFX = (chipForce[0] - bM * o_gravity * sinf(chipTz)) / bM;
	tmpFZ = (chipForce[2] - bM * o_gravity * sinf(chipTx)) / bM;

	if ( tmpFX > 1 )	tmpFX = 1.0f;
	if ( tmpFX < -1 )	tmpFX = -1.0f;
	if ( tmpFZ > 1 )	tmpFZ = 1.0f;
	if ( tmpFZ < -1 )	tmpFZ = -1.0f;

	chipTz += asinf( tmpFX );
	chipTx += asinf( tmpFZ );
	
	Vector fInerChip;
	fInerChip[0] = o_gravity * sinf( chipTz );
	fInerChip[1] = 0;
	fInerChip[2] = o_gravity * sinf( chipTx );

	// AYSE: merge control on axes
	if (effect->guidedStyli[BLUE])
	{
		caX = fInerChip[0] + chipForce[0]/bM;
		caZ = fInerChip[2] + chipForce[2]/bM;

		//float caMag = sqrt(caX * caX + caZ * caZ);
		//if (caMag > 1.0e-5)
		//{
		//	caX = 1.0e-5 * caX / caMag;
		//	caZ = 1.0e-5 * caZ / caMag;
		//}

		cvX += caX;
		cvZ += caZ;
		cpX += cvX;
		cpZ += cvZ;

	}
	else 
	{
		caX = 0;	cvX = 0;	cpX = hpX;
		caZ = 0;	cvZ = 0;	cpZ = hpZ;
	}

	graphCtrller->iPs[0].setAcceleration(Vector(caX, 0, caZ));
	graphCtrller->iPs[0].setVelocity(Vector(cvX, 0, cvZ));
	graphCtrller->iPs[0].setPosition(Vector(cpX, 0, cpZ));


#ifdef RECORD_DATA	
	//Vector *fOnCHIP = new SbVec3f(caX*bM, 0, caZ*bM);
	//Vector *chipV =  new SbVec3f(cvX, 0, cvZ);
	//Vector *chipP =  new SbVec3f(cpX, 0, cpZ);
	//// change Vector with HduVec ??!!
	//dataRec->fOnCHIP->push_back( *fOnCHIP );
	//dataRec->chipV->push_back( *chipV );
	//dataRec->chipP->push_back( *chipP );
#endif
}

void calcNHIPpos()
{
	static HDdouble yVec[3] = {0, 1, 0};

	float distX/*FromHIP*/, distZ/*FromHIP*/;
	float cpX, cpY, cpZ;
	float npX, npY, npZ;
	float hpX, hpY, hpZ;
	Ball *ball;
	ball = graphCtrller->ballGr->getBall();

	graphCtrller->iPs[0].getPosition().getValue(cpX, cpY, cpZ);
	graphCtrller->iPs[1].getPosition().getValue(hpX, hpY, hpZ);
	graphCtrller->iPs[2].getPosition().getValue(npX, npY, npZ);

	//hduVecSet(fSprFromBtoN, -ctrlForce[0], -ctrlForce[1], -ctrlForce[2]);

	// force exerted on NHIP due to ball's accelaration 
	fFromBall[0] = ball->getFInertia()[0];
	fFromBall[1] = ball->getFInertia()[1];
	fFromBall[2] = ball->getFInertia()[2];

	// AYSE: force exerted by the spring between NHIP and ball
	hduVecSet(fSprFromBtoN, -ctrlForce[0], -ctrlForce[1], -ctrlForce[2]);
							  //( ((ball->posX - npX)*kpBN) + (ball->velX*kdBN) )
							  //, 0
							  //, ( ((ball->posZ - npZ)*kpBN) + (ball->velZ * kdBN) ) );

	
	if (effect->guidedStyli[BLUE]) {
		float distXCN = cpX - npX;
		fFromNtoC[0] = distXCN * kpCN;
	}
	if (effect->guidedStyli[GREEN]) {
		float distZCN = cpZ - npZ;
		fFromNtoC[2] = distZCN * kpCN;
	}
	if (effect->userControlledStyli[BLUE]) {
		float distXHN = hpX - npX;
		fFromNtoH[0] = distXHN * kpHN;	
	}
	if (effect->userControlledStyli[GREEN]) {
		float distZHN = hpZ - npZ;
		fFromNtoH[2] = distZHN * kpHN;
	}

	HDdouble fNetOnNHIP[3] = {0, 0, 0};

	hduVecAdd( fNetOnNHIP, fFromBall, fFromNtoC );
	hduVecAdd( fNetOnNHIP, fNetOnNHIP, fFromNtoH );
	hduVecAdd( fNetOnNHIP, fNetOnNHIP, fSprFromBtoN );

	distX = fNetOnNHIP[0] / 200.0f /*no hap_thr 0.1*/;
	distZ = fNetOnNHIP[2] / 200.0f /*no hap_thr 0.1*/;

	// HACK: AYSE: why do the scale here???
	ctrlForce[0] = fNetOnNHIP[0] / 200.0f;
	ctrlForce[2] = fNetOnNHIP[2] / 200.0f;

	graphCtrller->iPs[2].setPosition(Vector(npX+distX, 0, npZ+distZ));

	graphCtrller->ballGr->incrementPosition(Point(-fSprFromBtoN[0]/200.0f, 0, -fSprFromBtoN[2]/200.0f));
#ifdef RECORD_DATA
	//Vector *fOnNbyC		= new SbVec3f((float)fFromNtoC[0], 0, (float)fFromNtoC[2]);
	//Vector *fOnNbyH		= new SbVec3f((float)fFromNtoH[0], 0, (float)fFromNtoH[2]);
	//Vector *fOnNbySpr	= new SbVec3f((float)fSprFromBtoN[0], 0, (float)fSprFromBtoN[2]);
	//Vector *fOnNbyBIner	= new SbVec3f((float)fFromBall[0], 0, (float)fFromBall[2]);

	//// all is here but a conversion from Vector to hduVector3f would be nicer
	//dataRec->fOnNbyC->push_back(*fOnNbyC);
	//dataRec->fOnNbyBSpr->push_back(*fOnNbySpr);
	//dataRec->fOnNbyBIner->push_back(*fOnNbyBIner);
	//dataRec->fOnNbyH->push_back(*fOnNbyH);
#endif
}

void forceBeWithUser(float plusOrMinus)
{
	//cout << "kpCNHigh: " << kpCN << endl;
	kpCN = (blendAlpha/blendTime) * kpLow + (1-blendAlpha/blendTime) * kpHigh;
	kpHN = (blendAlpha/blendTime) * kpHigh + (1-blendAlpha/blendTime) * kpLow;

	blendAlpha += plusOrMinus;

	if ( blendAlpha < 0 )
		blendAlpha = 0;
	else if ( blendAlpha > blendTime )
		blendAlpha = blendTime;
}


// AYSE: make role exchange check global... functions checkRolExchangeX and checkRolExchangeZ are obsolete
void checkRolExchange(HDdouble fX, HDdouble fZ)
{
	static bool isAboveThreshold = true;
	static long timePrev = 0l;
	long timeNow = getTicks();

	if (timeNow-timePrev < 0) 
	{
		timePrev = timeNow;
	}	
	// check if we are still inside the pre-defined interval
	else if (timeNow-timePrev < ROLE_INT)
	{
		if (abs(fX) < thresholdF_Up[0] && abs(fZ) < thresholdF_Up[2])
		{
			// force on both axes are below the determined upper threshold value
			isAboveThreshold = false;			
		}
		else	
		{
			consecRolE++;		// count how many of them 
		}
	}
	else if (timeNow-timePrev == ROLE_INT)
	{
		// give the control to the user 
		// 1. all the values were above or 2. 90% of the were above -- try 2 options than decide
		if (isAboveThreshold || consecRolE > 0.8f*ROLE_INT)	
		{
			userWantsCtrl = true;		// now user would gain control
		}
		isAboveThreshold = true;
		consecRolE = 0;
		timePrev = timeNow;		// re-initialize time for the next check
	}
	else
	{
		timePrev = timeNow;		// re-initialize time for the next check
	}
}


// AYSE: make role exchange check global... 
void checkRolExBack(HDdouble fX, HDdouble fZ)
{
	static bool isBelowThreshold = true;
	static long timePrev = 0l;
	long timeNow = getTicks();

	if (timeNow-timePrev < 0) 
	{
		timePrev = timeNow;
	}
	// check if we are still inside the pre-defined interval
	else if (timeNow-timePrev < ROLE_INT)
	{
		//timePrev = timeNow;		// now we can change previous role exchange check time 
		if (abs(fX) > thresholdF_Down[0] || abs(fZ) > thresholdF_Down[2])
		{
			// force on one of (or both) x and z axes is below threshold value
			isBelowThreshold = false;			
		}
		else	
		{
			consecRolEBack++;		// count how many of them 
		}
	}
	else if (timeNow-timePrev == ROLE_INT){
		// give the control to the user on x-axis if
		// 1. all the values were above or 2. 90% of the were above -- try 2 options than decide
		if (isBelowThreshold || consecRolEBack > 0.8f*ROLE_INT)	
		{
			userGivesCtrl = true;		// now user would share control 
		}
		isBelowThreshold = true;
		consecRolEBack = 0;
		timePrev = timeNow;		// re-initialize time for the next check
	}
	else
		timePrev = timeNow;		// re-initialize time for the next check
}


void computeFOnUserAveg(HDdouble fToD[3])
{
	float deltaX, deltaZ;

	meanVarCnt++;
	deltaX		= abs(fToD[0]) - fCurAvg[0];		
	fCurAvg[0]	= fCurAvg[0] + deltaX / meanVarCnt;	// find m_new
	// This expression uses the new value of mean
	M2x			+= deltaX*(abs(fToD[0]) - fCurAvg[0]);	// find M2_new 

	deltaZ		= abs(fToD[2]) - fCurAvg[2];
	fCurAvg[2]	= fCurAvg[2] + deltaZ / meanVarCnt;
	// This expression uses the new value of mean
	M2z			+= deltaZ*(abs(fToD[2]) - fCurAvg[2]); 
	/*fCurAvg[0] = (fCurAvg[0]*(runs-1) + abs(fToD[0])) / runs;
	fCurAvg[2] = (fCurAvg[2]*(runs-1) + abs(fToD[2])) / runs;*/
	//fCurAvg = (fCurAvg*(runs-1) + hduVecMagnitude(fToD)) / runs;
}

void calculateThreshold(int dSampSize)
{
	fVar[0] = M2x/(meanVarCnt-1);		fVar[2] = M2z/(meanVarCnt-1);
	fStd[0]	= InvSqrt(1.0f/fVar[0]);	fStd[2]	= InvSqrt(1.0f/fVar[2]);

	float decx = /*(1-abs(1-fCurAvg[0])) * */fStd[0];
	float decz = /*(1-abs(1-fCurAvg[2])) * */fStd[2];
	thresholdF_Up[0] = (float)fCurAvg[0] + decx;
	thresholdF_Up[2] = (float)fCurAvg[2] + decz;

	thresholdF_Down[0] = (float)fCurAvg[0] - decx;
	thresholdF_Down[2] = (float)fCurAvg[2] - decz;

	if ( thresholdF_Up[0] > FORCE_LIMIT )		
	{
		thresholdF_Up[0] = FORCE_LIMIT * F_SCALE_FACTOR;
	}
	
	if ( thresholdF_Up[2] > FORCE_LIMIT )		
	{
		thresholdF_Up[2] = FORCE_LIMIT * F_SCALE_FACTOR;
	}

	if ( thresholdF_Down[0] < 0.2f )	
	{
		thresholdF_Down[0] = 0.2;
	}

	if ( thresholdF_Down[2] < 0.2f )	
	{
		thresholdF_Down[2] = 0.5;
	}

	//cout << "Avg: " << fCurAvg[0]		<< " " << fCurAvg[2]	<< endl;
	//cout << "Var: " << fVar[0]			<< " " << fVar[2]		<< endl;
	//cout << "Std: " << fStd[0]			<< " " << fStd[2]		<< endl;
	//cout << "ThU: " << thresholdF_Up[0]	<< " " << thresholdF_Up[2] << endl;
	//cout << "ThD: " << thresholdF_Down[0]<< " " << thresholdF_Down[2] << endl;
}

void smoothFxzGauss(int callID)
{	
	float	tmpFOnUx = 0.0f, tmpFOnUz = 0.0f;
	int		initSmo = runs-SMOOTH_FREQ;
	//cout << initSmo << endl;
	// handle last SMOOTH_FREQ/2 force values
	if (callID == 1) {
		for (unsigned int i=(runs-SMOOTH_FREQ/2); i<runs; i++) {
			dataRec->smoothFxOnUser->push_back( (float) ((Vector)dataRec->fOnUser->at(i))[0] );
			dataRec->smoothFzOnUser->push_back( (float) ((Vector)dataRec->fOnUser->at(i))[2] );
		}
	}
	else if (runs<SMOOTH_FREQ) {
		if (runs<SMOOTH_FREQ/2) {
			dataRec->smoothFxOnUser->push_back( (float) ((Vector)dataRec->fOnUser->at(runs-1))[0] );
			dataRec->smoothFzOnUser->push_back( (float) ((Vector)dataRec->fOnUser->at(runs-1))[2] );
		}
	}
	else {
		for (unsigned int i=0; i<SMOOTH_FREQ; i++) {
			tmpFOnUx += ((float) ((Vector)dataRec->fOnUser->at(initSmo))[0]) * gaussCoef[i];	// x-smooth with a gaussian filter;
			tmpFOnUz += ((float) ((Vector)dataRec->fOnUser->at(initSmo))[2]) * gaussCoef[i];	// z-smooth with a gaussian filter;
			initSmo++;
		}
		dataRec->smoothFxOnUser->push_back( tmpFOnUx );	// store the smoothed force values x
		dataRec->smoothFzOnUser->push_back( tmpFOnUz );	// store the smoothed force values z
	}
	//cout << initSmo << endl;
}

void downSampleF()
{
	int j=0;
	int dSampSize = (int)(runs/DOWNSAMPLE_FREQ);
	dSampF = (float *)malloc(sizeof(float)*dSampSize);
	//cout << (int)(runs/DOWNSAMPLE_FREQ) << endl;
	//ofstream fileOutPos;
	//fileOutPos.open("dSampF.txt");
	for (int i=0; i<runs; i+=DOWNSAMPLE_FREQ){
		dSampF[j] = (float)dataRec->smoothFOnUser->at(i);
		//fileOutPos << dSampF[j] << ", ";
		j++;
	}
	//fileOutPos.close();
	calculateThreshold(dSampSize);
}

void downSampleFxz()
{
	int j=0;
	int dSampSize = (int)(runs/DOWNSAMPLE_FREQ);
	dSampFx = (float *)malloc(sizeof(float)*dSampSize);
	dSampFz = (float *)malloc(sizeof(float)*dSampSize);
	//cout << (int)(runs/DOWNSAMPLE_FREQ) << endl;
	//ofstream fileOutPos;
	//fileOutPos.open("dSampF.txt");
	for (int i=0; i<runs; i+=DOWNSAMPLE_FREQ){
		dSampFx[j] = (float)dataRec->smoothFxOnUser->at(i);
		dSampFz[j] = (float)dataRec->smoothFzOnUser->at(i);
		//fileOutPos << dSampFx[j] << "\t" << dSampFz[j] << endl;
		j++;
	}
	//fileOutPos.close();
	calculateThreshold(dSampSize);
}

void hbgGauss()
{
	int j=0;
	float sumGauCoef=0.0f;

	for (int i=(-SMOOTH_FREQ/2); i<SMOOTH_FREQ/2; i++){
		gaussCoef[j] = expf(-SQUARE((float)i)/(2.0f*SQUARE(HBG_VARIANCE)));
		sumGauCoef += gaussCoef[j];
		j++;
	}
	//ofstream fileOutPos;
	//fileOutPos.open("gCoef.txt");
	for (int i=0; i<SMOOTH_FREQ; i++) {
		gaussCoef[i] = gaussCoef[i] / sumGauCoef;
		//fileOutPos << gaussCoef[i] << " ";
	}
	//fileOutPos.close();
}

void copyBallToCHIP()
{
	graphCtrller->iPs[0].setPosition(graphCtrller->ballGr->getPosition());
	graphCtrller->iPs[0].setVelocity(graphCtrller->ballGr->getVelocity());
}

void setHIPpos()
{
	float randPx, randPz;
	randPx = rand() % HIP_RANDPOS;
	randPz = rand() % HIP_RANDPOS;
	if ( (rand() % 2) == 0 )
		HIP->posX = CHIP->posX + randPx;
	else
		HIP->posX = CHIP->posX - randPx;

	if ( (rand() % 2) == 0 )
		HIP->posZ = CHIP->posZ + randPz;
	else
		HIP->posZ = CHIP->posZ - randPz;
}


void calcForceFromBoardRot(Point magPt, Vector &fBoard)
{
	fBoard[0] = -magPt[0] / (boundary[0] - boundaryThickness) * maxForceFromBoard;
	fBoard[1] = 0;
	fBoard[2] = -magPt[2] / (boundary[2] - boundaryThickness) * maxForceFromBoard;
}

float InvSqrt (float x)
{
	float xhalf = 0.5f*x;
	int i = *(int*)&x;
	i = 0x5f3759df - (i>>1);
	x = *(float*)&i;
	return x*(1.5f - xhalf*x*x);
}


void smoothFGauss(int callID)
{	
	float	tmpFOnU = 0.0f;
	int		initSmo = runs-SMOOTH_FREQ;

	// handle last SMOOTH_FREQ/2 force values
	if (callID == 1) {
		for (unsigned int i=(runs-SMOOTH_FREQ/2); i<runs; i++) 
			dataRec->smoothFOnUser->push_back( (float)dataRec->fOnUMag->at(i) );
	}
	else if (runs<SMOOTH_FREQ) {
		if (runs<SMOOTH_FREQ/2)
			dataRec->smoothFOnUser->push_back( (float)dataRec->fOnUMag->at(runs-1) );
	}
	else {
		for (unsigned int i=0; i<SMOOTH_FREQ; i++)
			tmpFOnU += (float)dataRec->fOnUMag->at(initSmo++) * gaussCoef[i];	// smooth wtih a gaussian dist.
		dataRec->smoothFOnUser->push_back( tmpFOnU );	// store the smoothed force values
	}
}