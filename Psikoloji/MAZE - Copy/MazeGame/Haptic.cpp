/**************************************************************************
Developed by: Ozgur Oguz and Ayse Kucukyilmaz 
Purpose: The server loop for haptic.
Date: Jul. 22, 2009
**************************************************************************/

// INCLUDE FILES
#include <float.h>
#include "Public.h"
#include "MathCB.h"
#include "GraphicsController.h"
#include "HapticCallBack.h"
#include "HapticInterfacePoint.h"
#include "BgTimer.h"

#include "SoundPlayerThread.h"
// EXTERNED VARIABLES
extern HapticCallBack *effect;
extern GraphicsController *graphCtrller;
extern void setScoreText(int score);
extern void setWarningText(char* warning);


extern float ballRadius;

Vector boundary;
float boundaryThickness;
float angle_rot;
static HDdouble *gServoMotorTemp = 0;
static HDint gNumMotors;
static HDdouble *aMotorTemp = 0;

SbVec3f *computedForces = new SbVec3f(0,0,0);
static HDdouble fFromBall[3] = {0, 0, 0}, fSprFromBtoN[3] = {0, 0, 0},
fFromNtoC[3] = {0, 0, 0}, fFromNtoH[3] = {0, 0, 0};
float cpX, cpY, cpZ, cvX, cvY, cvZ, caX, caY, caZ;
int wall_hit=0;
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



int crunsw=0;
int crunsb=0;
// var for path vis
int ballPathPtCnt = 0, ballPathUpdCnt = 0;
int findMin(float dista[12]);
int findMAX(float dista[12]);
static int collision;
// GLOBALS
SbVec3f new_probe_tip;
//SbVec3f new_probe_tip2;

float P[3], P2[3], V[3], V2[3];
float angleeb;
float wallZ11=wallTz+wallDepth*0.5;//wallTz+wallDepth*0.5+bRadius;//17;
float wallZ12=wallTz-wallDepth*0.5;//-wallTz-wallDepth*0.5-bRadius;//-27
float wallZ21=wallTz2+wallDepth*0.5;;//wallTz2+wallDepth*0.5+bRadius;//12;
float wallZ22=wallTz2-wallDepth*0.5;//wallTz2-wallDepth*0.5-bRadius;//-32;
float wallZ31=wallTz3+wallDepth*0.5;;//wallTz2+wallDepth*0.5+bRadius;//12;
float wallZ32=wallTz3-wallDepth*0.5;//wallTz2-wallDepth*0.5-bRadius;//-32;

float wallX1=wallTx;//+0.5*wallWidth;//+wallWidth*0.5;
float wallX2=wallTx2;//-0.5*wallWidth;//-wallWidth*0.5;
float wallX3=wallTx3;

float tolarance=2.0f;
float Torque;
int wall_n=3;
float xwall[3][3];
Vector ctrlForce;
int xcoll;
/* ozgur - */
static long int elapsed = 0, elapsedTurn, runs = 0, cntF = 0;//The variables in these two lines
static time_t t;	       		// are used to calculate the server rate.
float maxForceFromBoard = 2.0f; // for each axis
void  sort(float bpoints[5][4],float minz[4],int numbers[4],int or);
float angleX, angleZ, positionX, positionZ, velocityX, velocityZ, accX, accZ;

static int  scoreTime					= 0;

// new model
const int ctrlTimestep = 100;			// 100 ms - for calculating the force to control the ball trajectory
const float o_gravity = 9.81f/10000.0f;	// gravity constant cm/ms^2
float kpBN = 0.5, kdBN = 0.15f; //0.5f, kdBN = 0.0015f; //kpBN = 0.1/*0.000000900f*/,	kdBN = 0.86f;/*0.0015500f*///86f;
float kpCT = 0.0000050f,	kdCT = 0.003500f;//kdCT = 0.00199000f;

#ifdef PHANTOM_PREMIUM
float o_massBall = (SCHEDULER_RATE != 500) ? 0.15f : 0.4f;	// mass of the ball - kg
float kpHN = kpHigh;
float kpCN = kpLow;
#else
float o_massBall = 0.4f;			// mass of the ball - kg


//Force on Nip
float kpHN = 0.5;//0.4;//0.01;//0.04;//0.090f;0.01;//
float kpCN = 0.5;//;//0.01;//0.04;//0.090f;0.01;//
float kdN  = 0.5f;//0.002;//0.005f;0.0005;//


//Force on Ball
float K_BALL_X = 0.090f;
float K_BALL_Z = 0.09f;
float kdBall = 0.005;


//WALL
float dpHN = 0.025f;
float dpCN = 0.025f;
float kpHW=0.050f;//YOK
float kpCW=0.050f;//YOK
float K_WALL_X=.04f;
float K_WALL_Z=.04f;
float Kd_WALL=0.0003F;
float K_WALL_X1=.01f;
float K_WALL_Z1=.01f;
float Kd_WALL1=0.0003F;

Vector fhw=Vector(0,0,0);
Vector fcw=Vector(0,0,0);
float boundaryforceHX=0.0f;
float boundaryforceHZ=0.0f;
float boundaryforceCX=0.0f;
float boundaryforceCZ=0.0f;
float boundaryforceNX=0.0f;
float boundaryforceNZ=0.0f;

float fhw_arr[2][3];
float fcw_arr[2][3];

float boundaryforceHXa[2];
float boundaryforceHZa[2];
float boundaryforceCXa[2];
float boundaryforceCZa[2];

/*fhw_arr[0][0]=0.0f;
fhw_arr[0][2]=0.0f;

fhw_arr[1][0]=0.0f;
fhw_arr[1][2]=0.0f;

fcw_arr[0][0]=0.0f;
fcw_arr[0][2]=0.0f;

fcw_arr[1][0]=0.0f;
fcw_arr[1][2]=0.0f;

boundaryforceHXa[0]=0.0f;
boundaryforceHXa[1]=0.0f;

boundaryforceHZa[0]=0.0f;
boundaryforceHZa[1]=0.0f;

boundaryforceCXa[0]=0.0f;
boundaryforceCXa[1]=0.0f;

boundaryforceCZa[0]=0.0f;
boundaryforceCZa[1]=0.0f;
*/
float f_forceX;
float f_forceZ;
#endif

float prevHZb,prevHXb,prevCZb,prevCXb;
float prevHZw,prevHXw,prevCZw,prevCXw;

float distx=0.0f;
float distz=0.0f;


/* kpBN = 0.002; kdBN = 0.22 // nice looking!*/
HapticInterfacePoint *CIP, *NIP, *HIP,*HANDLEH,*HANDLEC;	// Controlled, Negotiated, and Real Haptic Interface Points
//Ball *ball;
extern DataRecord *dataRec;
//extern ForceAngleRec *faRec;
float	sumFs[3] = {0.0f, 0.0f, 0.0f}, sqrFs[3] = {0.0f, 0.0f, 0.0f};

extern vector <HDSchedulerHandle> callbackHandlers;
void setForces(Vector wallDisp,int forcetype);
void setForces2(int r11,int r22,int r33, int r44,int forcetypes);
void calcForceFromBoardRot(Point pt, Vector &fBoard);
void boundaryColl();
Vector boundaryColl2(float angle,Vector ballPoint);
void copyBallToCIP();
Vector calcBallPos(float angle);
Vector calcNipPos();
Vector calcHandleCPos(float angle);
Vector calcHandleHPos(float angle);
Vector xwallColl(float wallx1,float wallx2,float wallz1,float wallz2,Vector ballPoint,float angle);
Vector wallCollPoints(float wallx1,float wallx2,float wallz1,float wallz2,Vector Points,Vector ballPoint);
Vector xwallColl_sp(float wallx1,float wallx2,float wallz1,float wallz2,Vector ballPoint);
Vector getForce();
void checkHip(float array1[1][4]);
void createLen(float lenArray[49][4]);
float InvSqrt (float x);
int sign(float number);
Vector collX(Vector ball);
float getTorque(Vector ballPoint,Vector wall,float angle);
HDdouble instRate;
int forcetypes;
Vector prevpos;
Vector prevpos2;
int giveWarningW;
int giveWarningB;

Vector fOnHip,fOnCip;
float theta_rotation;

HapticCallBack::HapticCallBack()
{ 
	curTrial = 0;

	// both styli are guided whence a guidance method is selected
	guidedStyli[BLUE]			= false;
	guidedStyli[GREEN]			= false;
	userControlledStyli[BLUE]	= true;
	userControlledStyli[GREEN]	= true;

	soundOn			= true;
	tiltOn			= false;
	hipsOn			= true;
	vectorOn		= false;
	RvectorOn		= true;
	scoreTextOn		= true;
	warningOn       = true;

	hitWalls = false;

	// initially user controls both axes
	separateAxes	= false;
	keepHapticLoop	= false;
	isGameFinished	= true;

	//	ball = new Ball;

	CIP			= new HapticInterfacePoint;
	NIP			= new HapticInterfacePoint;
	HIP			= new HapticInterfacePoint;
	HANDLEH		= new HapticInterfacePoint;
	HANDLEC		= new HapticInterfacePoint;

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

	cout << "FORCE_LIMIT: " << FORCE_LIMIT << endl
		<< "KP_LIMIT: " << KP_LIMIT << endl
		<< "KD_LIMIT: " << KD_LIMIT << endl;

	cout << "Current KP: " << 1.0 / ( 1.0 / kpHN + 1.0 / (kpBN + (1.0/(1.0/kpCT	 + 1.0/kpCN))) ) << endl
		<< "Current KD: " << 1.0 / (1.0/kdBN + 1.0/kdCT) << endl;

	cout << endl << "Phantom id: " << (int)hHD1<< " is initialized." << endl;

	hHD2 = hdInitDevice(DEVICE2);

	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		//hduPrintError(stderr, &error, "Failed to initialize haptic device");
		printf("Failed to initialize haptic device");
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

	hdGetDoublev(HD_NOMINAL_MAX_CONTINUOUS_FORCE, &FORCE_LIMIT);
	hdGetFloatv(HD_NOMINAL_MAX_STIFFNESS, &KP_LIMIT);
	hdGetFloatv(HD_NOMINAL_MAX_DAMPING, &KD_LIMIT);

	cout << "FORCE_LIMIT: " << FORCE_LIMIT << endl
		<< "KP_LIMIT: " << KP_LIMIT << endl
		<< "KD_LIMIT: " << KD_LIMIT << endl;

	cout << "Current KP: " << 1.0 / ( 1.0 / kpHN + 1.0 / (kpBN + (1.0/(1.0/kpCT	 + 1.0/kpCN))) ) << endl
		<< "Current KD: " << 1.0 / (1.0/kdBN + 1.0/kdCT) << endl;

	cout << endl << "Phantom id: " << (int)hHD1<< " is initialized." << endl;

	return(1);
}

HDCallbackCode HDCALLBACK BeginFrameCallback(void *)
{
	hdBeginFrame(effect->hHD1);
	hdBeginFrame(effect->hHD2);

	return HD_CALLBACK_CONTINUE;
}
HDCallbackCode HDCALLBACK EndFrameCallback(void *)
{
	hdEndFrame(effect->hHD2);
	hdEndFrame(effect->hHD1);
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
	fhw=Vector(0.0,0.0,0.0);
	fcw=Vector(0.0,0.0,0.0);
	boundaryforceHX=0.0f;
	boundaryforceHZ=0.0f;
	boundaryforceCX=0.0f;
	boundaryforceCZ=0.0f;
	boundaryforceNX=0.0f;
	boundaryforceNZ=0.0f;
	
	static Vector forceFB1 = Vector(0, 0, 0);
	static Vector forceFB2 = Vector(0, 0, 0);
	static float tmpFX = 0, tmpFZ = 0, tmpFInerX = 0, tmpFInerZ = 0;

	Ball *ball = graphCtrller->ballGr->getBall();
	Ball *ball2 = graphCtrller->ballGr2->getBall();
	static bool firstTime = true;
	Vector wall_force;
	Vector ballPoint;
	float friction=0.005;
	float angle11;
	float prevPosX;
	float prevPosZ;


	Point myP1, myP2;
	hduVector3Dd pos;
	hdMakeCurrentDevice(effect->hHD1);
	hdGetDoublev(HD_CURRENT_POSITION, pos);

	myP1.setValue((float)pos[0], /*(float)pos[1]*/0, (float)(pos[2])/**1.5f*/);		

	graphCtrller->hip.setPosition( myP1 );

	hdMakeCurrentDevice(effect->hHD2);
	hdGetDoublev(HD_CURRENT_POSITION, pos);

	myP2.setValue((float)pos[0], /*(float)pos[1]*/0, (float)(pos[2])/**1.5f*/);		

	graphCtrller->cip.setPosition( myP2 );

	ctrlForce = Vector(0, 0, 0);

	Point ballDirection;
	Point movementDirection;

	effect->PrintMotorTemp(aMotorTemp, gNumMotors);

	if((firstTime == true) && ((effect->keepHapticLoop) == true))
	{
		firstTime = false;

		//   fprintf (fp,"feklf");

		//t = time(NULL);	elapsed = (long)t;
		/*sec_init();*/
		runs = 1;
		angleX = 0;	angleZ = 0;	

		ctrlForce = Vector(0,0,0);	

		// AYSE: set CIP, HIP and NIP to ball's position
		graphCtrller->nip.setPosition(0.5*(myP1 + myP2) );
		Vector nipp;//=0.5*(myP1 + myP2);
		nipp=Vector(-ball_width,0,0);
		graphCtrller->handleH.setPosition(Point(nipp[0],nipp[1],nipp[2]+ball_depth*0.5 ));
		graphCtrller->handleC.setPosition(Point(nipp[0],nipp[1],nipp[2]-ball_depth*0.5 ));

		//ball->setPosition(0.5*(myP1 + myP2) , true);

		graphCtrller->cip.setVelocity(Vector(0, 0, 0));
		graphCtrller->hip.setVelocity(Vector(0, 0, 0));
		graphCtrller->handleH.setVelocity(Vector(0, 0, 0));
		graphCtrller->handleC.setVelocity(Vector(0, 0, 0));
		graphCtrller->nip.setVelocity(Vector(0, 0, 0));
		ball->setVelocity(Vector(0, 0, 0));
	}
	else if((effect->keepHapticLoop) == true)
	{
		float cpX, cpY, cpZ, hpX, hpY, hpZ, npX, npY, npZ,bbpX,bbpY,bbpZ;
		float cvX, cvY, cvZ, hvX, hvY, hvZ,nvX,nvY,nvZ,hhvX,hhvY,hhvZ,hcvX,hcvY,hcvZ;
		float hhpX,hhpY,hhpZ,hcpX,hcpY,hcpZ;
		float distance;
		graphCtrller->cip.getPosition().getValue(cpX, cpY, cpZ);
		graphCtrller->hip.getPosition().getValue(hpX, hpY, hpZ);
		graphCtrller->nip.getPosition().getValue(npX, npY, npZ);
		graphCtrller->handleH.getPosition().getValue(hhpX, hhpY, hhpZ);
		graphCtrller->handleC.getPosition().getValue(hcpX, hcpY, hcpZ);
		graphCtrller->handleH.getVelocity().getValue(hhvX, hhvY, hhvZ);
		graphCtrller->handleC.getVelocity().getValue(hcvX, hcvY, hcvZ);
		Vector cpos,hpos;
		cpos=Vector(cpX,cpY,cpZ);
		hpos=Vector(hpX,hpY,hpZ);
		//cout<<hpX<<"   "<<hpZ<<"  "<<cpX<<"   "<<cpZ<<"  "<<endl;
		//P[0] = myP1[0];	P[1] = myP1[1];	P[2] = myP1[2];

		// set the negotiation point 
		// AYSE: calculate force on the spring between nhip and the ball
		ctrlForce +=  Vector( -(ball->posX - npX) * kpBN - (ball->velX * kdBN),
			0,
			-(ball->posZ - npZ) * kpBN - (ball->velZ * kdBN) );

		ballPoint=calcBallPos(angle_rot);
		wall_force[0]=0;
		wall_force[1]=0;
		wall_force[2]=0;
		
		boundaryforceHX=0;///cigil 21 eylul
		boundaryforceCX=0;
		boundaryforceHZ=0;
		boundaryforceCZ=0;
		fhw[0]=0;
		fhw[2]=0;
		fcw[0]=0;
		fcw[2]=0;
		giveWarningW=0;
		giveWarningB=0;

		collision=0;

		xwall[0][0]=wallX1;
		xwall[0][1]=wallZ11;
		xwall[0][2]=wallZ12;
		
		xwall[1][0]=wallX2;
		xwall[1][1]=wallZ21;
		xwall[1][2]=wallZ22;

		xwall[2][0]=wallX3;
		xwall[2][1]=wallZ31;
		xwall[2][2]=wallZ32;

		//cout<<xwall[2][0]<<"	z	"<<xwall[2][1]<<endl;
		Vector wallDisp;	
		float x_distance;//[2];
		Vector wallCollide;


		ballPoint=calcBallPos(angle_rot);
/*
		wallCollide=collX(Vector(ball->posX,ball->posY,ball->posZ));
		
		
		float wallx1=wallCollide[0];
		float wallz1=wallCollide[1];
		float wallz2=wallCollide[2];
*/
		//cout<<wallx1<<endl;
		//graphCtrller->ballGr->setPosition(graphCtrller->ballGr->getPosition());
		//graphCtrller->ballGr->setRotation(graphCtrller->ballGr->getAngleBallGr(forceFB1,forceFB2));

		/*if(runs % 10000==0)
		{
		//cout<<"angle_rot"<<endl;
		angle_rot=graphCtrller->ballGr->calculateAngleBallGr(hpos,cpos);
		graphCtrller->ballGr->setAngleBallGr(angle_rot);
		//angle_rot=graphCtrller->ballGr->ggetAngleBallGr();
		//cout<<angle_rot<<endl;
		}*/


		//cigilll
		scoreTime += 1;
		setScoreText(scoreTime);
		//cout<<scoreTime<<endl;


		float previousA;
		Vector forceFB11;
		forceFB11[0]=(npX- hpX)*kpHN + (nvX-hvX)*kdN  ;
		forceFB11[2]=(npZ- hpZ)*kpHN + (nvZ-hvZ)*kdN  ;
		Vector forceFB22;
		forceFB22[0]=(npX- cpX)*kpCN + (nvX-cvX)*kdN  ;
		forceFB22[2]=(npZ- cpZ)*kpCN + (nvZ-cvZ)*kdN  ;

		if(runs % 5==0)
		{
			//cout<<"angle_rot"<<endl;
			angle_rot=graphCtrller->ballGr->calculateAngleBallGr(forceFB2,forceFB1);
			//graphCtrller->ballGr->setAngleBallGr(angle_rot);
			//previousA=angle_rot;
			//graphCtrller->ballGr->setRotation(angle_rot);
			//angle_rot=graphCtrller->ballGr->ggetAngleBallGr();
			//cout<<"angle_rot"<<endl;	
		}	


		ballPoint=calcBallPos(angle_rot);
        
		theta_rotation=angle_rot;
		graphCtrller->ballGr->setAngleBallGr(angle_rot);
		graphCtrller->ballGr2->setAngleBallGr(angle_rot);

		//ballPoint=xwallColl(wallx1,0,wallz1,wallz2,ballPoint,angle_rot);
		graphCtrller->ballGr->setPosition(ballPoint);

		graphCtrller->ballGr2->setPosition(Point(ballPoint[0]+OFFSET,ballPoint[1],ballPoint[2]));

		//cout<<ball->posX<<endl;



		//graphCtrller->ballGr->setPosition(ballPoint);

	

		if(collision==0){
			prevpos[0]=ball->posX;
			prevpos[1]=ball->posY;
			prevpos[2]=ball->posZ;
			//	cout<<prevpos[0]<<"  "<<prevpos[1]<<"   "<<prevpos[2]<<"  ";
		}

		collision=0;
		//Torque=getTorque(graphCtrller->ballGr->getPosition(),wallCollide,angle_rot);


		graphCtrller->cip.getPosition().getValue(cpX, cpY, cpZ);
		graphCtrller->hip.getPosition().getValue(hpX, hpY, hpZ);
		graphCtrller->nip.getPosition().getValue(npX, npY, npZ);
		graphCtrller->nip.getVelocity().getValue(nvX, nvY, nvZ);
		graphCtrller->hip.getVelocity().getValue(hvX, hvY, hvZ);
		graphCtrller->nip.getVelocity().getValue(nvX, nvY, nvZ);
		graphCtrller->ballGr->getPosition().getValue(bbpX, bbpY, bbpZ);
		//cout << "npx: " << npX << " hpx: " << hpX << " " 
		//	 << "npz: " << npZ << " hpz: " << hpZ << endl;





		graphCtrller->ballGr->setPosition(boundaryColl2(angle_rot,ballPoint));//
		graphCtrller->ballGr->getPosition().getValue(bbpX, bbpY, bbpZ);
		graphCtrller->ballGr2->setPosition(Point(bbpX+OFFSET,bbpY,bbpZ));//

		if(bbpX>=(fwx+(boundary[0]-boundaryThickness)-8-INITIAL_TARGET-(TARGET_DIM*0.5)))
		{
			graphCtrller->boardGr->setArrived1(true);
			
		}
		else
		{
			graphCtrller->boardGr->setArrived1(false);

		}
		if(bbpX>=(fwx+(boundary[0]-boundaryThickness)-8-(TARGET_DIM*0.5)))
		{
			graphCtrller->boardGr->setArrived2(true);
			
		}
		else
		{
			graphCtrller->boardGr->setArrived2(false);
			//cout<<" arrived2"<<graphCtrller->boardGr->arrived2<<endl;
		}
	

	//boundaryColl();


#ifdef PRINTANGLE
		if(runs%100==0){
			cout<<"ANGLE   "<<(graphCtrller->ballGr->calculateAngleBallGr(forceFB22,forceFB11))<<"   GETANGLE   "<<graphCtrller->ballGr->ggetAngleBallGr() <<endl;
			cout<<"forcecipx    "<<forceFB22[0]<<"forcecipZ    "<<forceFB22[2]<<endl;
			cout<<"nip   "<<npX<<"  hip  "<<hpX<<"   cip    "<<cpX<<endl;
			cout<<"Z nip   "<<npZ<<"  hip  "<<hpZ<<"   cip    "<<cpZ<<endl;
			cout<<"forcehipx    "<<forceFB11[0]<<"forcehipZ    "<<forceFB11[2]<<endl<<endl;
		}
#endif 

		if(collision==0){
			prevpos2[0]=ball->posX;
			prevpos2[1]=ball->posY;
			prevpos2[2]=ball->posZ;
			/*if(runs%1000==0){
			cout<<prevpos2[0]<<"  "<<collision<<"   "<<prevpos2[2]<<"  "<<endl;
			}*/
			//setWarningText("   ");
		}
		
		collision=0;
		
		if((giveWarningW==1)||(giveWarningB==1))
		{	
			effect->hitWalls = true;
			setWarningText("Fault!!!");
		}
		else 
		{
			effect->hitWalls = false;
			setWarningText("   ");
		}
		//float teta=graphCtrller->ballGr->ggetAngleBallGr();

		f_forceX=-10*G*COEFFICIENT_FRICTION*sign(ball->posX);//graphCtrller->ballGr->getMass()*gravitation*friction*1*sign(ball->velX);
		f_forceZ=-10*G*COEFFICIENT_FRICTION*sign(ball->posZ);//graphCtrller->ballGr->getMass()*gravitation*friction*1*sign(ball->velZ);

		float bpX=ball->posX;
		float bpZ=ball->posZ;

		hhpX=bpX-(ball_width+IP_RADIUS)*0.5*cos(angle_rot);
		hhpZ=bpZ+(ball_width+IP_RADIUS)*0.5*sin(angle_rot);

		hcpX=bpX+(ball_width+IP_RADIUS)*0.5*cos(angle_rot);
		hcpZ=bpZ-(ball_width+IP_RADIUS)*0.5*sin(angle_rot);

		graphCtrller->handleH.setPosition(Vector(hhpX, hhpY, hhpZ));
		graphCtrller->handleC.setPosition(Vector(hcpX, hcpY, hcpZ));
		

		

		/*HÝP*/forceFB1 = Vector(( (hhpX- hpX)*kpHN + (hhvX-hvX)*kdN )  + fhw[0]+ boundaryforceHX + f_forceX , 
			0, 
			( (hhpZ - hpZ)*kpHN + (hhvZ-hvZ)*kdN ) + fhw[2] + boundaryforceHZ + f_forceZ );
		fOnHip=forceFB1;

		/*CÝP*/forceFB2 = Vector(( (hcpX- cpX)*kpCN + (hcvX-cvX)*kdN )  + fcw[0] +/*-*/ boundaryforceCX + f_forceX , 
			0,
			( (hcpZ- cpZ)*kpCN + (hcvZ-cvZ)*kdN )  + fcw[2] /*-*/+boundaryforceCZ +f_forceZ );

		fOnCip=forceFB2;
		
		graphCtrller->ballGr->setAngleBallGr(graphCtrller->ballGr->calculateAngleBallGr(forceFB2,forceFB1));//(forceFB1,forceFB2));//cigil
		graphCtrller->ballGr2->setAngleBallGr(graphCtrller->ballGr->calculateAngleBallGr(forceFB2,forceFB1));
		Vector forceReac;
		forceReac[0]=fhw[0]+ boundaryforceHX+fcw[0] +boundaryforceCX; 
		forceReac[2]=fhw[2]+ boundaryforceHZ+fcw[2] +boundaryforceCZ;
		
	//if(runs%1000==0)
	//{
	//		cout<<"Hreacx  "<<fhw[0]+boundaryforceHX<<"  Freacz  "<< fhw[2]+boundaryforceHZ<<endl;
    //		cout<<"CreacX  "<<fcw[0]+boundaryforceCX<<"  Freacz  "<< fcw[2]+boundaryforceCZ<<endl;
    //	}

		//#ifdef FORCEVECTOR_ON

		if(runs%25==0)
		{
			Vector Fnnnet;
			Fnnnet=forceFB1+forceFB2;
			float mag3=sqrt(Fnnnet[0]*Fnnnet[0]+Fnnnet[2]*Fnnnet[2]);
			graphCtrller->fvector->setVectorDepth(mag3*5);

			float forceangle=0.0f;
			if((Fnnnet[2]!=0)&&(Fnnnet[0]!=0))
			{
				forceangle=atan2(Fnnnet[0],Fnnnet[2]);


			}else if((Fnnnet[0]>0)&&(Fnnnet[2]==0))
			{
				forceangle=PI/2;
			}else if((Fnnnet[0]<0)&&(Fnnnet[2]==0))
			{
				forceangle=-PI/2;
			}else if((Fnnnet[0]==0)&&(Fnnnet[2]<0))
			{
				forceangle=0.0f;
			}else if((Fnnnet[0]==0)&&(Fnnnet[2]>0))
			{
				forceangle=PI;
			}
			graphCtrller->fvector->setVectorAngle(forceangle);

			graphCtrller->fvector->setVectorT( Vector( (ball->posX),ball->posY, (ball->posZ)) );

			graphCtrller->fvector->correctT();
		}


		if(runs%25==0)
		{
			float mag2=sqrt(forceFB2[0]*forceFB2[0]+forceFB2[2]*forceFB2[2]);
			graphCtrller->cvector->setVectorDepth(mag2*5);

			float forceangle2=0.0f;
			if((forceFB2[2]!=0)&&(forceFB2[0]!=0))
			{
				forceangle2=atan2(forceFB2[0],forceFB2[2]);
			}else if((forceFB2[0]>0)&&(forceFB2[2]==0))
			{
				forceangle2=PI/2;
			}else if((forceFB2[0]<0)&&(forceFB2[2]==0))
			{
				forceangle2=-PI/2;
			}else if((forceFB2[0]==0)&&(forceFB2[2]<0))
			{
				forceangle2=0.0f;
			}else if((forceFB2[0]==0)&&(forceFB2[2]>0))
			{
				forceangle2=PI;
			}


			graphCtrller->cvector->setVectorAngle(forceangle2);

			graphCtrller->cvector->setVectorT( Vector( hcpX,hcpY, hcpZ) );

			graphCtrller->cvector->correctT();
		}

	
		if(runs%25==0){
		float mag22=sqrt(forceReac[0]*forceReac[0]+forceReac[2]*forceReac[2]);
		graphCtrller->reacV->setVectorDepth(mag22*20);

		//if(runs%1000==0)
		//{
		//cout<<mag22<<endl<<endl;
		//}

		float forceangle4=0.0f;
		if((forceReac[2]!=0)&&(forceReac[0]!=0))
		{
		forceangle4=atan2(forceReac[0],forceReac[2]);
		}else if((forceReac[0]>0)&&(forceReac[2]==0))
		{
		forceangle4=-PI/2;//cigil
		}else if((forceReac[0]<0)&&(forceReac[2]==0))
		{
		forceangle4=PI/2;//cigil
		}else if((forceReac[0]==0)&&(forceReac[2]<0))
		{
		forceangle4=0.0f;
		}else if((forceReac[0]==0)&&(forceReac[2]>0))
		{
		forceangle4=PI;
		}


		graphCtrller->reacV->setVectorAngle(forceangle4);

		graphCtrller->reacV->setVectorT( Vector( (ball->posX),ball->posY, (ball->posZ)) ) ;

		graphCtrller->reacV->correctT();

		}


	

		if(runs%25==0){
			float mag1=sqrt(forceFB1[0]*forceFB1[0]+forceFB1[2]*forceFB1[2]);
			graphCtrller->hvector->setVectorDepth(mag1*5);

			float forceangle1=0.0f;
			if((forceFB1[2]!=0)&&(forceFB1[0]!=0))
			{
				forceangle1=atan2(forceFB1[0],forceFB1[2]);
			}else if((forceFB1[0]>0)&&(forceFB1[2]==0))
			{
				forceangle1=PI/2;
			}else if((forceFB1[0]<0)&&(forceFB1[2]==0))
			{
				forceangle1=-PI/2;
			}else if((forceFB1[0]==0)&&(forceFB1[2]<0))
			{
				forceangle1=0.0f;
			}else if((forceFB1[0]==0)&&(forceFB1[2]>0))
			{
				forceangle1=PI;
			}


			graphCtrller->hvector->setVectorAngle(forceangle1);

			graphCtrller->hvector->setVectorT( Vector( hhpX,hhpY, hhpZ) );

			graphCtrller->hvector->correctT();
		}

	


		//#endif


		/*
		if(runs%25==0)
		{
			prevHZb=boundaryforceHZ;
			prevHXb=boundaryforceHX;
			prevCZb=boundaryforceCZ;
			prevCXb=boundaryforceCX;

			prevHZw=fhw[2];prevHXw=fhw[0];
			prevCZw=fcw[2];prevCXw=fcw[0];

		}
	*/





	}
#ifdef PRINT_FORCE
	if(runs%50==0)
	{
		cout << "HIP -- WALL: "	<< fhw[0] << " " << fhw[2] << endl
			<< "boundary     "<<boundaryforceHX<<"   "<<boundaryforceHZ<<endl
			// << " \tspring: "		<<-( (npX- hpX)*kpHN + (nvX-hvX)*kdN ) << " " <<-( (npZ- hpZ)*kpHN + (nvZ-hvZ)*kdN )  << endl
			<< " \tTOTAL		"	<< forceFB1[0] << " " << forceFB1[2] << endl << endl 

			<< "CIP -- WALL: "	<< fcw[0] << " " << fcw[2] << endl
			<< "boundary     "<<boundaryforceCX<<"   "<<boundaryforceCZ<<endl
			//<< " \tspring: "		<<-( (npX- cpX)*kpCN + (nvX-cvX)*kdN ) << " " <<-( (npZ- cpZ)*kpCN + (nvZ-cvZ)*kdN ) << endl 
			<< " \tTOTAL		"	<< forceFB2[0] << " " << forceFB2[2] << endl;


		//cout<< "NIP Pos: " << npX << " HIP Pos: " << hpX << endl << endl;
	}
#endif

	float fMag = forceFB1.length();
	if( fMag > FORCE_LIMIT)
	{
		forceFB1 = (forceFB1 / fMag) * FORCE_LIMIT ;
	}
	//else if (fMag < 0.01)
	//{
	//	forceFB1[0] = 0;
	//	forceFB1[1] = 0;
	//	forceFB1[2] = 0;
	//}
	forceFB1 = F_SCALE_FACTOR * forceFB1;

	fMag = forceFB2.length();
	if( fMag > FORCE_LIMIT) 
	{
		forceFB2 = (forceFB2 / fMag) * FORCE_LIMIT ;
	}
	//else if (fMag < 0.01)
	//{
	//	forceFB2[0] = 0;
	//	forceFB2[1] = 0;
	//	forceFB2[2] = 0;
	//}
	forceFB2 = F_SCALE_FACTOR * forceFB2;




	//graphCtrller->ballGr->setRotation(angle_rot);
	//cout<<angle_rot<<"  previousA  "<<previousA<<"  ANGLEBALLGR  "<<graphCtrller->ballGr->angleBallGr<<endl;
	//if (runs%10000==0){
	//cout<< "hip: " << forceFB1[0] << " " <<  forceFB1[2] << endl;
	//cout<< "cip: " << forceFB2[0] << " " <<  forceFB2[2] << endl;
	//cout<< "hip total: " << forceFB1.length()<< "  cip total: " <<  forceFB2.length() << endl << endl;
	//}
	//cout<<collision<<endl;

	//	cout<<collision<<endl;


	hduVector3Dd position;
	HDdouble kStiffness;

	hdMakeCurrentDevice(effect->hHD1);
	hdGetDoublev(HD_CURRENT_POSITION, position);
	hdGetDoublev(HD_NOMINAL_MAX_STIFFNESS, &kStiffness);




	//cigil 1 ekim
	/* AYSE: Render two simple horizontal planes to contstrain user movement to x-z plane. */
	/* the planes are not stiff */
	//if(runs%1000==0)
	//cout<<position[1]<<endl;

	//cout<<position[0]<<"     "<<position[2]<<endl;
if((position[0]<=-1)&&(position[0]>=1)&&(position[2]<-89.2142)&&(position[2]>-86.0142))
{
if (position[1] < 0) // ground
	{
		forceFB1[1] = kStiffness * (0 - position[1]);
	}
	else if (position[1] > 20) // ceiling
	{
		forceFB1[1] = kStiffness * (50 - position[1]);
	}
}else  
{
	forceFB1[1] = 0.0f;
}

	// send all forces to device
	HDdouble fToD1[3] = { forceFB1[0], forceFB1[1], forceFB1[2] };


	effect->PreventWarmMotors( hduVector3Dd(fToD1[0], fToD1[1], fToD1[2]) );  
	hdSetDoublev(HD_CURRENT_FORCE, fToD1);

	//if (runs%1000==0)
	//printf("%f  %f\n",fToD1[0],fToD1[2]);

	hdScheduleSynchronous(QueryMotorTemp, aMotorTemp,
		HD_DEFAULT_SCHEDULER_PRIORITY);


	hdMakeCurrentDevice(effect->hHD2);
	hdGetDoublev(HD_CURRENT_POSITION, position);
	hdGetDoublev(HD_NOMINAL_MAX_STIFFNESS, &kStiffness);
	//cout<<position[0]<<"     "<<position[2]<<endl;
	
	//cigil 1 ekim
if((position[0]<=-1)&&(position[0]>=1)&&(position[2]<-89.2142)&&(position[2]>-86.0142))

{
	
if (position[1] < 0) // ground -70 -60
	{
		forceFB2[1] = kStiffness * (0 - position[1]);
	}
	else if (position[1] > 20) // ceiling  -50
	{
		forceFB2[1] = kStiffness * (50 - position[1]);
	}
}
else  
{//cout<<"here  "<<position[0]<<"     "<<position[2]<<endl;
	forceFB2[1] = 0.0f;
}

	// send all forces to device
	HDdouble fToD2[3] = { forceFB2[0], forceFB2[1], forceFB2[2] };


	effect->PreventWarmMotors( hduVector3Dd(fToD2[0], fToD2[1], fToD2[2]) );  
	hdSetDoublev(HD_CURRENT_FORCE, fToD2);

	//if (runs%1000==0)
	//printf("%f  %f\n\n",fToD2[0],fToD2[2]);

	hdScheduleSynchronous(QueryMotorTemp, aMotorTemp,
		HD_DEFAULT_SCHEDULER_PRIORITY);


	//HDdouble fToD[3] = { (*computedForces)[0], (*computedForces)[1], (*computedForces)[2] };
	runs++;	cntF++;



	if(/*(firstTime == false) && */((effect->keepHapticLoop) == true))
	{

		float hpX, hpY, hpZ, npX, npY, npZ;
		float hvX,hvY,hvZ,cvX,cvY,cvZ;
		graphCtrller->hip.getPosition().getValue(hpX, hpY, hpZ);
		graphCtrller->hip.getVelocity().getValue(hvX,hvY,hvZ);
		graphCtrller->cip.getPosition().getValue(cpX, cpY, cpZ);
		graphCtrller->cip.getVelocity().getValue(cvX,cvY,cvZ);



		if (effect->update_stylus_position(myP1) == -1)
			return HD_CALLBACK_DONE;
// record 
//#ifdef RECORD_DATA

	/*	if(effect->userFlaggedMoment)
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
	*/
		dataRec->runsCnt->push_back(runs);
		dataRec->trialID->push_back(effect->curTrial);

		dataRec->timeScore->push_back(scoreTime);
		//dataRec->runsInPits->push_back(runsInPits);
		//dataRec->runsOnTargets->push_back(runsOnTargets);
		//dataRec->ticksInPits->push_back(ticksInPits);
		//dataRec->ticksOnTargets->push_back(ticksOnTargets);
		
		dataRec->ballP->push_back(Vector(ball->posX, 0, ball->posZ));		
		dataRec->ballV->push_back(Vector(ball->velX, 0, ball->velZ));	
		dataRec->ballA->push_back(Vector(ball->accX, 0, ball->accZ));
		dataRec->theta->push_back(theta_rotation);//ball->angle);
		//cout<<theta_rotation<<"   "<<ball->angle<<endl;
		dataRec->cipP->push_back(Vector(cpX,0,cpZ));
		dataRec->cipV->push_back(Vector(cvX,0,cvZ));
		dataRec->hipP->push_back(Vector(hpX,0,hpZ));
		dataRec->hipV->push_back(Vector(hvX,0,hvZ));
		

		dataRec->fOnCIP->push_back(fOnCip);
		dataRec->fOnHIP->push_back(fOnHip);


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

		/*
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
		*/
//#endif


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

//#ifdef RECORD_DATA
	dataRec->writeMeToFile();
//#endif

#ifdef RECORD_FORCE
	//	faRec->writeFile();
#endif

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

	if(myStylusFlag == false)
	{
		float bpX, bpY, bpZ, cpX, cpY, cpZ, hpX, hpY, hpZ;

		graphCtrller->cip.getPosition().getValue(cpX, cpY, cpZ);
		graphCtrller->hip.getPosition().getValue(hpX, hpY, hpZ);


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

		if (isGameOver() == 1)
		{
			if (effect->curTrial == NUM_TRIALS_PER_COND)
			{
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
Vector xwallColl(float wallx1,float wallx2,float wallz1,float wallz2,Vector ballPoint,float angle)
{  //cout<<wallx1<<" x2  "<<wallx2<<"  z1  "<<wallz1<<"	z2	"<<wallz2<<endl;
	Vector wall_force;
	Vector wallDisp;
	float cpX, cpY, cpZ;
	float hpX, hpY, hpZ;
	float bpX, bpY, bpZ;
	float npX,npY,npZ;
	float bvX,bvY,bvZ;
	float baX,baY,baZ;
	float ballRadius;
	float accX,accY,accZ;
	float Bpnts[49][4];
	float dist[4];
	float bppX;
	float bppZ;
	float bppY;
	float answer;
	float answerz;
	float zcontrol;
	answer=ballPoint[0];
	answerz=ballPoint[2];
	float angle1;
	float beta;
	float wallx11=wallx1+0.5*wallWidth;
	float wallx12=wallx1-0.5*wallWidth;
	//cout<<wallx11<<"  wallx12  "<<wallx12<<endl; 
	graphCtrller->cip.getPosition().getValue(cpX, cpY, cpZ);
	graphCtrller->hip.getPosition().getValue(hpX, hpY, hpZ);
	graphCtrller->nip.getPosition().getValue(npX, npY, npZ);
	graphCtrller->ballGr->getPosition().getValue(bppX, bppY, bppZ);

	bpX=ballPoint[0];
	bpY=ballPoint[1];
	bpZ=ballPoint[2];



	graphCtrller->ballGr->getVelocity().getValue(bvX,bvY,bvZ);
	graphCtrller->ballGr->getAcceleration().getValue(baX,baY,baZ);
	ballRadius=graphCtrller->ballGr->getRadius();
	beta=atan2(ball_depth,ball_width);
	//cout<<beta<<"   angle "<<angle<<endl;
	angle1=1*(PI-(beta-angle));
	float teta=atan2(ball_depth*0.25,ball_width*0.5);
	float angle2=1*(angle+beta+PI);
	float angle3=(2*PI-(beta-angle));
	float angle4=beta+angle;
	float angle5=angle+0.5*PI;
	float angle6=angle+1.5*PI;
	float angle7=angle;
	float angle8=angle+1*PI;
	float angle9=2*PI-(teta-angle);
	float angle10=teta+angle;
	float angle11=PI-(teta-angle);
	float angle12=PI+(teta+angle);

	float bnow[49][4];
	float lenArray[49][4];

	float h2=sqrt((ball_depth*0.25)*(ball_depth*0.25)*(ball_width*0.5)*(ball_width*0.5));
	float h=sqrt(ball_width*ball_width+ball_depth*ball_depth)*0.5;


	createLen(lenArray);


	for(int j=1;j<=48;j++)
	{
		Bpnts[j][1]=bpX+(lenArray[j][1]*cos(angle))+(lenArray[j][3]*sin(angle));
		Bpnts[j][3]=bpZ+(lenArray[j][3]*cos(angle))-(lenArray[j][1]*sin(angle));
		bnow[j][1]=bppX+(lenArray[j][1]*cos(angle))+(lenArray[j][3]*sin(angle));
		bnow[j][3]=bppZ+(lenArray[j][3]*cos(angle))-(lenArray[j][1]*sin(angle));
	}



	float aarray[13];
	aarray[1]=angle1;aarray[2]=angle2;aarray[3]=angle3;aarray[4]=angle4;aarray[5]=angle5;aarray[6]=angle6;
	aarray[7]=angle7;aarray[8]=angle8;aarray[9]=angle9;aarray[10]=angle10;aarray[11]=angle11;aarray[12]=angle12;

	float harray[13];
	harray[1]=h;harray[2]=h;harray[3]=h;harray[4]=h;harray[5]=ball_depth*0.5;harray[6]=ball_depth*0.5;
	harray[7]=ball_width*0.5;harray[8]=ball_width*0.5;harray[9]=h2;harray[10]=h2;harray[11]=h2;harray[12]=h2;
	harray[13]=sqrt(pow(ball_width*0.5,2)+pow(ball_depth/8,2));
	harray[14]=harray[13];	harray[15]=harray[13];	harray[16]=harray[13];
	harray[17]=sqrt(pow(ball_width*0.5,2)+pow((-ball_depth/8)*3,2));
	harray[18]=harray[17];harray[19]=harray[17];harray[20]=harray[17];
	harray[21]=sqrt(pow(-ball_width/4,2)+pow(-ball_depth*0.5,2));
	harray[22]=harray[21];harray[23]=harray[21];harray[24]=harray[21];

#ifdef PRINT_CORNERS

	if(runs%10000==0){

		cout<<bpZ<<  "zler  "<<Bpnts[1][3]<<"   2  "<<Bpnts[2][3]<<"  3  "<<Bpnts[3][3]<<"  4  "<<Bpnts[4][3]<<endl;

		cout<<bpX<<"  xler  "<<Bpnts[1][1]<<"   2  "<<Bpnts[2][1]<<"  3  "<<Bpnts[3][1]<<"  4  "<<Bpnts[4][1]<<endl<<endl;

		//cout<<angle<<endl<<endl;
	}
#endif
	int contactP=0;
	int coll=0;
	collision=0;
	float dista[12];
	int jj=0;
	float wd=0;//wallDepth/18;//z
	float dw=0;//wallWidth/18;
	//coefficients of line1 and line2
	float a1=((wallz1+0.5*wd)-(wallz2-0.5*wd));
	float b1=- ( (wallx12-0.5*dw)-(wallx11+0.5*dw) );
	float c1=(wallx11+0.5*dw)*(a1)+(wallz2-0.5*wd)*b1;
	float a2=-((wallz1+0.5*wd)-(wallz2-0.5*wd));
	float b2=-((wallx12-0.5*dw)-(wallx11+0.5*dw));
	float c2=(wallx11+0.5*dw)*(a2)+(wallz1+0.5*wd)*b2;
	//cout<<wallx11<<"  x2	"<<wallx12<<endl;
	//cout<<wallz1<< "  z2	"<<wallz2<<endl;

#ifdef HIP_CONTROL_ON	
	float hipp[1][4];
	hipp[0][1]=hpX;   hipp[0][2]=hpY;
	hipp[0][3]=hpZ;
	checkHip(hipp);
	graphCtrller->hip.setPosition(Point(hipp[0][1],hipp[0][2],hipp[0][3]));
#endif
	for (int i=1;i<=48;i++){
		if((Bpnts[i][1]<(boundary[0]+fwx-boundaryThickness))&& (Bpnts[i][1]> fwx-(boundary[0]-boundaryThickness)))
		{
			if((Bpnts[i][1]<=(wallx1+0.5*wallWidth+0.5*dw))&&(Bpnts[i][1]>=(wallx1-0.5*wallWidth-0.5*dw))&&(Bpnts[i][3]<=(wallz1+0.5*wd))&&(Bpnts[i][3]>=(wallz2-(0.5*wd))))
			{  //cout<<Bpnts[i][1]<<"  z   "<<Bpnts[i][3]<<endl;
				collision=1;
				coll=i;
				//if (abs(Bpnts[i][1]-wallx1+0.5*wallWidth+0.5)> abs(Bpnts[i][1]-(wallx1-0.5*wallWidth-0.5) ) )
				//{}
			}
			//else
			//{
			//collision=0;
			//}
		}
	}
	int rx=0;
	int rz=0;

	//points at the regions
	int region1=0;
	int region2=0;
	int region3=0;
	int region4=0;

	int collz=0;

	for (int i=1;i<=48;i++){
		if((Bpnts[i][1]<(fwx+boundary[0]-boundaryThickness))&& (Bpnts[i][1]> fwx-(boundary[0]-boundaryThickness)))
		{
			if((Bpnts[i][1]<=wallx1+0.5*wallWidth+0.5*dw)&&(Bpnts[i][1]>=wallx1-0.5*wallWidth-0.5*dw)&&(Bpnts[i][3]<=wallz1+0.5*wd)&&(Bpnts[i][3]>=(wallz2-0.5*wd)))
			{ if((bnow[i][1]<=wallx1+0.5*wallWidth+0.5*dw)&&(bnow[i][1]>=wallx1-0.5*wallWidth-0.5*dw)&&(bnow[i][3]<=wallz1+0.5*wd)&&(bnow[i][3]>=(wallz2-0.5*wd)))
			{
				//cout<<Bpnts[i][1]<<"  z   "<<Bpnts[i][3]<<endl;
				collision=1;
				float ox=Bpnts[i][1];
				float oz=Bpnts[i][3];

				float nox=bnow[i][1];
				float noz=bnow[i][3];


				if( ( ((a1*ox)+(b1*oz)) < c1 )&& ( ((a2*ox)+(b2*oz)) > c2 ) &&( ((a1*nox)+(b1*noz)) < c1 )&& ( ((a2*nox)+(b2*noz)) > c2 ))
				{   
					region1=region1+1;

					//cout<<"region1   "<<region1<<endl;
					//rx=1;
					collision=1; coll=i;
					dista[i]=abs(Bpnts[i][1]-(wallx1-0.5*wallWidth-0.5));

				} 
				else if( ( ((a1*ox)+(b1*oz)) > c1 )&& ( ((a2*ox)+(b2*oz)) < c2 ) && ( ((a1*nox)+(b1*noz)) > c1 )&& ( ((a2*ox)+(b2*noz)) < c2 ))
				{   //rx=2;
					region2=region2+1;
					collision=1;coll=i;
					//cout<<"region2"<<endl;
					dista[i]=abs(Bpnts[i][1]-(wallx1+0.5*wallWidth+0.5));

				} 
				else if( ( ((a1*ox)+(b1*oz)) > c1 )&& ( ((a2*ox)+(b2*oz)) > c2 ) &&( ((a1*nox)+(b1*noz)) > c1 )&& ( ((a2*nox)+(b2*noz)) > c2 ))
				{   //rz=3;

					region3=region3+1;
					//cout<<"region 3   "<<region3<<endl;
					collision=1;collz=i;
					dista[i]=abs(Bpnts[i][3]-(wallz1+0.5));

				} 
				else if( ( ((a1*ox)+(b1*oz)) < c1 )&& ( ((a2*ox)+(b2*oz)) < c2 )&&( ((a1*nox)+(b1*noz)) < c1 )&& ( ((a2*nox)+(b2*noz)) < c2 )  )
				{  // rz=4;
					region4=region4+1;
					//cout<<"region4"<<endl;
					collision=1;collz=i;
					dista[i]=abs(Bpnts[i][3]-(wallz2-0.5));


				} 

				else if ( (((a1*ox)+(b1*oz)) <c1 )&& ( ((a2*ox)+(b2*oz)) == c2 )&&(((a1*nox)+(b1*noz)) <c1 )&& ( ((a2*nox)+(b2*noz)) == c2 ) )
				{  //rx=2;rz=3;
					region1=region1+1; region4=region4+1;
					//cout<<"region 2 3"<<endl;
					collision=1;coll=i;collz=i;
					dista[i]=sqrt(((Bpnts[i][3]-(wallz1+0.5))*(Bpnts[i][3]-(wallz1+0.5)))+(Bpnts[i][1]-(wallx1+0.5*wallWidth+0.5))*(Bpnts[i][1]-(wallx1+0.5*wallWidth+0.5)));


				}
				else if ( (((a1*ox)+(b1*oz)) >c1 )&& ( ((a2*ox)+(b2*oz)) == c2 )&& (((a1*nox)+(b1*noz)) >c1 )&& ( ((a2*nox)+(b2*noz)) == c2 ) )
				{//rx=1;rz=4;
					region3=region3+1; region2=region2+1;
					collision=1;coll=i;collz=i;
					//cout<<"region 1 4"<<endl;
					dista[i]=sqrt(((Bpnts[i][3]-(wallz2-0.5))*(Bpnts[i][3]-(wallz2-0.5)))+(Bpnts[i][1]-(wallx1-0.5*wallWidth-0.5))*(Bpnts[i][1]-(wallx1-0.5*wallWidth-0.5)));


				}
				else if(( ((a1*ox)+(b1*oz)) ==c1 )&& ( ((a2*ox)+(b2*oz)) > c2 )&&( ((a1*nox)+(b1*noz)) ==c1 )&& ( ((a2*nox)+(b2*noz)) > c2 ) )
				{  // rx=1;rz=3;
					//cout<<"region 1 3"<<endl;
					region1=region1+1; 
					region3=region3+1;
					collision=1;coll=i;collz=i;
					dista[i]=sqrt(((Bpnts[i][3]-(wallz1+0.5))*(Bpnts[i][3]-(wallz1+0.5)))+(Bpnts[i][1]-(wallx1-0.5*wallWidth-0.5))*(Bpnts[i][1]-(wallx1-0.5*wallWidth-0.5)));


				}
				else if ( (((a1*ox)+(b1*oz)) ==c1 )&& ( ((a2*ox)+(b2*oz)) < c2 )&&(((a1*nox)+(b1*noz)) ==c1 )&& ( ((a2*nox)+(b2*noz)) < c2 ) )
				{   //rx=2; rz=4;
					region2=region2+1; 
					region4=region4+1;
					//cout<<"region 2 4"<<endl;
					collision=1;coll=i;collz=i;
					dista[i]=sqrt(((Bpnts[i][3]-(wallz2-0.5))*(Bpnts[i][3]-(wallz2-0.5)))+(Bpnts[i][1]-(wallx1+0.5*wallWidth+0.5))*(Bpnts[i][1]-(wallx1+0.5*wallWidth+0.5)));



				}
				else if ( (((a1*ox)+(b1*oz)) ==c1 )&& ( ((a2*ox)+(b2*oz)) < c2 )&&(((a1*nox)+(b1*noz)) ==c1 )&& ( ((a2*nox)+(b2*noz)) == c2 ) )
				{
					region2=region2+1;
					region4=region4+1;
					region3=region3+1;
					region1=region1+1;

					collision=1; coll=i; collz=i;
				}


			}
			}
		}
	}

	if(collision==1){
	//since it is wall collision forcetypes=2
	crunsw=crunsw+1;
	setForces2(region1,region2,region3,region4,2);//18 eylul
	}
	else
	{
		fhw[0]=0.0f;
		fcw[0]=0.0f;
		fhw[2]=0.0f;
		fcw[2]=0.0f;

		fhw_arr[0][0]=0.0f;
		fcw_arr[0][0]=0.0f;
		fhw_arr[0][2]=0.0f;
		fcw_arr[0][2]=0.0f;

		fhw_arr[1][0]=0.0f;
		fcw_arr[1][0]=0.0f;
		fhw_arr[1][2]=0.0f;
		fcw_arr[1][2]=0.0f;
		graphCtrller->wall1->setHit(false);


		
		crunsw=0;
	}

	if((collision==1)&&(wall_hit==1))
	{
		graphCtrller->wall1->setHit(true);
		giveWarningW=1;
	}
	else if((collision==1)&&(wall_hit==2))
	{
		graphCtrller->wall2->setHit(true);
		giveWarningW=1;
	}
		else if((collision==1)&&(wall_hit==3))
	{
		graphCtrller->wall3->setHit(true);
		giveWarningW=1;
	}
	else if(collision==0)
	{
		graphCtrller->wall1->setHit(false);
		graphCtrller->wall2->setHit(false);
		graphCtrller->wall3->setHit(false);
		giveWarningW=0;
	}

	//cout<<"wall  "<<region1<<" r2 "<<region2<<" r3 "<<region3<<" r4 "<<region4<<endl;


	if((region1>0)&&(region1>region2))
	{
		rx=1;
	}
	else if((region2>0)&&(region2>region1))
	{
		rx=2;
	}

	if((region3>0)&&(region3>region4))
	{
		rz=3;
	}
	else if((region4>0)&&(region4>region3))
	{
		rz=4;
	}





	for(int j=1;j<=48;j++)
	{

		if ((rx==1)&&(coll==j)&&(collision==1))
		{
			ballPoint[0]=(wallx1-0.5*wallWidth)+((-lenArray[j][1]*cos(angle))+(-lenArray[j][3]*sin(angle)));//-((lenArray[j][1]*cos(0.0f))+(lenArray[j][3]*sin(0.0f)));//-harray[j]*cos(aarray[j]);
			//ballPoint[1]=ballPoint[1];
			//ballPoint[2]=ballPoint[2];
			//cout<<"region1"<<endl<<endl;
		}
		else if ((rx==2)&&(coll==j)&&(collision==1))
		{
			ballPoint[0]=wallx1+0.5*wallWidth+((-lenArray[j][1]*cos(angle))+(-lenArray[j][3]*sin(angle)));//-((lenArray[j][1]*cos(0.0f))+(lenArray[j][3]*sin(0.0f)));//-harray[j]*cos(aarray[j]);
			//ballPoint[1]=ballPoint[1];
			//ballPoint[2]=ballPoint[2];
			//cout<<"region2"<<endl<<endl;
		}
		else if ((rz==3)&&(collz==j)&&(collision==1))
		{
			//ballPoint[0]=ballPoint[0];
			//ballPoint[1]=ballPoint[1];
			ballPoint[2]=(wallz1)+((-lenArray[j][3]*cos(angle))-(-lenArray[j][1]*sin(angle)));//-((lenArray[j][3]*cos(0.0f))-(lenArray[j][1]*sin(0.0f)));//+harray[j]*sin(aarray[j]);
			//cout<<"region3"<<endl<<endl;
		}
		else if ((rz==4)&&(collz==j)&&(collision==1))
		{
			//ballPoint[0]=ballPoint[0];
			//ballPoint[1]=ballPoint[1];
			ballPoint[2]=(wallz2)+((-lenArray[j][3]*cos(angle))-(-lenArray[j][1]*sin(angle)));//-((lenArray[j][3]*cos(0.0f))-(lenArray[j][1]*sin(0.0f)));//+harray[j]*sin(aarray[j]);
			//cout<<"region4"<<endl<<endl;
		}
		else if((rx==1)&&(rz==3)&&(coll==j)&&(collz==j)&&(collision==1))
		{   
			ballPoint[0]=(wallx1-0.5*wallWidth)+((-lenArray[j][1]*cos(angle))+(-lenArray[j][3]*sin(angle)));//-((lenArray[j][1]*cos(0.0f))+(lenArray[j][3]*sin(0.0f)));//-harray[j]*cos(aarray[j]);
			//ballPoint[1]=ballPoint[1];
			ballPoint[2]=(wallz1)+((-lenArray[j][3]*cos(-angle))-(-lenArray[j][1]*sin(-angle)));//-((lenArray[j][3]*cos(0.0f))-(lenArray[j][1]*sin(0.0f)));//+harray[j]*sin(aarray[j]);
			//cout<<"region1-3"<<endl<<endl;

		}
		else if((rx==1)&&(rz==4)&&(coll==j)&&(collision==1)&&(collz==j))
		{   
			ballPoint[0]=(wallx1-0.5*wallWidth)+((-lenArray[j][1]*cos(angle))+(-lenArray[j][3]*sin(angle)));//-((lenArray[j][1]*cos(0.0f))+(lenArray[j][3]*sin(0.0f)));//-harray[j]*cos(aarray[j]);
			//ballPoint[1]=ballPoint[1];
			ballPoint[2]=(wallz2)+((-lenArray[j][3]*cos(angle))-(-lenArray[j][1]*sin(angle)));//-(lenArray[j][3]*cos(0.0f))-(lenArray[j][1]*sin(0.0f));//+harray[j]*sin(aarray[j]);
			//cout<<"region1-4"<<endl<<endl;

		}
		else if((rx==2)&&(rz==3)&&(coll==j)&&(collision==1)&&(collz==j))
		{   
			ballPoint[0]=(wallx1+0.5*wallWidth)+((-lenArray[j][1]*cos(angle))+(-lenArray[j][3]*sin(angle)));//-((lenArray[j][1]*cos(0.0f))+(lenArray[j][3]*sin(0.0f)));//-harray[j]*cos(aarray[j]);
			//ballPoint[1]=ballPoint[1];
			ballPoint[2]=(wallz1)+((-lenArray[j][3]*cos(angle))-(-lenArray[j][1]*sin(angle)));//-((lenArray[j][3]*cos(0.0f))-(lenArray[j][1]*sin(0.0f)));//+harray[j]*sin(aarray[j]);
			//cout<<"region2-3"<<endl<<endl;

		}
		else if((rx==2)&&(rz==4)&&(coll==j)&&(collision==1)&&(collz==j))
		{   
			ballPoint[0]=(wallx1+0.5*wallWidth)+((-lenArray[j][1]*cos(angle))+(-lenArray[j][3]*sin(angle)));//-((lenArray[j][1]*cos(0.0f))+(lenArray[j][3]*sin(0.0f)));//-harray[j]*cos(aarray[j]);
			//ballPoint[1]=bpY;//ballPoint[1];
			ballPoint[2]=(wallz2)+((-lenArray[j][3]*cos(angle))-(-lenArray[j][1]*sin(angle)));//-((lenArray[j][3]*cos(0.0f))-(lenArray[j][1]*sin(0.0f)));//+harray[j]*sin(aarray[j]);
			//cout<<"region2-4"<<endl<<endl;

		}


		else if((collision==0)&&(rx==0)&&(rz==0)&&(coll==0)&&(collz=0))
		{
			//ballPoint=ballPoint;
			//cout<<"no coll"<<endl<<endl;
			ballPoint[0]=bpX;ballPoint[1]=bpY;ballPoint[2]=bpZ;
		}

	}



	//	if(runs%100==0){
	//cout<<"region1  "<<region1<<"  region2  "<<region2<<"  region3  "<<region3<<"  region4 "<<region4<<endl;
	//	}
	if ((region1!=0)&&(region1>region2))
	{
		if((bvX > 0)) {
			bvX = 0.0f;baX=0.0f;
			graphCtrller->ballGr->setVelocity(Vector(bvX,bvY,bvZ));
			graphCtrller->ballGr->setAcceleration(Vector(baX,baY,baZ));
		}

	}else if((region2!=0)&&(region2>region1))
	{
		if((bvX < 0)) 
		{
			bvX = 0.0f;baX=0.0f;
			graphCtrller->ballGr->setVelocity(Vector(bvX,bvY,bvZ));
			graphCtrller->ballGr->setAcceleration(Vector(baX,baY,baZ));
		}
	}

	if ((region3!=0)&&(region3>region4))
	{
		if((bvZ < 0)) {
			bvZ = 0.0f;baZ=0.0f;
			graphCtrller->ballGr->setVelocity(Vector(bvX,bvY,bvZ));
			graphCtrller->ballGr->setAcceleration(Vector(baX,baY,baZ));
		}
	}
	else if ((region4!=0)&&(region4>region3))
	{
		if((bvZ > 0)) {
			bvZ = 0.0f;baZ=0.0f;
			graphCtrller->ballGr->setVelocity(Vector(bvX,bvY,bvZ));
			graphCtrller->ballGr->setAcceleration(Vector(baX,baY,baZ));
		}
	}


	ballPoint[1]=bpY;//ballPoint[1];
	if(coll==0)
	{ballPoint[0]=bpX;//ballPoint[0];
	}
	if(collz==0)
	{ballPoint[2]=bpZ;//ballPoint[2];
	}
	/*
	if (runs%10==0){
	//cout<<"collision point		"<<coll<<endl;
	//cout<<"region1 "<<region1<<"	r2	"<<region2<<"	r3	"<<region3<<"	r4	"<<region4<<endl;
	cout<<rx<<"	z	"<<rz<<endl<<endl;
	//cout<<ballPoint[0]<<endl<<endl;
	}*/
	//graphCtrller->ballGr->setPosition(ballPoint);
	return ballPoint;
}

int sign(float number){
	int answer;
	if (number>=0){
		answer=1;}
	else {answer=-1;}
	return answer;
}


Vector collX(Vector ball){
	Vector answer;
	float xdistance=abs(ball[0]-xwall[0][0]);
	for (int i=0;i<wall_n;i++)
	{
		if((ball[2]<xwall[i][1]+ball_depth)&&(ball[2]>xwall[i][2]-ball_depth))
		{
			if(xdistance>=abs(ball[0]-xwall[i][0])){
			xdistance=abs(ball[0]-xwall[i][0]);
			answer[0]=xwall[i][0];
			answer[1]=xwall[i][1];
			answer[2]=xwall[i][2];
			wall_hit=i+1;
		}
		}
	}
	return answer;
}


Vector calcNipPos()
{
	float cpX, cpY, cpZ;
	float hpX, hpY, hpZ;
	float cvX, cvY, cvZ;
	float hvX, hvY, hvZ;
	float npX,npY,npZ;
	float bpX,bpY,bpZ;
	float nvX,nvY,nvZ;
	float veloX,veloY,veloZ;
	float bvX,bvY,bvZ;
	float newnpX,newnpY,newnpZ;
	float ballRadius;
	float angle_ch;
	float ballMass;
	float accX,accY,accZ;
	Vector fOnNip;
	ballMass=graphCtrller->ballGr->getMass();

	angle_ch=graphCtrller->ballGr->ggetAngleBallGr();
	graphCtrller->cip.getPosition().getValue(cpX, cpY, cpZ);
	graphCtrller->hip.getPosition().getValue(hpX, hpY, hpZ);
	graphCtrller->nip.getPosition().getValue(npX, npY, npZ);
	graphCtrller->nip.getVelocity().getValue(nvX, nvY, nvZ);
	graphCtrller->hip.getVelocity().getValue(hvX, hvY, hvZ);
	graphCtrller->cip.getVelocity().getValue(cvX, cvY, cvZ);
	graphCtrller->ballGr->getPosition().getValue(bpX, bpY, bpZ);
	ballRadius=graphCtrller->ballGr->getRadius();
	graphCtrller->ballGr->getVelocity().getValue(bvX,bvY,bvZ);

	fOnNip[0]=kpHN*(hpX-npX)+kpCN*(cpX-npX)+K_BALL_X*(bpX-npX)+kdN*(hvX+cvX-2*nvX)+kdBall*(bvX-nvX)-ballMass*G*COEFFICIENT_FRICTION*sign(bpX);
	fOnNip[2]=kpHN*(hpZ-npZ)+kpCN*(cpZ-npZ)+K_BALL_Z*(bpZ-npZ)+kdN*(cvZ+hvZ-2*nvZ)+kdBall*(bvZ-nvZ)-ballMass*G*COEFFICIENT_FRICTION*sign(bpZ);


	accX=fOnNip[0]/ballMass;
	accZ=fOnNip[2]/ballMass;

	veloX=nvX+accX;
	veloZ=nvZ+accZ;

	newnpX=npX+veloX;//chpp[0];//
	newnpZ=npZ+veloZ;//chpp[2];

	npX=newnpX;
	npY=newnpY;
	npZ=newnpZ;
	return Point(npX,npY,npZ);

}
Vector calcBallPos(float angle)
{
	float cpX, cpY, cpZ;
	float cvX, cvY, cvZ;
	float hpX, hpY, hpZ;
	float hcpX, hcpY, hcpZ;
	float hhpX, hhpY, hhpZ;
	float bpX, bpY, bpZ;
	float npX,npY,npZ;
	float bvX,bvY,bvZ;
	float hhvX,hhvY,hhvZ;
	float hcvX,hcvY,hcvZ;
	float hvX,hvY,hvZ;

	float veloX,veloY,veloZ;
	float newbpX,newbpY,newbpZ;
	float ballRadius;
	float angle_ch;
	float ballMass;
	float accX,accY,accZ;
	Vector forceBall;

	angle_ch=graphCtrller->ballGr->ggetAngleBallGr();
	graphCtrller->nip.setPosition(calcNipPos());
	//graphCtrller->handleH.setPosition(calcHandleHPos(angle));
	//graphCtrller->handleC.setPosition(calcHandleCPos(angle));

	graphCtrller->cip.getPosition().getValue(cpX, cpY, cpZ);
	graphCtrller->hip.getPosition().getValue(hpX, hpY, hpZ);


	//graphCtrller->handleH.getPosition().getValue(hpX, hpY, hpZ);
	//graphCtrller->handleC.getPosition().getValue(cpX, cpY, cpZ);

	graphCtrller->nip.getPosition().getValue(npX, npY, npZ);
	graphCtrller->ballGr->getPosition().getValue(bpX, bpY, bpZ);
	graphCtrller->ballGr->getVelocity().getValue(bvX,bvY,bvZ);


	graphCtrller->handleH.getVelocity().getValue(hvX,hvY,hvZ);
	graphCtrller->handleC.getVelocity().getValue(cvX,cvY,cvZ);

	graphCtrller->ballGr->getVelocity().getValue(bvX,bvY,bvZ);

	ballRadius=graphCtrller->ballGr->getRadius();
	ballMass=graphCtrller->ballGr->getMass();






	// make nip and hip coincide and lie in between hip and cip
	/*graphCtrller->nip.setPosition(Point(0.5*(cpX+hpX), 0.5*(cpY+hpY), 0.5*(cpZ+hpZ)));
	Vector chpp;
	chpp[0]=0.5*(cpX+hpX);
	chpp[1]=0.5*(cpY+hpY);
	chpp[2]=0.5*(cpZ+hpZ);*////Cigil
	hhpX=bpX-(ball_width)*0.5*cos(angle_rot);
	hhpZ=bpZ+(ball_width)*0.5*sin(angle_rot);

	hcpX=bpX+(ball_width)*0.5*cos(angle_rot);
	hcpZ=bpZ-(ball_width)*0.5*sin(angle_rot);


	//graphCtrller->handleH.setPosition(Point(hhpX,bpY,hhpZ));
	//graphCtrller->handleC.setPosition(Point(hcpX,bpY,hcpZ));
	graphCtrller->hip.getVelocity().getValue(hvX,hvY,hvZ);
	graphCtrller->cip.getVelocity().getValue(cvX,cvY,cvZ);

	forceBall[0]=kpHN*(hpX-hhpX)+kdN*(hvX/*-hhvX*/)-ballMass*G*COEFFICIENT_FRICTION*sign(bpX);
	forceBall[0]+=kpHN*(cpX-hcpX)+kdN*(cvX/*-hcvX*/);
	forceBall[2]=kpHN*(hpZ-hhpZ)+kdN*(hvZ/*-hhvZ*/)-ballMass*G*COEFFICIENT_FRICTION*sign(bpZ);
	forceBall[2]+=kpHN*(cpZ-hcpZ)+kdN*(cvZ/*-hcvZ*/);

	ballMass=graphCtrller->ballGr->getMass();

	accX=forceBall[0]/ballMass;
	accZ=forceBall[2]/ballMass;

	veloX=bvX+accX;
	veloZ=bvZ+accZ;

	newbpX=bpX+veloX;//chpp[0];//
	newbpZ=bpZ+veloZ;//chpp[2];

	bpX=newbpX;
	bpZ=newbpZ;
	return Point(bpX,bpY,bpZ);


}

void copyBallToCIP()
{
	graphCtrller->cip.setPosition(graphCtrller->ballGr->getPosition());
	graphCtrller->cip.setVelocity(graphCtrller->ballGr->getVelocity());
}

void setForces(Vector wallDisp,int forcetype)
{
	float cpX, cpY, cpZ;
	float hpX, hpY, hpZ;
	float bpX, bpY, bpZ;
	float bvX,bvY,bvZ;
	float cvX, cvY, cvZ;
	float hvX, hvY, hvZ;
	float npX, npY, npZ;
	float nvX,nvY,nvZ;

	graphCtrller->cip.getPosition().getValue(cpX, cpY, cpZ);
	graphCtrller->hip.getPosition().getValue(hpX, hpY, hpZ);
	graphCtrller->cip.getVelocity().getValue(cvX, cvY, cvZ);
	graphCtrller->hip.getVelocity().getValue(hvX, hvY, hvZ);
	graphCtrller->ballGr->getPosition().getValue(bpX, bpY, bpZ);
	graphCtrller->ballGr->getVelocity().getValue(bvX,bvY,bvZ);

	graphCtrller->nip.getPosition().getValue(npX, npY, npZ);
	graphCtrller->nip.getVelocity().getValue(nvX, nvY, nvZ);
	if (forcetype==1)
	{
		fhw[0]=abs(wallDisp[0]-npX)*K_WALL_X-(nvX)*Kd_WALL;
		//fhw[2]=abs(wallDisp[2]-npZ)*K_WALL_X-(nvZ)*Kd_WALL;
		fcw[0]=abs(wallDisp[0]-npX)*K_WALL_X-(nvX)*Kd_WALL;
		//fcw[2]=abs(wallDisp[2]-npZ)*K_WALL_X-(nvZ)*Kd_WALL;
	}else if(forcetype==2)
	{
		fhw[0]=-abs(wallDisp[0]-npX)*K_WALL_X-(nvX)*Kd_WALL;
		//fhw[2]=-abs(wallDisp[2]-npZ)*K_WALL_X-(nvZ)*Kd_WALL;
		fcw[0]=-abs(wallDisp[0]-npX)*K_WALL_X-(nvX)*Kd_WALL;
		//fcw[2]=-abs(wallDisp[2]-npZ)*K_WALL_X-(nvZ)*Kd_WALL;
	}


}

void setForces2(int r11,int r22,int r33, int r44,int forcetypes)
{   
	//if(runs%1000==0)
	//{
	//cout<<r11<<"  r2  "<<r22<<endl;
	//cout<<r33<<"  r4  "<<r44<<endl<<endl;
	//}


	/*boundaryforceHZ= 0.0f;
	boundaryforceCZ= 0.0f;
	boundaryforceHX= 0.0f;
	boundaryforceCX= 0.0f;


	fhw[2]=0.0f; 
	fcw[2]=0.0f;
	fhw[0]=0.0f; 
	fcw[0]=0.0f;*/

	float constantf=0.030f;//.03f;
	float constantfb=0.030f;//.03f;
	float repeat=5000.0f;
	
	int repeat2=5000;
	if ((forcetypes==1)&&(runs%repeat2==0))//boundary collision
	{
		if ((r44 > 0)&& (r44>r33))
		{
			boundaryforceHZa[0]= constantfb*(r44-r33);
			boundaryforceCZa[0]= constantfb*(r44-r33);
		}	
		else if ((r33 > 0)&&(r33>r44))
		{
			boundaryforceCZa[0]= -constantfb*(r33-r44);
			boundaryforceHZa[0]= -constantfb*(r33-r44);
		}


		if ((r11 > 0)&&(r11>r22))
		{
			boundaryforceHXa[0]= constantfb*(r11-r22);
			boundaryforceCXa[0]= constantfb*(r11-r22);
		}	
		else if ((r22 > 0)&&(r22>r11))
		{
			boundaryforceCXa[0]= -constantfb*(r22-r11);
			boundaryforceHXa[0]= -constantfb*(r22-r11);
		}


		float magh1=sqrt(boundaryforceHXa[0]*boundaryforceHXa[0]+boundaryforceHZa[0]*boundaryforceHZa[0]);
		if(magh1!=0)
		{
			boundaryforceHXa[0]=boundaryforceHXa[0]/magh1*constantf;
			boundaryforceHZa[0]=boundaryforceHZa[0]/magh1*constantf;
		}
		float magc1=sqrt(boundaryforceCXa[0]*boundaryforceCXa[0]+boundaryforceCZa[0]*boundaryforceCZa[0]);
		if(magc1!=0)
		{
			boundaryforceCXa[0]=boundaryforceCXa[0]/magc1*constantf;
			boundaryforceCZa[0]=boundaryforceCZa[0]/magc1*constantf;
		}
	}

	if ((forcetypes==1)&&(runs%repeat2==2500))//boundary collision
	{
		if ((r44 > 0)&&(r44>r33))
		{
			boundaryforceHZa[1]= constantfb*(r44-r33);
			boundaryforceCZa[1]= constantfb*(r44-r33);
		}	
		else if ((r33 > 0)&&(r33>r44))
		{
			boundaryforceCZa[1]= -constantfb*(r33-r44);
			boundaryforceHZa[1]= -constantfb*(r33-r44);
		}


		if ((r11 > 0)&&(r11>r22))
		{
			boundaryforceHXa[1]= constantfb*(r11-r22);
			boundaryforceCXa[1]= constantfb*(r11-r22);
		}	
		else if ((r22 > 0)&&(r22>r11))
		{
			boundaryforceCXa[1]= -constantfb*(r22-r11);
			boundaryforceHXa[1]= -constantfb*(r22-r11);
		}


		float magh1=sqrt(boundaryforceHXa[1]*boundaryforceHXa[1]+boundaryforceHZa[1]*boundaryforceHZa[1]);
		if(magh1!=0)
		{
			boundaryforceHXa[1]=boundaryforceHXa[1]/magh1*constantf;
			boundaryforceHZa[1]=boundaryforceHZa[1]/magh1*constantf;
		}
		float magc1=sqrt(boundaryforceCXa[1]*boundaryforceCXa[1]+boundaryforceCZa[1]*boundaryforceCZa[1]);
		if(magc1!=0)
		{
			boundaryforceCXa[1]=boundaryforceCXa[1]/magc1*constantf;
			boundaryforceCZa[1]=boundaryforceCZa[1]/magc1*constantf;
		}
	}
	/*else if(forcetypes==1)//&&(runs%repeat2!=0))
	{
		prevHXb=boundaryforceHX;
		prevHZb=boundaryforceHZ;

		prevCXb=boundaryforceHX;
		prevCZb=boundaryforceHZ;



	}*/
	if((forcetypes==2)&&(runs%repeat2==0))//wall collision
	{
		if ((r44 > 0)&&(r44>r33))
		{
			fhw_arr[0][2]= -constantf*(r44-r33);
			fcw_arr[0][2]= -constantf*(r44-r33);
		}	
		else if ((r33 > 0)&&(r33>r44))
		{
			fhw_arr[0][2]= constantf*(r33-r44);
			fcw_arr[0][2]= constantf*(r33-r44);
		}


		if ((r11 > 0)&&(r11>r22))
		{
			fhw_arr[0][0]= -constantf*(r11-r22);
			fcw_arr[0][0]= -constantf*(r11-r22);
		}	
		else if ((r22 > 0)&&(r22-r11))
		{
			fhw_arr[0][0]= constantf*(r22-r11);
			fcw_arr[0][0]= constantf*(r22-r11);
		}



		float magh=sqrt(fhw_arr[0][0]*fhw_arr[0][0]+fhw_arr[0][2]*fhw_arr[0][2]);
		if(magh!=0)
		{
			fhw_arr[0][0]=fhw_arr[0][0]/magh*constantf;
			fhw_arr[0][2]=fhw_arr[0][2]/magh*constantf;
		}
		float magc=sqrt(fcw_arr[0][0]*fcw_arr[0][0]+fcw_arr[0][2]*fcw_arr[0][2]);
		if(magc!=0)
		{
			fcw_arr[0][0]=fcw_arr[0][0]/magc*constantf;
			fcw_arr[0][2]=fcw_arr[0][2]/magc*constantf;
		}	

		//cout<<"Hreacxset  "<<fhw[0]+boundaryforceHX<<"  Freacz  "<< fhw[2]+boundaryforceHZ<<endl;
	}

		if((forcetypes==2)&&(runs%repeat2==2500))//wall collision
	{
		if ((r44 > 0)&&(r44>r33))
		{
			fhw_arr[1][2]= -constantf*(r44-r33);
			fcw_arr[1][2]= -constantf*(r44-r33);
		}	
		else if ((r33 > 0)&&(r33>r44))
		{
			fhw_arr[1][2]= constantf*(r33-r44);
			fcw_arr[1][2]= constantf*(r33-r44);
		}


		if ((r11 > 0)&&(r11>r22))
		{
			fhw_arr[1][0]= -constantf*(r11-r22);
			fcw_arr[1][0]= -constantf*(r11-r22);
		}	
		else if ((r22 > 0)&&(r22>r11))
		{
			fhw_arr[1][0]= constantf*(r22-r11);
			fcw_arr[1][0]= constantf*(r22-r11);
		}



		float magh=sqrt(fhw_arr[1][0]*fhw_arr[1][0]+fhw_arr[1][2]*fhw_arr[1][2]);
		if(magh!=0)
		{
			fhw_arr[1][0]=fhw_arr[1][0]/magh*constantf;
			fhw_arr[1][2]=fhw_arr[1][2]/magh*constantf;
		}
		float magc=sqrt(fcw_arr[1][0]*fcw_arr[1][0]+fcw_arr[1][2]*fcw_arr[1][2]);
		if(magc!=0)
		{
			fcw_arr[1][0]=fcw_arr[1][0]/magc*constantf;
			fcw_arr[1][2]=fcw_arr[1][2]/magc*constantf;
		}	

		//cout<<"Hreacxset  "<<fhw[0]+boundaryforceHX<<"  Freacz  "<< fhw[2]+boundaryforceHZ<<endl;
	}
		/*else if((forcetypes==2)&&(runs%repeat2!=0))
		{

			prevHXw=fhw[0];
			prevHZw=fhw[2];

			prevCXw=fhw[0];
			prevCZw=fhw[2];
		}
*/

	

	/*float a5,a4,a3;
	a5=(3/5)/pow(repeat,19);
	a4=(-3/2)/pow(repeat,15);
	a3=1/(pow(repeat,11));
	float xx=fmod(runs,repeat);
	*/
	repeat=repeat*0.5;
	if(forcetypes==2){
		fhw[0]=((fhw_arr[1][0]-fhw_arr[0][0])/repeat)*fmod(crunsw,repeat)+fhw_arr[0][0];//a5*fhw[0]*(pow(xx,5))+a4*fhw[0]*(pow(xx,4))+a3*fhw[0]*(pow(xx,3)); //
		fhw[2]=((fhw_arr[1][2]-fhw_arr[0][2])/repeat)*fmod(crunsw,repeat)+fhw_arr[0][2]; //a5*fhw[2]*(pow(xx,5))+a4*fhw[2]*(pow(xx,4))+a3*fhw[2]*(pow(xx,3)); //
		fcw[0]=((fcw_arr[1][0]-fcw_arr[0][0])/repeat)*fmod(crunsw,repeat)+fcw_arr[0][0]; //a5*fhw[0]*(pow(xx,5))+a4*fhw[0]*(pow(xx,4))+a3*fhw[0]*(pow(xx,3)); //
		fcw[2]=((fcw_arr[1][2]-fcw_arr[0][2])/repeat)*fmod(crunsw,repeat)+fcw_arr[0][2]; //a5*fcw[2]*(pow(xx,5))+a4*fcw[2]*(pow(xx,4))+a3*fcw[2]*(pow(xx,3)); //
	}
	else if(forcetypes==1){
		boundaryforceHX=((boundaryforceHXa[1]-boundaryforceHXa[0])/repeat)*fmod(crunsb,repeat)+boundaryforceHXa[0];//a5*boundaryforceHX*(pow(xx,5))+a4*boundaryforceHX*(pow(xx,4))+a3*boundaryforceHX*(pow(xx,3)); //
		boundaryforceHZ=((boundaryforceHZa[1]-boundaryforceHZa[0])/repeat)*fmod(crunsb,repeat)+boundaryforceHZa[0];//a5*boundaryforceHZ*(pow(xx,5))+a4*boundaryforceHZ*(pow(xx,4))+a3*boundaryforceHZ*(pow(xx,3)); //
		boundaryforceCX=((boundaryforceCXa[1]-boundaryforceCXa[0])/repeat)*fmod(crunsb,repeat)+boundaryforceCXa[0];//a5*	boundaryforceCX*(pow(xx,5))+a4*boundaryforceCX*(pow(xx,4))+a3*	boundaryforceCX*(pow(xx,3)); //
		boundaryforceCZ=((boundaryforceCZa[1]-boundaryforceCZa[0])/repeat)*fmod(crunsb,repeat)+boundaryforceCZa[0];//a5*	boundaryforceCZ*(pow(xx,5))+a4*boundaryforceCZ*(pow(xx,4))+a3*	boundaryforceCZ*(pow(xx,3)); //
	}

	/*if(runs%25==0)
	{
		cout<<"Hreacxset  "<<fhw[0]+boundaryforceHX<<"  Freacz  "<< fhw[2]+boundaryforceHZ<<endl;
		cout<<"prev wallx  "<<prevCXw<<"prev wall z  "<<prevCZw<<endl;
		cout<<"runs  "<<runs<<endl;
		cout<<"CreacX  "<<fcw[0]+boundaryforceCX<<"  Freacz  "<< fcw[2]+boundaryforceCZ<<endl<<endl;
	}*/
		if(runs%1000==0)
		{
			//cout<<" x  "<<fhw[0]<<"  z  "<<fhw[2]<<endl<<endl;
		}
  
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



Vector xwallColl_sp(float wallx1,float wallx2,float wallz1,float wallz2,Vector ballPoint)
{   
	Vector wall_force;
	Vector wallDisp;
	float cpX, cpY, cpZ;
	float hpX, hpY, hpZ;
	float bpX, bpY, bpZ;
	float npX,npY,npZ;
	float bvX,bvY,bvZ;
	float baX,baY,baZ;
	float ballRadius;
	//float bppx,bppy,bppz;
	float accX,accY,accZ;

	ballRadius=5.0f;
	float answer;
	graphCtrller->ballGr->getVelocity().getValue(bvX,bvY,bvZ);
	graphCtrller->ballGr->getAcceleration().getValue(baX,baY,baZ);

	if((ballPoint[0]<(boundary[0]-boundaryThickness-ballRadius))&& (ballPoint[0]> -(boundary[0]-boundaryThickness-ballRadius))){
		if((ballPoint[2]<=wallz1)&&(ballPoint[2]>=wallz2))
		{
			if((ballPoint[0] <= (wallx1+0.5*wallWidth+ballRadius))&& (ballPoint[0] >(wallx1+ballRadius)) )
			{//cout<<"arti taraf"<<endl;
				//ballPoint[0]=boundary[0]-boundaryThickness-ballRadius;ball->posX=boundary[0]-boundaryThickness-ballRadius;
				//graphCtrller->ballGr->setPosition( Point(boundary[0]-boundaryThickness-ballRadius, ball->posY, ball->posZ));
				answer=(wallx1+0.5*wallWidth+ballRadius);
				//wall_force[0]=(ball->posX-(boundary[0]-boundaryThickness-ballRadius))*K_WALL_X;
				//wall_force[1]=0;wall_force[2]=0;
				//graphCtrller->ballGr->setAngleBallGr(graphCtrller->ballGr->calculateAngleBallGr_col(wall_force,Vector(0,0,0)));
				//boundaryforceHX = (boundary[0]-npX)*K_WALL_X-(nvX)*Kd_WALL;
				//boundaryforceCX = (boundary[0]-npX)*K_WALL_X-(nvX)*Kd_WALL;
				/*if (runs % 1000 == 0)
				cout << "+x" << ball->posX <<"BALLPOINT"<<ballPoint[0]<<endl;*/
				if(bvX > 0) {
					bvX = 0;		baX = 0;

					graphCtrller->ballGr->setVelocity(Vector(bvX,bvY,bvZ));
					graphCtrller->ballGr->setAcceleration(Vector(baX,baY,baZ)); 
				}

			}
			else if(ballPoint[0] >= (wallx1-0.5*wallWidth-ballRadius)&& (ballPoint[0] <(wallx1-ballRadius)) )
			{//cout<<"eksi taraf"<<endl;
				//ballPoint[0]=boundary[0]-boundaryThickness-ballRadius;ball->posX=boundary[0]-boundaryThickness-ballRadius;
				//graphCtrller->ballGr->setPosition( Point(boundary[0]-boundaryThickness-ballRadius, ball->posY, ball->posZ));
				answer=(wallx1-0.5*wallWidth-ballRadius);
				//wall_force[0]=(ball->posX-(boundary[0]-boundaryThickness-ballRadius))*K_WALL_X;
				//wall_force[1]=0;wall_force[2]=0;
				//graphCtrller->ballGr->setAngleBallGr(graphCtrller->ballGr->calculateAngleBallGr_col(wall_force,Vector(0,0,0)));
				//boundaryforceHX = (boundary[0]-npX)*K_WALL_X-(nvX)*Kd_WALL;
				//boundaryforceCX = (boundary[0]-npX)*K_WALL_X-(nvX)*Kd_WALL;
				/*if (runs % 1000 == 0)
				cout << "+x" << ball->posX <<"BALLPOINT"<<ballPoint[0]<<endl;*/
				if(bvX < 0) {
					bvX = 0;		baX = 0;

					graphCtrller->ballGr->setVelocity(Vector(bvX,bvY,bvZ));
					graphCtrller->ballGr->setAcceleration(Vector(baX,baY,baZ)); 
				}

			}
		}

	}
	ballPoint=Vector(answer,ballPoint[1],ballPoint[2]);

	return ballPoint;
}



float getTorque(Vector ballPoint, Vector wall,float angle)
{   
	Vector wall_force;
	Vector wallDisp;
	float cpX, cpY, cpZ;
	float hpX, hpY, hpZ;
	float bpX, bpY, bpZ;
	float npX,npY,npZ;
	float bvX,bvY,bvZ;
	float baX,baY,baZ;
	float ballRadius;


	float wallx1=wall[0];
	float wallz1=wall[1];
	float wallz2=wall[2];
	//float bppx,bppy,bppz;
	float accX,accY,accZ;
	float Bpnts[5][4];
	int collision=0;
	//float angle;
	float dist[4];
	float bppx;
	float bppz;
	float bppy;
	float halfdiag=0.5*sqrt(pow(ball_width,2)+pow(ball_depth,2));
	float answer;
	float answerz;
	float zcontrol;

	float angle1;//=(atan(ball_width/ball_depth)-angle);
	float di_angle=atan(ball_width/ball_depth);
	float beta;
	//float angle2=(atan(ball_width/ball_depth)+angle);
	//if (runs % 1000 == 0)
	//cout << "here" << angle << endl;
	graphCtrller->cip.getPosition().getValue(cpX, cpY, cpZ);
	graphCtrller->hip.getPosition().getValue(hpX, hpY, hpZ);
	graphCtrller->nip.getPosition().getValue(npX, npY, npZ);
	graphCtrller->ballGr->getPosition().getValue(bpX, bpY, bpZ);
	graphCtrller->ballGr->getVelocity().getValue(bvX,bvY,bvZ);
	graphCtrller->ballGr->getAcceleration().getValue(baX,baY,baZ);
	ballRadius=graphCtrller->ballGr->getRadius();
	beta=atan2(ball_depth,ball_width);
	angle1=PI-(beta-angle);
	float angle2=angle+beta+PI;
	float angle3=2*PI-(beta-angle);
	float angle4=beta+angle;
	Bpnts[1][1]=(bpX)*cos(angle)+(bpZ)*sin(angle)-5*cos(angle1);
	//bpX-halfdiag*cos(angle1);//ball_width*0.5*cosangle-ball_depth*0.5*sinangle;
	Bpnts[1][2]=bpY;
	Bpnts[1][3]=-(bpX)*sin(angle)+(bpZ)*cos(angle)+5*sin(angle1);
	//bpZ+halfdiag*sin(angle1);//ball_width*0.5*sinangle+ball_depth*0.5*cosangle;

	Bpnts[3][1]=(bpX)*cos(angle)+(bpZ)*sin(angle)+5*cos(angle3);
	//bpX-halfdiag*cos(angle2);
	Bpnts[3][2]=bpY;
	Bpnts[3][3]=-(bpX)*sin(angle)+(bpZ)*cos(angle)+5*sin(angle3);
	//bpZ+halfdiag*sin(angle2);

	Bpnts[2][1]=(bpX)*cos(angle)+(bpZ)*sin(angle)+5*cos(angle2);
	//bpX+halfdiag*cos(angle1);
	Bpnts[2][2]=bpY;
	Bpnts[2][3]=-(bpX)*sin(angle)+(bpZ)*cos(angle)+5*sin(angle2);
	//bpZ-halfdiag*sin(angle1);

	Bpnts[4][1]=(bpX)*cos(angle)+(bpZ)*sin(angle)+5*cos(angle4);
	//bpX+halfdiag*cos(angle2);
	Bpnts[4][2]=bpY;
	Bpnts[4][3]=-(bpX)*sin(angle)+(bpZ)*cos(angle)+5*sin(angle4);
	//bpZ-halfdiag*sin(angle2);


	if (wallx1>=0)
	{
		zcontrol=max(Bpnts[1][3],Bpnts[3][3]);
		zcontrol=max(Bpnts[2][3],zcontrol);
		zcontrol=max(Bpnts[4][3],zcontrol);
	}
	else 
	{
		zcontrol=min(Bpnts[1][3],Bpnts[3][3]);
		zcontrol=min(Bpnts[2][3],zcontrol);
		zcontrol=min(Bpnts[4][3],zcontrol);
	}

	for (int i=1;i<=4;i++){
		if((Bpnts[i][1]<(boundary[0]-boundaryThickness-ballRadius))&& (Bpnts[i][1]> -(boundary[0]-boundaryThickness-ballRadius)))
		{
			if((Bpnts[i][1]<=wallx1/*+ballRadius*/+0.5*wallWidth)&&(Bpnts[i][1]>=wallx1-0.5*wallWidth/*-ballRadius*/)&&(zcontrol<=wallz1)&&(zcontrol>=wallz2))
			{//cout << "collision11 " <<  endl;
				if(((Bpnts[i][1]-(wallx1/*-0.5*wallWidth*/))<=0)&&(ballPoint[0]>=wallx1-ballRadius-0.5*wallWidth)){
					collision=10+i;
					break;
				}
				else if(((Bpnts[i][1]-(wallx1/*+wallWidth*0.5*/))>=0)&&((ballPoint[0])<=wallx1+ballRadius+wallWidth*0.5)){
					collision=20+i;	
					break;
				}
			}
		}
	}

	Vector force=getForce();
	if ((collision==11)||(collision==21)||(collision==14)||(collision==24))
	{
		answer=-force[0]*ball_depth*0.5-force[2]*ball_width*0.5;
	}
	else 
		if ( (collision==12) || (collision==22) ||(collision==13)||(collision==23))
		{
			answer=force[0]*ball_depth*0.5-force[2]*ball_width*0.5;
		}



		return answer;
}
Vector getForce()
{
	Vector force;
	float bpX, bpY, bpZ;
	float npX,npY,npZ;
	float bvX,bvY,bvZ;

	force[0]=K_BALL_X*(npX-bpX)+kdBall*bvX;
	force[2]=K_BALL_Z*(npZ-bpZ)+kdBall*bvZ;
	return force;
}
int findMin(float dista[12])
{

	float prev;
	float prev1;
	int cc;
	int cc1;
	float minz1;
	minz1=dista[0];
	for(int j=1;j<=12;j++)
	{
		if(dista[j]<=minz1)
		{
			minz1=dista[j];
			cc=j+1;
		}else
		{
			minz1=minz1;
		}
	}
	return cc;
}


int findMAX(float dista[12])
{

	float prev;
	float prev1;
	int cc;
	int cc1;
	float max1;
	max1=dista[0];
	for(int j=1;j<=12;j++)
	{
		if(dista[j]>=max1)
		{
			max1=dista[j];
			cc=j+1;
		}else
		{
			max1=max1;
		}
	}
	return cc;
}

void sort(float bpoints[9][4],float minz[4],int numbers[4],int or)
{

	float minz1,minz2;
	minz1=bpoints[1][or];
	for(int j=1;j<=8;j++)
	{
		if(bpoints[j][or]<=minz1)
		{
			minz1=bpoints[j][or];
			numbers[0]=j;
		}else
		{
			minz1=minz1;
		}
	}

	for(int j=1;j<=8;j++)
	{
		if(bpoints[j][or]>minz1)
		{
			minz2=bpoints[j][or];
			break;
		}
	}

	for(int j=1;j<=8;j++)
	{
		if((bpoints[j][or]>minz1)&&(bpoints[j][or]<=minz2))
		{
			minz2=bpoints[j][or];
			numbers[1]=j;
		}else
		{
			minz2=minz2;
		}
	}


	float maxz1,maxz2;
	maxz1=bpoints[1][or];
	for(int j=1;j<=8;j++)
	{
		if(bpoints[j][or]>=maxz1)
		{
			maxz1=bpoints[j][3];
			numbers[3]=j;
		}else
		{
			maxz1=maxz1;
		}
	}

	for(int j=1;j<=8;j++)
	{
		if(bpoints[j][or]<maxz1)
		{
			maxz2=bpoints[j][or];
			break;
		}
	}

	for(int j=1;j<=8;j++)
	{
		if((bpoints[j][or]>=maxz2)&&(bpoints[j][or]<=maxz1))
		{
			maxz2=bpoints[j][or];
			numbers[2]=j;

		}else
		{
			maxz2=maxz2;
		}
	}


	minz[0]=minz1;
	minz[1]=minz2;
	minz[2]=maxz2;
	minz[3]=maxz1;

	//cout<<or<<"    "<<minz1<<"   "<<minz2<<"   "<<maxz2<<"    "<<maxz1<<endl;

}
void boundaryColl()
{
	Vector answer; 
	float bpX,bpY,bpZ;
	float bvX,bvY,bvZ;
	float baX,baY,baZ;
	float npX,npY,npZ;
	float nvX,nvY,nvZ;

	graphCtrller->ballGr->getPosition().getValue(bpX,bpY,bpZ);
	graphCtrller->ballGr->getVelocity().getValue(bvX,bvY,bvZ);
	graphCtrller->ballGr->getAcceleration().getValue(baX,baY,baZ);


	graphCtrller->nip.getPosition().getValue(npX, npY, npZ);
	graphCtrller->nip.getVelocity().getValue(nvX, nvY, nvZ);

	if(bpX > (boundary[0]-boundaryThickness-ballRadius))
	{   
		//answer[0]
		bpX=boundary[0]-boundaryThickness-ballRadius;
		graphCtrller->ballGr->setPosition(Vector(bpX,bpY,bpZ));
		boundaryforceHX = -abs(boundary[0]-npX)*K_WALL_X-(nvX)*Kd_WALL;
		boundaryforceCX = -abs(boundary[0]-npX)*K_WALL_X-(nvX)*Kd_WALL;
		if(bvX > 0) {
			bvX = 0;		baX = 0;
			graphCtrller->ballGr->setVelocity(Vector(bvX,bvY,bvZ));
			graphCtrller->ballGr->setAcceleration(Vector(baX,baY,baZ));
		}
	}
	else if(bpX < -(boundary[0]-boundaryThickness-ballRadius))
	{
		//answer[0]
		bpX=-(boundary[0]-boundaryThickness-ballRadius);
		graphCtrller->ballGr->setPosition(Vector(bpX,bpY,bpZ));
		boundaryforceHX=abs(-boundary[0]-npX)*K_WALL_X-(nvX)*Kd_WALL;
		boundaryforceCX=abs(-boundary[0]-npX)*K_WALL_X-(nvX)*Kd_WALL;
		if(bvX < 0) {
			bvX = 0;		baX = 0;
			graphCtrller->ballGr->setVelocity(Vector(bvX,bvY,bvZ));
			graphCtrller->ballGr->setAcceleration(Vector(baX,baY,baZ));
		}
	}

	if(bpZ > (boundary[2]-boundaryThickness-ballRadius))
	{	
		//answer[2]
		bpZ=boundary[2]-boundaryThickness-ballRadius;	
		graphCtrller->ballGr->setPosition(Vector(bpX,bpY,bpZ));
		boundaryforceHZ=-abs(boundary[2]-npZ)*K_WALL_Z-(nvZ)*Kd_WALL;
		boundaryforceCZ=-abs(boundary[2]-npZ)*K_WALL_Z-(nvZ)*Kd_WALL;
		if(bvZ > 0) {
			bvZ = 0;		baZ = 0;
			graphCtrller->ballGr->setVelocity(Vector(bvX,bvY,bvZ));
			graphCtrller->ballGr->setAcceleration(Vector(baX,baY,baZ));
		}
	}
	else if(bpZ < -(boundary[2]-boundaryThickness-ballRadius))
	{	
		//answer[2]
		bpZ=-(boundary[2]-boundaryThickness-ballRadius);
		graphCtrller->ballGr->setPosition(Vector(bpX,bpY,bpZ));
		boundaryforceHZ=abs(-boundary[2]-npZ)*K_WALL_Z-(nvZ)*Kd_WALL;
		boundaryforceCZ=abs(-boundary[2]-npZ)*K_WALL_Z-(nvZ)*Kd_WALL;
		if(bvZ < 0) {
			bvZ = 0;		baZ = 0;
			graphCtrller->ballGr->setVelocity(Vector(bvX,bvY,bvZ));
			graphCtrller->ballGr->setAcceleration(Vector(baX,baY,baZ));
		}
	}


	//return answer;
}

Vector boundaryColl2(float angle,Vector ballPoint)
{	

	Vector answer; 
	float bpX,bpY,bpZ;
	float bppX,bppY,bppZ;
	float bvX,bvY,bvZ;
	float baX,baY,baZ;
	float npX,npY,npZ;
	float nvX,nvY,nvZ;
	float Bpnts[49][4];
	float bnow[49][4];
	float lenArray[49][4];

	bpX=ballPoint[0];
	bpY=ballPoint[1];
	bpZ=ballPoint[2];
	int r1,r2,r3,r4;

	r1=0;
	r2=0;
	r3=0;
	r4=0;

	graphCtrller->ballGr->getPosition().getValue(bppX,bppY,bppZ);
	graphCtrller->ballGr->getVelocity().getValue(bvX,bvY,bvZ);
	graphCtrller->ballGr->getAcceleration().getValue(baX,baY,baZ);


	graphCtrller->nip.getPosition().getValue(npX, npY, npZ);
	graphCtrller->nip.getVelocity().getValue(nvX, nvY, nvZ);


	int coll=0;
	int collision2=0;
	float angle1;
	float beta;
	graphCtrller->cip.getPosition().getValue(cpX, cpY, cpZ);
	graphCtrller->nip.getPosition().getValue(npX, npY, npZ);

	beta=atan2(ball_depth,ball_width);
	angle1=1*(PI-(beta-angle));
	float teta=atan2(ball_depth*0.25,ball_width*0.5);
	float angle2=1*(angle+beta+PI);
	float angle3=(2*PI-(beta-angle));
	float angle4=beta+angle;
	float angle5=angle+0.5*PI;
	float angle6=angle+1.5*PI;
	float angle7=angle;
	float angle8=angle+1*PI;
	float angle9=2*PI-(teta-angle);
	float angle10=teta+angle;
	float angle11=PI-(teta-angle);
	float angle12=PI+(teta+angle);



	float h2=sqrt((ball_depth*0.25)*(ball_depth*0.25)*(ball_width*0.5)*(ball_width*0.5));
	float h=sqrt(ball_width*ball_width+ball_depth*ball_depth)*0.5;
	createLen(lenArray);

	for(int j=1;j<=48;j++)
	{
		Bpnts[j][1]=bpX+(lenArray[j][1]*cos(angle))+(lenArray[j][3]*sin(angle));
		Bpnts[j][3]=bpZ+(lenArray[j][3]*cos(angle))-(lenArray[j][1]*sin(angle));
		bnow[j][1]=bppX+(lenArray[j][1]*cos(angle))+(lenArray[j][3]*sin(angle));
		bnow[j][3]=bppZ+(lenArray[j][3]*cos(angle))-(lenArray[j][1]*sin(angle));
	}





	float aarray[13];
	aarray[1]=angle1;aarray[2]=angle2;aarray[3]=angle3;aarray[4]=angle4;aarray[5]=angle5;aarray[6]=angle6;
	aarray[7]=angle7;aarray[8]=angle8;aarray[9]=angle9;aarray[10]=angle10;aarray[11]=angle11;aarray[12]=angle12;

	float harray[13];
	harray[1]=h;harray[2]=h;harray[3]=h;harray[4]=h;harray[5]=ball_depth*0.5;harray[6]=ball_depth*0.5;
	harray[7]=ball_width*0.5;harray[8]=ball_width*0.5;harray[9]=h2;harray[10]=h2;harray[11]=h2;harray[12]=h2;
	int collision1=0;
	float tol=0.0f;
	float tolx=0.0f;
	int collx=0;
	for (int j=1;j<=48;j++){
	
		if((Bpnts[j][1] >= (boundary[0]-boundaryThickness-tol+fwx))&& (bnow[j][1] >= (fwx+boundary[0]-boundaryThickness-tol)))
		{   
			//answer[0]
			r2=r2+1;
			//answer[0]=prevpos2[0];
			//answer[1]=bpY;
			//answer[2]=bpZ;
			collision1=11;
			collision=1;
			collx=j;
			if(bvX > 0) {
				bvX = 0;		baX = 0;
				graphCtrller->ballGr->setVelocity(Vector(bvX,bvY,bvZ));
				graphCtrller->ballGr->setAcceleration(Vector(baX,baY,baZ));
			}
		}
		else if((Bpnts[j][1]<= fwx-(boundary[0]-boundaryThickness-tol))&&(bnow[j][1]<= fwx-(boundary[0]-boundaryThickness-tol)))
		{
			//answer[0]
			r1=r1+1;
			//answer[0]=prevpos2[0];
			//answer[1]=bpY;
			//answer[2]=bpZ;
			collision1=12;
			collision=1;
			collx=j;
			if(bvX < 0) {
				bvX = 0;		baX = 0;
				graphCtrller->ballGr->setVelocity(Vector(bvX,bvY,bvZ));
				graphCtrller->ballGr->setAcceleration(Vector(baX,baY,baZ));
			}
		}



		if((Bpnts[j][3] >= (fw+boundary[2]-boundaryThickness))&&(bnow[j][3] >= (fw+boundary[2]-boundaryThickness)))
		{	
			//answer[2]
			collision2=21;
			collision=1;
			coll=j;
			//answer[0]=bpX;
			//answer[1]=bpY;
			//answer[2]=prevpos2[2];
			r3=r3+1;
			if(bvZ > 0) {
				bvZ = 0;		baZ = 0;
				graphCtrller->ballGr->setVelocity(Vector(bvX,bvY,bvZ));
				graphCtrller->ballGr->setAcceleration(Vector(baX,baY,baZ));
			}
		}
		else if((Bpnts[j][3] <= fw-(boundary[2]-boundaryThickness))&&(bnow[j][3] <= fw-(boundary[2]-boundaryThickness)))
		{	r4=r4+1;
		collision2=22;
		collision=1;
		coll=j;
		//answer[0]=bpX;
		//answer[1]=bpY;
		//answer[2]=prevpos2[2];
		if(bvZ < 0) {
			bvZ = 0;		baZ = 0;
			graphCtrller->ballGr->setVelocity(Vector(bvX,bvY,bvZ));
			graphCtrller->ballGr->setAcceleration(Vector(baX,baY,baZ));
		}
		}

	} 


	for(int j=1;j<=48;j++){//xler için
		int i=j;
		if((collision1==11)&&(collx==j)/*&&(collision2==0)*/&&(collision==1))
		{
			answer[0]=(fwx+boundary[0]-boundaryThickness)-((lenArray[i][1]*cos(angle/*0.0f*/))+(lenArray[i][3]*sin(angle/*0.0f*/)));////-harray[j]*cos(aarray[j]);//
			//answer[1]=bppY;
			//answer[2]=bppZ;
		}
		else if((collision1==12)&&(collx==j)&&/*(collision2==0)&&*/(collision==1))
		{
			answer[0]=fwx-(boundary[0]-boundaryThickness)-((lenArray[i][1]*cos(angle/*0.0f*/))+(lenArray[i][3]*sin(angle/*0.0f*/)));//-harray[j]*cos(aarray[j]);//
			//answer[1]=bppY;
			//answer[2]=bppZ;
		}

		//		if((collision1==11)&&(collx==j)&&(collision2==21)&&(collision==1)&&(coll==i))
		//		{
		//			answer[0]=(boundary[0]-boundaryThickness)-((lenArray[i][1]*cos(angle/*0.0f*/))+(lenArray[i][3]*sin(angle/*0.0f*/)));//-harray[j]*cos(aarray[j]);
		//			//answer[1]=bppY;
		//			answer[2]=boundary[2]-boundaryThickness-((lenArray[i][3]*cos(angle/*0.0f*/))-(lenArray[i][1]*sin(angle/*0.0f*/)));//+harray[i]*sin(aarray[i]);
		//		}
		//		else 	if((collision1==11)&&(collx==j)&&(collision2==22)&&(collision==1)&&(coll==i))
		//		{
		//			answer[0]=(boundary[0]-boundaryThickness)-((lenArray[i][1]*cos(angle/*0.0f*/))+(lenArray[i][3]*sin(angle/*0.0f*/)));//-harray[j]*cos(aarray[j]);
		//			//answer[1]=bppY;
		//			answer[2]=-(boundary[2]-boundaryThickness)-((lenArray[i][3]*cos(angle/*0.0f*/))-(lenArray[i][1]*sin(angle/*0.0f*/)));//+harray[i]*sin(aarray[i]);
		//		}
		//		else if((collision1==12)&&(collx==j)&&(collision2==22)&&(collision==1)&&(coll==i))
		//		{
		//			answer[0]=-(boundary[0]-boundaryThickness)-((lenArray[i][1]*cos(angle/*0.0f*/))+(lenArray[i][3]*sin(angle/*0.0f*/)));//-harray[j]*cos(aarray[j]);
		//			//answer[1]=bppY;
		//			answer[2]=-(boundary[2]-boundaryThickness)-((lenArray[i][3]*cos(angle/*0.0f*/))-(lenArray[i][1]*sin(angle/*0.0f*/)));//+harray[i]*sin(aarray[i]);
		//		}
		//		else if((collision1==12)&&(collx==j)&&(collision2==21)&&(collision==1)&&(coll==i))
		//		{
		//			answer[0]=-(boundary[0]-boundaryThickness)-((lenArray[i][1]*cos(angle/*0.0f*/))+(lenArray[i][3]*sin(angle/*0.0f*/)));//-harray[j]*cos(aarray[j]);
		//			//answer[1]=bppY;
		//			answer[2]=boundary[2]-boundaryThickness-((lenArray[i][3]*cos(angle/*0.0f*/))-(lenArray[i][1]*sin(angle/*0.0f*/)));//+harray[i]*sin(aarray[i]);
		//		}


		if((collision2==21)&&(coll==i)&&(collision==1)/*&&(collision1==0)*/)
		{
			//answer[0]=bppX;
			//answer[1]=bppY;
			answer[2]=fw+boundary[2]-boundaryThickness-((lenArray[i][3]*cos(angle/*0.0f*/))-(lenArray[i][1]*sin(angle/*0.0f*/)));//+harray[i]*sin(aarray[i]);//

		}


		else if((collision2==22)&&(coll==i)&&(collision==1)/*&&(collision1==0)*/)
		{ 
			//answer[0]=bppX;
			//answer[1]=bppY;
			answer[2]=fw-(boundary[2]-boundaryThickness)-((lenArray[i][3]*cos(angle/*0.0f*/))-(lenArray[i][1]*sin(angle/*0.0f*/)));//+harray[i]*sin(aarray[i]);//
		}


		if ((collision==0)&&(coll==0)&&(collision2==0)&&(collision1==0)&&(collx==0))
		{
			answer[0]=bppX;
			answer[2]=bppZ;
		}

	}


	answer[1]=bppY;

	if(collx==0)
	{
		answer[0]=bppX;
	}

	if(coll==0)
	{
		answer[2]=bppZ;
	}



	/*if(runs%1000==0){
	cout<<answer[0]<<"   "<<answer[2]<<endl;
	}*/
	if(collision==1){
	setForces2(r1,r2,r3,r4,1);//18 eylul
	crunsb=crunsb+1;
	graphCtrller->boardGr->setHit(true);
	giveWarningB=1;
	}
	else
	{
		boundaryforceCX=0.0f;
		boundaryforceCZ=0.0f;
		boundaryforceHX=0.0f;
		boundaryforceHZ=0.0f;

		boundaryforceCXa[1]=0.0f;
		boundaryforceCZa[1]=0.0f;
		boundaryforceHXa[1]=0.0f;
		boundaryforceHZa[1]=0.0f;

		
		boundaryforceCXa[0]=0.0f;
		boundaryforceCZa[0]=0.0f;
		boundaryforceHXa[0]=0.0f;
		boundaryforceHZa[0]=0.0f;

		graphCtrller->boardGr->setHit(false);
		giveWarningB=0;

		crunsb=0;
	}

	//cout<<r1<<"  "<<r2<<"  "<<r3<<"  "<<r4<<endl;
	//graphCtrller->ballGr->setPosition(Vector(bpX,bpY,bpZ));
	return answer;
}

void checkHip(float newHp[1][4])
{
	float hpX,hpY,hpZ,cpX,cpY,cpZ,npX,npY,npZ;
	Vector prevHp;
	graphCtrller->cip.getPosition().getValue(cpX, cpY, cpZ);
	graphCtrller->nip.getPosition().getValue(npX, npY, npZ);
	graphCtrller->hip.getPosition().getValue(hpX, hpY, hpZ);

	if  ((hpX-npX)*(cpX-npX)<=0)
	{
		newHp[0][2]=hpY;
		newHp[0][1]=hpX;
		newHp[0][3]=hpZ;
	}
	else
	{		newHp[0][2]=hpY;
	newHp[0][1]=npX;
	newHp[0][3]=npZ;
	}




}
Vector calcHandleHPos(float angle)
{ 
	float hpX, hpY, hpZ;
	float hhpX, hhpY, hhpZ;
	float cvX, cvY, cvZ;
	float hvX, hvY, hvZ;
	float hhvX, hhvY, hhvZ;
	float npX,npY,npZ;
	float bpX,bpY,bpZ;
	float nvX,nvY,nvZ;
	float veloX,veloY,veloZ;
	float bvX,bvY,bvZ;
	float newhhpX,newhhpY,newhhpZ;
	float ballRadius;

	float ballMass;
	float accX,accY,accZ;
	Vector fOnHandleH;
	ballMass=graphCtrller->ballGr->getMass();

	graphCtrller->ballGr->getPosition().getValue(bpX, bpY, bpZ);
	ballRadius=graphCtrller->ballGr->getRadius();
	graphCtrller->ballGr->getVelocity().getValue(bvX,bvY,bvZ);
	graphCtrller->cip.getPosition().getValue(cpX, cpY, cpZ);
	graphCtrller->hip.getPosition().getValue(hpX, hpY, hpZ);
	//graphCtrller->handleH.getPosition().getValue(hhpX, hhpY, hhpZ);
	hhpX=bpX-ball_width*0.5*cos(angle);
	hhpZ=bpZ+ball_width*0.5*sin(angle);
	//graphCtrller->handleH.getVelocity().getValue(hhvX, hhvY, hhvZ);
	graphCtrller->hip.getVelocity().getValue(hvX, hvY, hvZ);
	graphCtrller->cip.getVelocity().getValue(cvX, cvY, cvZ);


	fOnHandleH[0]=kpHN*(hpX-hhpX)+kdN*(hvX/*-hhvX*/)-ballMass*G*COEFFICIENT_FRICTION*sign(bpX);
	fOnHandleH[2]=kpHN*(hpZ-hhpZ)+kdN*(hvZ/*-hhvZ*/)-ballMass*G*COEFFICIENT_FRICTION*sign(bpZ);


	accX=fOnHandleH[0]/ballMass;
	accZ=fOnHandleH[2]/ballMass;

	veloX=hvX+accX;
	veloZ=hvZ+accZ;

	newhhpX=hpX+veloX;//chpp[0];//
	newhhpZ=hpZ+veloZ;//chpp[2];

	hhpX=newhhpX;
	hhpY=newhhpY;
	hhpZ=newhhpZ;
	return Point(hhpX,hhpY,hhpZ);
}
Vector calcHandleCPos(float angle)
{ 
	float hpX, hpY, hpZ;
	float hcpX, hcpY, hcpZ;
	float cvX, cvY, cvZ;
	float hvX, hvY, hvZ;
	float hcvX, hcvY, hcvZ;
	float npX,npY,npZ;
	float bpX,bpY,bpZ;
	float nvX,nvY,nvZ;
	float veloX,veloY,veloZ;
	float bvX,bvY,bvZ;
	float newhcpX,newhcpY,newhcpZ;
	float ballRadius;
	float angle_ch;
	float ballMass;
	float accX,accY,accZ;
	Vector fOnHandleC;
	ballMass=graphCtrller->ballGr->getMass();

	angle_ch=graphCtrller->ballGr->ggetAngleBallGr();
	graphCtrller->cip.getPosition().getValue(cpX, cpY, cpZ);
	graphCtrller->hip.getPosition().getValue(hpX, hpY, hpZ);
	graphCtrller->handleC.getPosition().getValue(hcpX, hcpY, hcpZ);
	//graphCtrller->handleC.getVelocity().getValue(hcvX, hcvY, hcvZ);
	graphCtrller->hip.getVelocity().getValue(hvX, hvY, hvZ);
	graphCtrller->cip.getVelocity().getValue(cvX, cvY, cvZ);
	graphCtrller->ballGr->getPosition().getValue(bpX, bpY, bpZ);
	
	ballRadius=graphCtrller->ballGr->getRadius();
	graphCtrller->ballGr->getVelocity().getValue(bvX,bvY,bvZ);
	hcpX=bpX+ball_width*0.5*cos(angle);
	hcpZ=bpZ-ball_width*0.5*sin(angle);
	//fOnHandleC[0]=kpCN*(cpX-hcpX)+kdN*(cvX/*-hcvX*/)-ballMass*G*COEFFICIENT_FRICTION*sign(bpX);
	//fOnHandleC[2]=kpCN*(cpZ-hcpZ)+kdN*(cvZ/*-hcvZ*/)-ballMass*G*COEFFICIENT_FRICTION*sign(bpZ);


	//accX=fOnHandleC[0]/ballMass;
	//accZ=fOnHandleC[2]/ballMass;

	veloX=cvX+accX;
	veloZ=cvZ+accZ;

	newhcpX=cpX+veloX;//chpp[0];//
	newhcpZ=cpZ+veloZ;//chpp[2];

	hcpX=newhcpX;
	hcpY=newhcpY;
	hcpZ=newhcpZ;
	return Point(hcpX,hcpY,hcpZ);
}
void createLen(float lenArray[49][4])
{
	float beta=atan2(ball_depth,ball_width);
	float teta=atan2(ball_depth*0.25,ball_width*0.5);
	float h2=sqrt((ball_depth*0.25)*(ball_depth*0.25)*(ball_width*0.5)*(ball_width*0.5));
	float h=sqrt(ball_width*ball_width+ball_depth*ball_depth)*0.5;
	
	lenArray[1][1]=-h*cos(beta);
	lenArray[1][3]=-h*sin(beta);
	lenArray[2][1]=-h*cos(beta);
	lenArray[2][3]=h*sin(beta);
	lenArray[3][1]=h*cos(beta);
	lenArray[3][3]=h*sin(beta);
	lenArray[4][1]=h*cos(beta);
	lenArray[4][3]=-h*sin(beta);
	lenArray[5][1]=0.0f;
	lenArray[5][3]=-ball_depth*0.5;
	lenArray[6][1]=0;
	lenArray[6][3]=ball_depth*0.5;
	lenArray[7][1]=-ball_width*0.5;
	lenArray[7][3]=0.0f;
	lenArray[8][1]=ball_width*0.5;
	lenArray[8][3]=0.0f;
	lenArray[9][1]=h2*cos(teta);
	lenArray[9][3]=h2*sin(teta);
	lenArray[10][1]=-h2*cos(teta);
	lenArray[10][3]=h2*sin(teta);
	lenArray[11][1]=-h2*cos(teta);
	lenArray[11][3]=-h2*sin(teta);
	lenArray[12][1]=-h2*cos(teta);
	lenArray[12][3]=h2*sin(teta);

	lenArray[13][1]=ball_width*0.25;
	lenArray[13][3]=-ball_depth/2;

	lenArray[14][1]=ball_width*0.25;
	lenArray[14][3]=ball_depth/2;

	lenArray[15][1]=-ball_width*0.25;
	lenArray[15][3]=-ball_depth/2;

	lenArray[16][1]=-ball_width*0.25;
	lenArray[16][3]=(ball_depth/2);

	lenArray[17][1]=ball_width/8;
	lenArray[17][3]=(-ball_depth/2);

	lenArray[18][1]=-ball_width/8;
	lenArray[18][3]=(-ball_depth/2);

	lenArray[19][1]=-ball_width/8;
	lenArray[19][3]=(ball_depth/2);

	lenArray[20][1]=ball_width/8;
	lenArray[20][3]=(ball_depth/2);

	lenArray[21][1]=ball_width/8*3;
	lenArray[21][3]=-ball_depth*0.5;

	lenArray[22][1]=-ball_width/8*3;
	lenArray[22][3]=-ball_depth*0.5;

	lenArray[23][1]=-ball_width/8*3;
	lenArray[23][3]=ball_depth*0.5;

	lenArray[24][1]=ball_width/8*3;
	lenArray[24][3]=ball_depth*0.5;

	lenArray[25][1]=ball_width/16;
	lenArray[25][3]=-ball_depth/2;

	lenArray[26][1]=ball_width/16;
	lenArray[26][3]=ball_depth/2;

	lenArray[27][1]=-ball_width/16;
	lenArray[27][3]=-ball_depth/2;

	lenArray[28][1]=-ball_width/16;
	lenArray[28][3]=ball_depth/2;

	lenArray[29][1]=ball_width/16*3;
	lenArray[29][3]=-ball_depth/2;

	lenArray[30][1]=ball_width/16*3;
	lenArray[30][3]=ball_depth/2;

	lenArray[31][1]=-ball_width/16*3;
	lenArray[31][3]=-ball_depth/2;

	lenArray[32][1]=-ball_width/16*3;
	lenArray[32][3]=ball_depth/2;

	lenArray[33][1]=ball_width/16*5;
	lenArray[33][3]=-ball_depth/2;

	lenArray[34][1]=ball_width/16*5;
	lenArray[34][3]=ball_depth/2;

	lenArray[35][1]=-ball_width/16*5;
	lenArray[35][3]=-ball_depth/2;

	lenArray[36][1]=-ball_width/16*5;
	lenArray[36][3]=ball_depth/2;

	lenArray[37][1]=ball_width/16*7;
	lenArray[37][3]=-ball_depth/2;

	lenArray[38][1]=ball_width/16*7;
	lenArray[38][3]=ball_depth/2;

	lenArray[39][1]=-ball_width/16*7;
	lenArray[39][3]=-ball_depth/2;

	lenArray[40][1]=-ball_width/16*7;
	lenArray[40][3]=ball_depth/2;

	lenArray[41][1]=-ball_width/2;
	lenArray[41][3]=-ball_depth/8;

	lenArray[42][1]=ball_width/2;
	lenArray[42][3]=-ball_depth/8;

	lenArray[43][1]=-ball_width/2;
	lenArray[43][3]=ball_depth/8;

	lenArray[44][1]=-ball_width/2;
	lenArray[44][3]=ball_depth/8;

	lenArray[45][1]=-ball_width/2;
	lenArray[45][3]=-ball_depth/8*3;

	lenArray[46][1]=ball_width/2;
	lenArray[46][3]=-ball_depth/8*3;

	lenArray[47][1]=-ball_width/2;
	lenArray[47][3]=ball_depth/8*3;

	lenArray[48][1]=ball_width/2;
	lenArray[48][3]=ball_depth/8*3;

}