/**********************************************************************
Onur Berk Töre
Yeditepe University, RIS Lab
**********************************************************************/
#include <Inventor/Win/SoWin.h>
#include <Inventor/Win/viewers/SoWinExaminerViewer.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoCube.h>
#include "CubeGr.h"
#include "Public.h"
#include "HapticCallBack.h"
#include <vector>
#include "HapticInterfacePoint.h"

#include <conio.h>
#include <iostream>
#include <stdio.h>
#include <assert.h>

#define timeToInitPos 1000

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

double desiredX; 
double desiredY;
double desiredZ;

// controller gains
double kp = 0.7;
double kd = 0.5f;
double ki = 0.0001;

double error1x;
double error1y;
double error1z;

double error2x;
double error2y;
double error2z;

//step sizes
double dx;
double dy;
double dz;
long long runs = 0;
int i = 0;
/*
vector <HDSchedulerHandle> callbackHandlers;
extern HDCallbackCode HDCALLBACK DutyCycleCallback(void *pUserData);
extern HDCallbackCode HDCALLBACK BeginFrameCallback(void *);
extern HDCallbackCode HDCALLBACK EndFrameCallback(void *);
extern HDCallbackCode HDCALLBACK MyHapticLoop(void *pUserData);
*/

HDCallbackCode HDCALLBACK closedLoopControllerCallback(void *data);

using namespace std;

void mainLoop(void);
Point CubePoint;


#define timeToGoalPos 10000 



bool firstTime = true;

//Function Declaration
void graphicsTimerCallback(void * data,SoSensor * );

//hduVector3Dd force;
//hduVector3Dd oldPosition;

HHD hHD1;
CubeGr * myCube;
CubeGr * myCube2;

HapticInterfacePoint *HIP;

float oldVelX,oldVelY,oldVelZ;
float newVelX,newVelY,newVelZ;
float oldBallPosX,oldBallPosY,oldBallPosZ;
float newBallPosX,newBallPosY,newBallPosZ;


hduVector3Dd oldHIPPos;
hduVector3Dd oldBallPos;


HHOOK hMessageBoxHook_;
static int messageBoxCounter = 0;

// callback for hooking unwanted coin error message boxes
LRESULT CALLBACK CbtHookProc(int nCode, WPARAM wParam, LPARAM lParam)
{
	messageBoxCounter++; 

	if (messageBoxCounter == 1)
        return ::CallNextHookEx(    hMessageBoxHook_, 
                                    nCode, 
                                    wParam, 
                                    lParam); 

	if(nCode < 0)
    {
        return ::CallNextHookEx(    hMessageBoxHook_, 
                                    nCode, 
                                    wParam, 
                                    lParam); 
    }

    switch(nCode)
    {
        case HCBT_CREATEWND: // a window is about to be created
			return -1;

    }

    return ::CallNextHookEx(    hMessageBoxHook_, 
                                nCode, 
                                wParam, 
                                lParam); 
}



int main(int, char ** argv){

  int screenWidth = 1920;
  int screenHeight = 1440;

  HDErrorInfo error_info;
  HDSchedulerHandle hClosedLoopController;


  hHD1 = hdInitDevice("Left");
  if (HD_DEVICE_ERROR(error_info = hdGetError())) {
	  printf("Failed to initialize haptic device");
	  //hduPrintError(stderr, &error_info, "Failed to initialize haptic device");
      fprintf(stderr, "\nPress any key to quit.\n");
      getch();
      return -1;
  }
  printf("Hello Left Device\n");
  printf("Found device model: %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));

  hdEnable(HD_FORCE_OUTPUT);

  hClosedLoopController = hdScheduleAsynchronous(
       closedLoopControllerCallback, 0, 
       HD_MAX_SCHEDULER_PRIORITY);
  hdStartScheduler();

  hdMakeCurrentDevice(hHD1);
    /* Check for errors and abort if so. */
    if (HD_DEVICE_ERROR(error_info = hdGetError())){
        printf("Failed to start scheduler");
		//hduPrintError(stderr, &error_info, "Failed to start scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        return -1;
   }

   
 

  HWND window = SoWin::init("Cube");
  if (window==NULL) return(-1);


  //"//msdn.microsoft.com/en-us/library/windows/desktop/ms644977(v=vs.85).aspx""
  //msdn.microsoft.com/en-us/library/windows/desktop/ms724457(v=vs.85).aspx"
  //msdn.microsoft.com/en-us/library/windows/desktop/ms724176(v=vs.85).aspx"
  //en.wikibooks.org/wiki/Windows_Programming/Handles_and_Data_Types#HANDLE
	hMessageBoxHook_ = SetWindowsHookEx(WH_CBT,&CbtHookProc,::GetModuleHandle(NULL), GetCurrentThreadId());
	::UnhookWindowsHookEx(hMessageBoxHook_);
	hMessageBoxHook_ = 0;

	SoSeparator * root = new SoSeparator;
 
  SoWinExaminerViewer * viewer = new SoWinExaminerViewer(window);
  viewer->setSceneGraph(root);
  SoCamera * myCamera = viewer->getCamera();

  myCamera->position.setValue(145,80,60);
  myCamera->pointAt(SbVec3f(140,-5,-30));
  viewer->saveHomePosition();
  viewer->setSize(SbVec2s(screenWidth/*2560*/, screenHeight));
  myCamera->scaleHeight(1.4f);
  viewer->setBackgroundColor(SbColor(0,0,0));
  viewer->show();

  SetWindowPos(window,HWND_TOP,0,0,screenWidth,screenHeight,SWP_SHOWWINDOW);

  
  
  
  //Global yaptým CubeGr *
  myCube = new CubeGr(root, Point(100,100,100),ball_width ,ball_height,ball_depth, BALL_RADIUS,25, Vector(0.9f, 0.2f, 0.4f) );
  myCube2 = new CubeGr(root,Point(500,500,500), ball_width2, ball_height2, ball_depth,BALL_RADIUS,25,Vector(1.0,1.0,1.0));
 
  
  SoTimerSensor *graphicUpdate = new SoTimerSensor(graphicsTimerCallback, root);
  graphicUpdate->setInterval(1.0/40.0);
  graphicUpdate->schedule();

  root->ref();
 
  viewer->setSceneGraph(root);
  viewer->show();

  SoWin::show(window);
  SoWin::mainLoop();
  delete viewer;
  root->unref();
  hdStopScheduler();
  hdUnschedule(hClosedLoopController);
  hdDisableDevice(hHD1);
  return 0;

}
void graphicsTimerCallback(void * data,SoSensor * ){
	cout << "Hello";
	hduVector3Dd force;
	Point HapticPos = HIP->getPos();

	Point cubePosition = myCube->getPos();
	runs++;
	if(runs == 1){
		error1x = 0;
		error1y = 0;
		error1z = 0;

		error2x = 0;
		error2y = 0;
		error2z = 0;

		oldHIPPos[0] = HapticPos[0];
		oldHIPPos[1] = HapticPos[1];
		oldHIPPos[2] = HapticPos[2];
		cubePosition[0] =  oldBallPos[0];
		cubePosition[1] =  oldBallPos[1];
		cubePosition[2] =  oldBallPos[2];
	}

	desiredX = cubePosition[0];
	desiredY = cubePosition[1];
	desiredZ = cubePosition[2];

	error1x = error2x;
	error1y = error2y;
	error1z = error2z;

	dx = ( HapticPos[0] -cubePosition[0] ) / (timeToInitPos);
	dy = ( HapticPos[1] -cubePosition[1] ) / (timeToInitPos);
	dz = ( HapticPos[2] -cubePosition[2] ) / (timeToInitPos);

	desiredX += dx;
	desiredY += dy;
	desiredZ += dz;


	error2x = desiredX - cubePosition[0];
	error2y = desiredY - cubePosition[1];
	error2z = desiredZ - cubePosition[2];

	force[0] = kp * error2x + kd * (error2x - error1x);
	force[1] = kp * error2y + kd * (error2y - error1y);
	force[2] = kp * error2z + kd * (error2z - error1z);


/*Not working
	force[0] = kp * ( HapticPos[0] -cubePosition[0] ) + kd * ( (HapticPos[0] - oldHIPPos[0]) -(cubePosition[0] - oldBallPos[0]) );
	force[1] = kp * ( HapticPos[1] -cubePosition[1] ) + kd * ( (HapticPos[1] - oldHIPPos[1]) -(cubePosition[1] - oldBallPos[1]) );
	force[2] = kp * ( HapticPos[2] -cubePosition[2] ) + kd * ( (HapticPos[2] - oldHIPPos[2]) -(cubePosition[2] - oldBallPos[2]) );
*/
	
	



		oldHIPPos[0] = HapticPos[0];
		oldHIPPos[1] = HapticPos[1];
		oldHIPPos[2] = HapticPos[2];

		cubePosition[0] =  oldBallPos[0];
		cubePosition[1] =  oldBallPos[1];
		cubePosition[2] =  oldBallPos[2];

	float accX , accY, accZ;

	accX = 0;
	accY = 0;
	accZ = 0;

	accX = force[0] / 25;
	accY = force[1] / 25;
	accZ = force[2] / 25;

	newVelX = oldVelX + accX;
	newVelY = oldVelY + accY;
	newVelZ = oldVelZ + accZ;

	oldVelX = newVelX;
	oldVelY = newVelY;
	oldVelZ = newVelZ;


	newBallPosX  = oldBallPosX + newVelX;
	newBallPosY  = oldBallPosY + newVelY;
	newBallPosZ  = oldBallPosZ + newVelZ;

	oldBallPosX = newBallPosX;
	oldBallPosY = newBallPosY;
	oldBallPosZ = newBallPosZ;

	cubePosition[0] = newBallPosX;
	cubePosition[1] = newBallPosY;
	cubePosition[2] = newBallPosZ;

	/* Old Code, One Point to Another
	newVelX = 0;
	newVelY = 0;
	newVelZ = 0;

	newBallPosX = 0;
	newBallPosY = 0;
	newBallPosZ = 0;

	hduVector3Dd force;	

	force[0] = 0;
	force[1] = 1;
	force[2] = 2;

	Point cubePosition = myCube->getPos();
	runs++;


	if(runs == 1 ){

		error1x = 0;
		error1y = 0;
		error1z = 0;

		error2x = 0;
		error2y = 0;
		error2z = 0;

		dx = (( 500 - 100) / timeToInitPos);
		dy = (( 500 - 100) / timeToInitPos);
		dz = (( 500 - 100) / timeToInitPos);

		desiredX = 100;
		desiredY = 100;
		desiredZ = 100;

	}

	error1x = error2x;
	error1y = error2y;
	error1z = error2z;

	desiredX += dx;
	desiredY += dy;
	desiredZ += dz;



	error2x = desiredX - cubePosition[0];
	error2y = desiredY - cubePosition[1];
	error2z = desiredZ - cubePosition[2];



	force[0] = kp * error2x + kd * (error2x - error1x);
	force[1] = kp * error2y + kd * (error2y - error1y);
	force[2] = kp * error2z + kd * (error2z - error1z);
	
	cout << "forces are: " << force[0] << "\t" << force[1] << "\t" << force[2] << "\n";
	
	float accX, accY, accZ;
	accX = 0;
	accY = 0;
	accZ = 0;

	accX = force[0] / 2500;
	accY = force[1] / 2500;
	accZ = force[2] / 2500;


	if( oldBallPosX >= 500 || oldBallPosY >= 500 || oldBallPosZ >= 500){
		force[0] = 0;
		force[1] = 0;
		force[2] = 0;
		return;
	
	}

	
	
	
	
	newVelX = oldVelX + accX;
	newVelY = oldVelY + accY;
	newVelZ = oldVelZ + accZ;

	oldVelX = newVelX;
	oldVelY = newVelY;
	oldVelZ = newVelZ;


	newBallPosX  = oldBallPosX + newVelX;
	newBallPosY  = oldBallPosY + newVelY;
	newBallPosZ  = oldBallPosZ + newVelZ;

	oldBallPosX = newBallPosX;
	oldBallPosY = newBallPosY;
	oldBallPosZ = newBallPosZ;

	cubePosition[0] = newBallPosX;
	cubePosition[1] = newBallPosY;
	cubePosition[2] = newBallPosZ;
	*/
	myCube->setTranslate(Vector(cubePosition[0],cubePosition[1],cubePosition[2]) );
	myCube->setPosition(cubePosition);
	myCube2->setTranslate(Vector(HapticPos[0],HapticPos[1],HapticPos[2]) );
	myCube2->setPosition(CubePoint);

	//cout << CubePoint[0];

	//myCube->setTranslate(Vector(CubePoint[0],CubePoint[1],CubePoint[2]) );
	//myCube->setPosition(CubePoint);
}

HDCallbackCode HDCALLBACK closedLoopControllerCallback(void *data){
    HDErrorInfo error_info;
	hduVector3Dd masterPosition;
	hdBeginFrame(hHD1);
	hdGetDoublev(HD_CURRENT_POSITION, masterPosition);
	
	
	if(firstTime == true){
		Point HIPStartPoint;
		HIPStartPoint[0] = masterPosition[0];
		HIPStartPoint[1] = masterPosition[1];
		HIPStartPoint[2] = masterPosition[2];
		HIP = new HapticInterfacePoint(HIPStartPoint);
		firstTime = false;
	}
	
	HIP->posX = masterPosition[0];
	HIP->posY = masterPosition[1];
	HIP->posZ = masterPosition[2];
	
	
	//CubePoint[0] = masterPosition[0];
	//CubePoint[1] = masterPosition[1];
	//CubePoint[2] = masterPosition[2];

	
	    hdEndFrame(hHD1);


    /* Check for errors and abort the callback if a scheduler error
       is detected */
    if (HD_DEVICE_ERROR(error_info = hdGetError()))
    {
		printf("Error detected \n");
        //hduPrintError(stderr, &error_info, 
            //"Error detected \n");

        //if (hduIsSchedulerError(&error_info))
        {
            return HD_CALLBACK_DONE;
        }
    }

    /* Signify that the callback should continue running, i.e. that
       it will be called again the next scheduler tick. */
    return HD_CALLBACK_CONTINUE;
}