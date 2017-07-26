/*
*	Author :		Ayse Kucukyilmaz, 2013
*
* The client program:
*	connects to the server
*	reads position of the haptic device
*	sends position to server
*	receives force from server
*	sends the received force to the device
*
*/
#include "..\Network\NetworkServices.h"
#include "..\Network\NetworkData.h"

#include "ClientApp.h"

#include <stdio.h>
#include <conio.h>
#include <assert.h>
#include <iostream>
#include <float.h>
using namespace std;

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>



// used for multi-threading
#include <process.h>

static volatile HDboolean bRampDownForces = FALSE;
static volatile HDboolean bRampUpForces = FALSE;
static HDdouble *gServoMotorTemp = 0;
static HDint gNumMotors;
static HDdouble *aMotorTemp = 0;



void clientLoop(void *arg);

HDCallbackCode HDCALLBACK closedLoopControllerCallback(void *data);
void PreventWarmMotors(hduVector3Dd force);
HDCallbackCode HDCALLBACK QueryMotorTemp(void *pUserData);

static long int runs = 0;								// number of servo ticks

ClientApp * client;

HHD hHD;
//ofstream fp;

//CÝGÝL
int interval = 50;
float prevForceX = 0.0f; float prevForceY = 0.0f; float prevForceZ = 0.0f; 
float nextForceX = 0.0f; float nextForceY = 0.0f; float nextForceZ = 0.0f;



/*******************************************************************************
 main function
 Initializes the device, starts the schedule, creates a schedule callback
 to handle forces, waits for the user to press a button, exits
 the application.
*******************************************************************************/
int main(int argc, char* argv[])
{    
//	fp.open("force_acc_vel.txt");
	HDErrorInfo error_info;
    HDSchedulerHandle hClosedLoopController;


    /* Initialize the device before attempting to call any hd functions. */
    hHD = hdInitDevice("Right");//Omni");
    if (HD_DEVICE_ERROR(error_info = hdGetError())) 
    {
        hduPrintError(stderr, &error_info, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;
    }

	printf("Hello Haptic Device!\n");
	printf("Phantom id: %d is initialized.\n", (int)hHD);
    printf("Found device model: %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));


    /* Schedule the main callback that will render forces to the device. */
    hClosedLoopController = hdScheduleAsynchronous(
        closedLoopControllerCallback, 0, 
        HD_DEFAULT_SCHEDULER_PRIORITY);

    hdEnable(HD_FORCE_OUTPUT);
 
	hdStartScheduler();

	
    /* Check for errors and abort if so. */
    if (HD_DEVICE_ERROR(error_info = hdGetError()))
    {
        hduPrintError(stderr, &error_info, "Failed to start scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        return -1;
    }


	// initialize the client 
    client = new ClientApp();
	//_beginthread(clientLoop,0,0);

    /* Wait until the user presses a key. Meanwhile, the scheduler is
    running and applying forces to the device. */
    printf("Press any key to quit.\n\n");

    while (!_kbhit())
    {
        /* Periodically check if the closedLoopController callback has exited */
        if (!hdWaitForCompletion(hClosedLoopController, HD_WAIT_CHECK_STATUS))
        {
            fprintf(stderr, "Press any key to quit.\n");     
            getch();
			getch();
			getch();
			getch();
            break;
        }
    }

    /* For cleanup, unschedule our callback and stop the scheduler. */
    hdStopScheduler();
    hdUnschedule(hClosedLoopController);

    /* Disable the device, since we are done using it. */
    hdDisableDevice(hHD);
//	fp.close();
    return 0;
}

/*******************************************************************************
 Servo loop.
*******************************************************************************/
HDCallbackCode HDCALLBACK closedLoopControllerCallback(void *data)
{
    HDErrorInfo error_info;
	
    hduVector3Dd position;	// current position of the tip of the device
	hduVector3Dd force;		// current force that you will apply to device
	hduVector3Dd givenForce;		// current force that you will apply to device
	hduVector3Dd error;		// error that you will use in your controller

	memset(force, 0, sizeof(hduVector3Dd));
	//HHD hHD = hdGetCurrentDevice();

    /* Begin haptics frame.  In general, all state-related haptics calls
       should be made within a frame. */
    hdBeginFrame(hHD);
	
	/* Get the current position of the device. */ 
    hdGetDoublev(HD_CURRENT_POSITION, position); 
	while ( client->positionData == NULL ){

	}
	client->positionData = position;

	///* Send position to server and receives force from server */
	client->update();

	force = client->forceData;

//	fp << client->timestamp << " " << force[0] << "  " << force[1] << "  " << force[2] << endl;

    /* Send the force to the device. */
	//PreventWarmMotors( force );  
    hdSetDoublev(HD_CURRENT_FORCE, force);
    
	//hdScheduleSynchronous(QueryMotorTemp, aMotorTemp, HD_DEFAULT_SCHEDULER_PRIORITY);

    /* End haptics frame. */
    hdEndFrame(hHD);

	runs++;
    /* Check for errors and abort the callback if a scheduler error
       is detected */
    if (HD_DEVICE_ERROR(error_info = hdGetError()))
    {
        hduPrintError(stderr, &error_info, "Error detected \n");

        if (hduIsSchedulerError(&error_info))
        {
            return HD_CALLBACK_DONE;
        }
    }

    /* Signify that the callback should continue running, i.e. that
       it will be called again the next scheduler tick. */
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
Scales the force down, taking into account the motor temperature.
*******************************************************************************/
void PreventWarmMotors(hduVector3Dd force)
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


void clientLoop(void *arg)
{
	hduVector3Dd givenForce;
	hduVector3Dd force;
	
	while (true)
	{
		/* Send position to server and receives force from server */
		client->update();
	
		force = client->forceData;

		if(runs % interval == 0)
		{
			
			prevForceX = nextForceX;
			prevForceY = nextForceY;
			prevForceZ = nextForceZ;

			nextForceX =	force[0];
			nextForceY =	force[1];
			nextForceZ =	force[2];
		}

		
		givenForce[0] = prevForceX + (((nextForceX - prevForceX)/interval) * fmod(runs,double(interval)) ) ;
		givenForce[1] = prevForceY + (((nextForceY - prevForceY)/interval) * fmod(runs,double(interval))) ;
		givenForce[2] = prevForceZ + (((nextForceZ - prevForceZ)/interval) * fmod(runs,double(interval))) ;

		client->forceData = givenForce;

	}
}
