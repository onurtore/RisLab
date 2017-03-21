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
using namespace std;

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>



// used for multi-threading
#include <process.h>



void mainLoop(void);
HDCallbackCode HDCALLBACK closedLoopControllerCallback(void *data);

 
static long int runs = 0;								// number of servo ticks

ClientApp * client;



/*******************************************************************************
 main function
 Initializes the device, starts the schedule, creates a schedule callback
 to handle forces, waits for the user to press a button, exits
 the application.
*******************************************************************************/
int main(int argc, char* argv[])
{    

    // initialize the client 
    client = new ClientApp();

    HDErrorInfo error_info;
    HDSchedulerHandle hClosedLoopController;


    /* Initialize the device before attempting to call any hd functions. */
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error_info = hdGetError())) 
    {
        hduPrintError(stderr, &error_info, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;
    }

	printf("Hello Haptic Device!\n");
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
            break;
        }
    }

    /* For cleanup, unschedule our callback and stop the scheduler. */
    hdStopScheduler();
    hdUnschedule(hClosedLoopController);

    /* Disable the device, since we are done using it. */
    hdDisableDevice(hHD);

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
	hduVector3Dd error;		// error that you will use in your controller

    HHD hHD = hdGetCurrentDevice();

    /* Begin haptics frame.  In general, all state-related haptics calls
       should be made within a frame. */
    hdBeginFrame(hHD);
	
	/* Get the current position of the device. */ 
    hdGetDoublev(HD_CURRENT_POSITION, position); 

	// Now you have the coordinates of haptic tip in the variable 'position'
	// print the current position
	if (runs % 1000 == 0)
		printf("pos[0] = %f\tpos[1] = %f\tpos[2] = %f\n",position[0],position[1],position[2]);

	client->positionData = position;
	/* Send position to server and receives force from server */
	client->update();

	memset(force, 0, sizeof(hduVector3Dd));

	force = client->forceData;

	if ( abs(force[0]) > 1.0 || abs(force[1]) > 1.0 || abs(force[2]) > 1.0)
	{   
		force[0]=0.0;
		force[1]=0.0;
		force[2]=0.0;
	}

    /* Send the force to the device. */
    hdSetDoublev(HD_CURRENT_FORCE, force);
    
    /* End haptics frame. */
    hdEndFrame(hHD);

	runs++;
    /* Check for errors and abort the callback if a scheduler error
       is detected */
    if (HD_DEVICE_ERROR(error_info = hdGetError()))
    {
        hduPrintError(stderr, &error_info, 
            "Error detected \n");

        if (hduIsSchedulerError(&error_info))
        {
            return HD_CALLBACK_DONE;
        }
    }

    /* Signify that the callback should continue running, i.e. that
       it will be called again the next scheduler tick. */
    return HD_CALLBACK_CONTINUE;
}