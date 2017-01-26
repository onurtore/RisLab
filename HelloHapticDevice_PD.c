/*********************************************************************
Onur Berk Töre
Yeditepe University, RIS Lab

Master-Slave relationship between two haptic devices. 
**********************************************************************/

#include <stdio.h>
#include <conio.h>
#include <assert.h>

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>



void mainLoop(void);
HDCallbackCode HDCALLBACK closedLoopControllerCallback(void *data);


#define timeToGoalPos   10000	// time to go to the initial position in millisecs


#define totalSimulationTime timeToInitPos + timeToGoalPos + 2 * timeToStabilize

long int runs = 0;			// number of servo ticks

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


double errTotalX = 0;
double errTotalY = 0;
double errTotalZ = 0;


// desired position to reach
double desiredX; 
double desiredY;
double desiredZ;


// controller gains
double kp = 0.9;
double kd = 1.5;
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

int i = 0;

HHD hHD1;
HHD hHD2;

hduVector3Dd oldmasterPosition;
hduVector3Dd vel;

/*******************************************************************************
 main function
 Initializes the device, starts the schedule, creates a schedule callback
 to handle forces, waits for the user to press a button, exits
 the application.
*******************************************************************************/
int main(int argc, char* argv[]){    



	

	HDErrorInfo error_info;
    HDSchedulerHandle hClosedLoopController;

    /* Initialize the device before attempting to call any hd functions. */
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
	

	
	hHD2 = hdInitDevice("Right");
    if (HD_DEVICE_ERROR(error_info = hdGetError())) {
        printf("Failed to initialize haptic device");
		//hduPrintError(stderr, &error_info, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;
    }

	printf("Hello Right Device\n");
    printf("Found device model: %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));

	hdEnable(HD_FORCE_OUTPUT);


    /* Schedule the main callback that will render forces to the device. */
    hClosedLoopController = hdScheduleAsynchronous(
        closedLoopControllerCallback, 0, 
        HD_MAX_SCHEDULER_PRIORITY);

    hdMakeCurrentDevice(hHD2);
	hdEnable(HD_FORCE_OUTPUT);
	hdStartScheduler();



	hdMakeCurrentDevice(hHD1);
    /* Check for errors and abort if so. */
    if (HD_DEVICE_ERROR(error_info = hdGetError())){
        printf("Failed to start scheduler");
		//hduPrintError(stderr, &error_info, "Failed to start scheduler");
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
    hdDisableDevice(hHD1);
	hdDisableDevice(hHD2);


	


    return 0;
}

/*******************************************************************************
 Servo loop.
*******************************************************************************/
HDCallbackCode HDCALLBACK closedLoopControllerCallback(void *data){
    
	HDErrorInfo error_info;

    hduVector3Dd masterPosition;	// current position of the tip of the master device
	hduVector3Dd slavePosition; // current position of the tip of the slave device
    hduVector3Dd force;		// current force that you will apply to device
	hduVector3Dd error;		// error that you will use in your controller

    
	float fMag;

    /* Begin haptics frame.  In general, all state-related haptics calls
       should be made within a frame. */
    hdBeginFrame(hHD1);
	hdBeginFrame(hHD2);

	
	hdMakeCurrentDevice(hHD1);
	/* Get the current position of the device. */ 
	hdGetDoublev(HD_CURRENT_POSITION, masterPosition);
	hdMakeCurrentDevice(hHD2);
	hdGetDoublev(HD_CURRENT_POSITION,slavePosition);

	// Now you have the coordinates of haptic tip in the variable 'position'

	// print the current position
	if (runs % 1000 == 0){
		//printf("masterpos[0] = %d\tmasterpos[1] = %d\tmasterpos[2] = %d\n",masterPosition[0],masterPosition[1],masterPosition[2]);
		//printf("slavepos[0] = %d\tslavepos[1] = %d\tslavepos[2] = %d\n",slavePosition[0],slavePosition[1],slavePosition[2]);
	}

	if(runs == 1 ){
		error1x = 0;
		error1y = 0;
		error1z = 0;

		error2x = 0;
		error2y = 0;
		error2z = 0;

	
	}
		desiredX = slavePosition[0];
		desiredY = slavePosition[1];
		desiredZ = slavePosition[2];

	
	runs = runs + 1;

	error1x = error2x;
	error1y = error2y;
	error1z = error2z;


	dx = ( masterPosition[0] - slavePosition[0] ) / (timeToGoalPos  )  ;
	dy = ( masterPosition[1] - slavePosition[1] ) / (timeToGoalPos ) ;
	dz = ( masterPosition[2] - slavePosition[2] ) / (timeToGoalPos ) ;

	desiredX += dx;
	desiredY += dy;
	desiredZ += dz;

	
	error2x = desiredX - slavePosition[0];
	error2y = desiredY - slavePosition[1];
	error2z = desiredZ - slavePosition[2];

	force[0] = kp * error2x + kd * ( error2x - error1x );
	force[1] = kp * error2y + kd * ( error2y - error1y );
	force[2] = kp * error2z + kd * ( error2z - error2z );

	force[0] *= 100;
	force[1] *= 100;
	force[2] *= 100;






	/*************************************************************************************************
	End of your code
	/*************************************************************************************************/	


    // FOLLOWING LINES OF CODE ARE FOR SAFETY. DO NOT DELETE THEM!!!
	fMag = sqrt( pow(force[0],2) + pow(force[1],2) + pow(force[2],2) );
	if (runs % 1000 == 0){
		printf("fMag = %f\n",fMag);	
	}
	if ( fMag > 2.4 ){   

	    force[0] = force[0] / fMag;
	    force[1] = force[1] / fMag;
	    force[2] = force[2] / fMag;
	}
	
	//if (runs % 1000 == 0)
		//printf("F[0] = %f\t F[1] = %f\t F[2] = %f\n", force[0], force[1], force[2]);


    /* Send the force to the device. */
    hdSetDoublev(HD_CURRENT_FORCE, force);

	    
    /* End haptics frame. */
    hdEndFrame(hHD1);
	hdEndFrame(hHD2);

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
