/*
*	Author :		Ayse Kucukyilmaz, 2013
*
* The client program:
*	accepts incoming clients 
*	gets position from client
*	calculates appropriate forces
*	send forces to client
*/

#include "..\Network\NetworkServices.h"
#include "..\Network\NetworkData.h"

#include <stdio.h>
#include <conio.h>
#include <assert.h>
#include <iostream>
using namespace std;

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>


#include "ServerApp.h"
// used for multi-threading
#include <process.h>

static long int runs = 0;								// number of servo ticks

ServerApp * server;

void serverLoop(void * arg) 
{ 
	while(true) 
	{
		server->update();
	}
}

/*******************************************************************************
main function
Initializes the device, starts the schedule, creates a schedule callback
to handle forces, waits for the user to press a button, exits
the application.
*******************************************************************************/
int main(int argc, char* argv[])
{    
	// initialize the server
	server = new ServerApp();

	hduVector3Dd position(0,0,0);	// current position of the tip of the device
	hduVector3Dd force(0,0,0);		// current force that you will apply to device

	server->forceData = force;
	// create thread with arbitrary argument for the run function
	_beginthread( serverLoop, 0, &server->forceData);
		

	// calculate new force
	while (true)
	{
		runs++;

		if (runs % 100000000  == 0)
		{
			force[0] += 0.005;
			force[1] += 0.005;
			force[2] += 0.005;
		}

		server->forceData = force;
	}
	return 0;
}
