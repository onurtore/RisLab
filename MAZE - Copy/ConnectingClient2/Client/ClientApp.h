#pragma once

#include <HDU/hduVector.h>

#include "..\Network\NetworkData.h"
#include "ClientNetwork.h"


#include <winsock2.h>
#include <Windows.h>

#include <fstream>
#include <iostream>
using namespace std;

class ClientApp
{

public:

    ClientApp();
    ~ClientApp(void);

    ClientNetwork* network; 
    char network_data[MAX_PACKET_SIZE];

	void sendActionPackets();
    void update();

	hduVector3Dd positionData; 	 // read from device
	hduVector3Dd forceData;      // received from server
	time_t timestamp;			 // timestamp for force calculation
};