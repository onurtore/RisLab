#pragma once

#include <HDU/hduVector.h>

#include "ServerNetwork.h"
#include "..\Network\NetworkData.h"

#include <iostream>
#include <fstream>

using namespace std;

class ServerApp
{
public:

	ServerApp();
	~ServerApp();

	void update();
	void receiveFromClients();
	void sendActionPackets();
	void sendTerminationPackets();
	bool clientConnected();

	hduVector3Dd positionData; 	 // received from client
	hduVector3Dd forceData;      // calculated
	time_t timestamp;			 // timestamp of force calculation

	//ofstream fileOut;

	
private:
	// IDs for the clients connecting for table in ServerNetwork 
	static unsigned int client_id;
	// The ServerNetwork object 
	ServerNetwork* network;
	// data buffer
	char network_data[MAX_PACKET_SIZE];
};
