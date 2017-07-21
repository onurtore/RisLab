#include "ServerApp.h"


unsigned int ServerApp::client_id; 
//static int data = 0;

ServerApp::ServerApp()
{
    // id's to assign clients for our table
    client_id = 0;
	
	
    // set up the server network to listen 
    network = new ServerNetwork(); 

	cout << "Waiting for clients" << endl;

	//fileOut.open("Server.csv");
}



ServerApp::~ServerApp()
{
	sendTerminationPackets();
}


void ServerApp::update()
{
	Packet packet;
    // get new clients
   if(network->acceptNewClient(client_id))
   {
	   cout << "client " << client_id << " has been connected to the server" << endl;

       client_id++;
   }

   receiveFromClients();
}



void ServerApp::receiveFromClients()
{
	Packet packet;
    // go through all clients
    std::map<unsigned int, SOCKET>::iterator iter;

    for(iter = network->sessions.begin(); iter != network->sessions.end(); iter++)
    {
        int data_length = network->receiveData(iter->first, network_data);

        if (data_length <= 0) 
        {
            //no data recieved
            continue;
        }

        int i = 0;
        while (i < (unsigned int)data_length) 
        {
            packet.deserialize(&(network_data[i]));
            i += sizeof(Packet);

            switch (packet.packet_type) {

                 case ACTION_EVENT:
					//fileOut << "R " << packet.packet_timestamp << ", " << packet.position_data << ", " << packet.force_data << endl;
					this->positionData = packet.position_data;
					sendActionPackets();
					//fileOut << "S " << packet.packet_timestamp << ", " << packet.position_data << ", " << packet.force_data << endl;
					//fileOut << "pos updated " << positionData << ", " << forceData << endl;
                    break;

                default:
                    cout << "error in packet types" << endl;

                    break;
            }
        }
    }
}



void ServerApp::sendActionPackets()
{
    // send action packet
    const unsigned int packet_size = sizeof(Packet);
    char packet_data[packet_size];

    Packet packet;
	packet.packet_timestamp = this->timestamp;
    packet.packet_type = ACTION_EVENT;
	packet.force_data = this->forceData;

    packet.serialize(packet_data);

    network->sendToAll(packet_data,packet_size);
}



void ServerApp::sendTerminationPackets()
{
    // send action packet
    const unsigned int packet_size = sizeof(Packet);
    char packet_data[packet_size];

    Packet packet;
	packet.packet_timestamp = this->timestamp;
    packet.packet_type = KILL_CONNECTION;
	
    packet.serialize(packet_data);

    network->sendToAll(packet_data,packet_size);
}
	
bool ServerApp::clientConnected()
{
	return (client_id > 0);
}