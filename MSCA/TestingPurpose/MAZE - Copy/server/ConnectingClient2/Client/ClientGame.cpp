#include "ClientGame.h"


ClientGame::ClientGame(void)
{
    network = new ClientNetwork();

    // send init packet
    const unsigned int packet_size = sizeof(Packet);
    char packet_data[packet_size];

    Packet packet;
    packet.packet_type = INIT_CONNECTION;

    packet.serialize(packet_data);

    NetworkServices::sendMessage(network->ConnectSocket, packet_data, packet_size);
}



void ClientGame::sendActionPackets()
{
    // send action packet
    const unsigned int packet_size = sizeof(Packet);
    char packet_data[packet_size];

    Packet packet;
	packet.packet_timestamp = GetTickCount();
    packet.packet_type = ACTION_EVENT;
	packet.packet_data = this->positionData;

    packet.serialize(packet_data);

    NetworkServices::sendMessage(network->ConnectSocket, packet_data, packet_size);
}



void ClientGame::update()
{
    Packet packet;
    int data_length = network->receivePackets(network_data);

    if (data_length <= 0) 
    {
        //no data recieved
        return;
    }

    int i = 0;
    while (i < (unsigned int)data_length) 
    {
        packet.deserialize(&(network_data[i]));
        i += sizeof(Packet);

        switch (packet.packet_type) {

            case ACTION_EVENT:

				cout << " " << packet.packet_timestamp << " client received forces: " << packet.packet_data << " " << endl;

				this->forceData = packet.packet_data;
                sendActionPackets();

                break;

            default:

                cout << "error in packet types" << endl;

                break;
        }
    }
}