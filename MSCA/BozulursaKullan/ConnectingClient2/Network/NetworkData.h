#pragma once
#include <string.h>
#include <time.h>
#include <HDU/hduVector.h>

#define MAX_PACKET_SIZE 1000000

enum PacketTypes {
	INIT_CONNECTION = 0,
    ACTION_EVENT = 1,
	KILL_CONNECTION = 2,

}; 

struct Packet {
	int packet_timestamp;
    unsigned int packet_type;
	hduVector3Dd position_data;
	hduVector3Dd force_data;

    void serialize(char * data) {
        memcpy(data, this, sizeof(Packet));
    }

    void deserialize(char * data) {
        memcpy(this, data, sizeof(Packet));
    }
};


