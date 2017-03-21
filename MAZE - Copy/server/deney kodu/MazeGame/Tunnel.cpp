#include "Tunnel.h"

Tunnel::Tunnel(Point* keypointPos, int keypointCount, float alpha)
{
	keypoints = new Point[keypointCount];
	
	for (int i = 0; i < keypointCount; i++)
	{
		keypoints[i] = keypointPos[i];
	}
	numKeypoints = keypointCount;

	angle = alpha;
}


Tunnel::~Tunnel()
{
	delete [] keypoints;
}

Point* Tunnel::getKeypoints()
{
	return keypoints;
}

int Tunnel::getKeypointCount()
{
	return numKeypoints;
}

float Tunnel::getAngle()
{
	return angle;
}