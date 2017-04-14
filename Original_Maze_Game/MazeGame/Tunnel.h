#pragma once
#include "Public.h"
#include "HapticInterfacePoint.h"
#include "MathCB.h"

class Tunnel
{
public:
	Tunnel(Point* keypointPos, int keypointCount, float alpha = 0);
	~Tunnel();

	Point* getKeypoints();
	int getKeypointCount();
	float getAngle();

	//float distanceSq(Point pt); 
	//float distance(Point pt); 

private:
	Point* keypoints;
	int numKeypoints;
	float angle;
};
