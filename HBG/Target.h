#pragma once
#include "Public.h"
#include "HapticInterfacePoint.h"
#include "MathCB.h"

class  Target
{
public:
	Target();
	Target(Point pos, float rd, int ch);

	Point getPosition();
	int getCharge();
	float getRadius();

	float distanceSq(HapticInterfacePoint *hIP); 
	float distance(HapticInterfacePoint *hIP); 

private:
	// default values
	static const int CHA = -1;
	// AYSE: move constants to public.h
	// static const int POSZERO = 0;
	// static const int RAD = 1;

	float posX;
	float posY;
	float posZ;
	float radius;
	int charge;
};
