#pragma once
#include "Public.h"
#include "HapticInterfacePoint.h"
#include "MathCB.h"

class  Bonus
{
public:
	Bonus();
	Bonus(Point pos, float radius, float bonusPts);

	Point getPosition();
	float getRadius();
	float getPoints();

	float distanceSq(HapticInterfacePoint *hIP); 
	float distance(HapticInterfacePoint *hIP); 

private:
	float posX;
	float posY;
	float posZ;
	float radius;
	float bonusPts;
};
