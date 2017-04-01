#include "Bonus.h"

Bonus::Bonus()
{
	// defaults
	posX		= POSZERO;	
	posY		= POSZERO;	
	posZ		= POSZERO;

	radius		= BONUS_RADIUS;
	bonusPts	= 0;
}

Bonus::Bonus(Point pos, float r, float pt)
{
	posX = pos[0];	posY = pos[1];	posZ = pos[2];

	radius		= r;
	bonusPts	= pt;
}

Point Bonus::getPosition()
{
	return Point(posX, posY, posZ);
}

float Bonus::getRadius()
{
	return radius;
}

float Bonus::getPoints()
{
	return bonusPts;
}

float Bonus::distance(HapticInterfacePoint *hIP) {
	float d = EUC_DIST(posX, posZ, hIP->posX, hIP->posZ) - (radius + hIP->radius);
	return d > 0?  d: 0.0000001;
}

float Bonus::distanceSq(HapticInterfacePoint *hIP) {
	float d = distance(hIP);
	return SQUARE(d);
}


