#include "Target.h"

Target::Target()
{
	// defaults
	posX = POSZERO;	posY = POSZERO;	posZ = POSZERO;

	radius = TARGET_RADIUS;
	charge = CHA;
}

Target::Target(Point pos, float rd, int ch)
{
	posX = pos[0];
	posY = pos[1];
	posZ = pos[2];

	radius = rd;
	charge = ch;
}

float Target::distanceSq(HapticInterfacePoint *hIP) {
	float d = distance(hIP);
	return SQUARE(d);
}

float Target::distance(HapticInterfacePoint *hIP) {
	float d = EUC_DIST(posX, posZ, hIP->posX, hIP->posZ) - (radius + hIP->radius);
	return d > 0?  d: 0.0000001;
}

