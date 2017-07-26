#pragma once

#include "Public.h"
#include <Inventor/SbBasic.h>
#include "Bonus.h"

class  BonusGraphics
{
public:
	Bonus *bonus;

	SoTransform *transfMat;

	// define obstacle shape and material
	SoSphere *shape;
	SoMaterial *mat;
	
	BonusGraphics();
	BonusGraphics(SoSeparator *root, Point pos, float r = BONUS_RADIUS, float pt = 0);
	~BonusGraphics();
	
	Point getPosition();
	float getRadius();
	float getPoints();

	float distanceSq(HapticInterfacePoint *hIP); 
	float distance(HapticInterfacePoint *hIP); 
};
