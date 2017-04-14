#pragma once

#include "Public.h"
#include <Inventor/SbBasic.h>
#include "Obstacle.h"

class  ObstacleGra
{
public:
	Obstacle *obs;

	SoTransform *transfMat;

	// define obstacle shape and material
	SoCylinder *shape;
	SoMaterial *mat;
	
	// define current target indicator 
	SoCylinder *targetIndicator;
	SoMaterial *targetIndicatorMat;
	

	ObstacleGra();
	ObstacleGra(SoSeparator *root, Point pos, float rd, float h, int ch);

	Point getPosition();
	int getCharge();
	float getRadius();

	float distanceSq(HapticInterfacePoint *hIP); 
	float distance(HapticInterfacePoint *hIP); 
};
