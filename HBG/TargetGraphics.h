#pragma once

#include "Public.h"
#include <Inventor/SbBasic.h>
#include "Target.h"

class  TargetGraphics
{
public:
	Target *target;

	SoTransform *transfMat;

	// define target shape and material
	SoCylinder *shape;
	SoMaterial *mat;
	
	// define current target indicator 
	SoCylinder *targetIndicator;
	SoMaterial *targetIndicatorMat;
	

	TargetGraphics();
	TargetGraphics(SoSeparator *root, Point pos, float rd = TARGET_RADIUS, float h = TARGET_HEIGHT, int ch = 1);
	~TargetGraphics(); //AYSE: add destructor
	
	Point getPosition();
	int getCharge();
	float getRadius();

	float distanceSq(HapticInterfacePoint *hIP); 
	float distance(HapticInterfacePoint *hIP); 
};
