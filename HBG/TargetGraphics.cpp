#include "TargetGraphics.h"

TargetGraphics::TargetGraphics()
{
	// defaults
	//AYSE: not really necessary
	//target = (Target*)malloc( sizeof(*target) );
	target = new Target();
}

TargetGraphics::TargetGraphics(SoSeparator *root, Point pos, float rd, float h, int ch)
{
	//AYSE: not really necessary
	//target = (Target*)malloc( sizeof(*target) );
	target = new Target(pos, rd, ch);

	SoSeparator *targetSep = new SoSeparator;

	transfMat			= new SoTransform;
	shape				= new SoCylinder;
	mat					= new SoMaterial;

	targetIndicator		= new SoCylinder;
	targetIndicatorMat	= new SoMaterial;

	transfMat->translation.setValue(pos);
	shape->radius.setValue(rd);
	shape->height.setValue(h);

	targetIndicator->radius.setValue(rd+1);
	targetIndicator->height.setValue(0.03f);
	targetIndicatorMat->diffuseColor.setValue(COLOR_TARGET_IND_OFF);

	targetSep->addChild(transfMat);
	targetSep->addChild(mat);
	targetSep->addChild(shape);	
	targetSep->addChild(targetIndicatorMat);
	targetSep->addChild(targetIndicator);

	// add cylinder to root
	root->addChild(targetSep);
}

//AYSE: add destructor
TargetGraphics::~TargetGraphics()
{
	delete target;	
}


float TargetGraphics::distanceSq(HapticInterfacePoint *hIP) {
	float d = target->distance(hIP);
	return SQUARE(d);
}

float TargetGraphics::distance(HapticInterfacePoint *hIP) {
	float d = target->distance(hIP);
	return d > 0?  d: 0.0000001;
}

