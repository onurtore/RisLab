#include "BonusGraphics.h"

BonusGraphics::BonusGraphics()
{
	// defaults
	bonus = new Bonus();
}

BonusGraphics::BonusGraphics(SoSeparator *root, Point pos, float r, float pt)
{
	bonus					= new Bonus(pos, r, pt);

	SoSeparator *bonusSep	= new SoSeparator;
	transfMat				= new SoTransform;
	shape					= new SoSphere;
	mat						= new SoMaterial;

	transfMat->translation.setValue(pos);
	
	mat->ambientColor.setValue(.3, .1, .1);
	mat->diffuseColor.setValue(.8, .7, .2);
	mat->specularColor.setValue(.4, .3, .1);
	mat->shininess = .4;
	
	shape->radius.setValue(r);
		
	bonusSep->addChild(transfMat);
	bonusSep->addChild(mat);
	bonusSep->addChild(shape);	

	// add cylinder to root
	root->addChild(bonusSep);
}

BonusGraphics::~BonusGraphics()
{
	delete bonus;
}
float BonusGraphics::distanceSq(HapticInterfacePoint *hIP) {
	float d = bonus->distance(hIP);
	return SQUARE(d);
}

float BonusGraphics::distance(HapticInterfacePoint *hIP) {
	float d = bonus->distance(hIP);
	return d > 0?  d: 0.0000001;
}

