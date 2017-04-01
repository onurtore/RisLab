#pragma once
#include "HIPgraphics.h"

HIPgraphics::HIPgraphics()
{
}
HIPgraphics::HIPgraphics(SoSeparator *root, Point pos, float radius, Vector color,float transhc)
{
	ip			= new HapticInterfacePoint(pos);

	SoSeparator *IPSep	= new SoSeparator;
	transfMat			= new SoTransform();
	shape				= new SoSphere();
	mat					= new SoMaterial();

	shape->radius.setValue(radius);
	transfMat->translation.setValue(pos);
	mat->diffuseColor.setValue(color);
	mat->transparency.setValue(transhc);
    
	IPSep->addChild(transfMat);//cigil
	IPSep->addChild(mat);//cigil

	//TODO: activate hips on somehow if(effect->hipsOn)
	IPSep->addChild(shape);//cigil

	root->addChild(IPSep);//cigil
	
}
Point HIPgraphics::getPosition()
{
	return Point(ip->posX, ip->posY, ip->posZ);
}

Vector HIPgraphics::getVelocity()
{
	return Vector(ip->velX, 0, ip->velZ);
}

Vector HIPgraphics::getAcceleration()
{
	return Vector(ip->accX, 0, ip->accZ);
}

void HIPgraphics::setTranslate(Vector tr)
{
	transfMat->translation.setValue(tr);
}

void HIPgraphics::setPosition(Point pt)
{
	ip->updatePosition(pt);
}

void HIPgraphics::setVelocity(Vector vel)
{
	ip->updateVelocity(vel);
}

void HIPgraphics::setAcceleration(Vector acc)
{
	ip->updateAcceleration(acc);
}


void HIPgraphics::setRadius(float r)
{
	shape->radius.setValue(r);
}

