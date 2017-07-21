#pragma once
#include "hIPgraphics.h"

hIPgraphics::hIPgraphics()
{
}
hIPgraphics::hIPgraphics(SoSeparator *root, Point pos, float radius, Vector color)
{
	ip			= new HapticInterfacePoint(pos);

	SoSeparator *IPSep	= new SoSeparator;
	transfMat			= new SoTransform();
	shape				= new SoSphere();
	mat					= new SoMaterial();

	shape->radius.setValue(radius);
	transfMat->translation.setValue(pos);
	mat->diffuseColor.setValue(color);

	IPSep->addChild(transfMat);
	IPSep->addChild(mat);

	//TODO: activate hips on somehow if(effect->hipsOn)
	IPSep->addChild(shape);

	root->addChild(IPSep);
	
}
Point hIPgraphics::getPosition()
{
	return Point(ip->posX, ip->posY, ip->posZ);
}

Vector hIPgraphics::getVelocity()
{
	return Vector(ip->velX, 0, ip->velZ);
}

Vector hIPgraphics::getAcceleration()
{
	return Vector(ip->accX, 0, ip->accZ);
}

void hIPgraphics::setTranslate(Vector tr)
{
	transfMat->translation.setValue(tr);
}

void hIPgraphics::setPosition(Point pt)
{
	ip->updatePosition(pt);
}

void hIPgraphics::setVelocity(Vector vel)
{
	ip->updateVelocity(vel);
}

void hIPgraphics::setAcceleration(Vector acc)
{
	ip->updateAcceleration(acc);
}


void hIPgraphics::setRadius(float r)
{
	shape->radius.setValue(r);
}

