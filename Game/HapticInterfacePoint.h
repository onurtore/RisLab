/**********************************************************************
Onur Berk Töre
Yeditepe University, RIS Lab
**********************************************************************/
#pragma once

#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/SbBasic.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoMaterial.h>
//#include "MathCB.h"
#include "Public.h"


class  HapticInterfacePoint
{
public:
	
	float posX, posY, posZ;
	float velX, velY, velZ;
	float accX, accY, accZ;
	float oldPosX, oldPosY,oldPosZ;
	float oldVelX, oldVelY,oldVelZ;
	float oldAccX, oldAccY,oldAccZ;
	float forceX, forceY,forceZ;
	float mass;
	float radius;

	
	//HapticInterfacePoint();
	HapticInterfacePoint(Point pos =Point(0,0,0), Vector vel = Vector(0,0,0), Vector acc = Vector(0,0,0));
	void updatePosition(Point newPos);
	void updateVelocity(Vector newVel);
	void updateAcceleration(Vector newAcc);
	void calculateForce();
	Point getPos();
};

