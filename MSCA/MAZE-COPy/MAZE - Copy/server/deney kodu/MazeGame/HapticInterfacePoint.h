#pragma once

#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/SbBasic.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoMaterial.h>
#include "MathCB.h"
#include "Public.h"


class  HapticInterfacePoint
{
public:
	
	float posX, posY, posZ, velX, velY, velZ, accX, accY, accZ;
	float mass;
	float radius;
	float prevPosX, prevPosZ;
	float nextPosX, nextPosZ;
	float windowVelX,windowVelZ;

	Vector fOnMe;
	SbMatrix transMatX, transMatZ, rotateMatX, rotateMatZ;

	//HapticInterfacePoint();
	HapticInterfacePoint(Point pos =Point(0,0,0));
	void updatePosition(Point newPos, int runs);
	void updateVelocity(Vector newVel);
	void updateAcceleration(Vector newAcc);
	Vector getWindowVel();
};

