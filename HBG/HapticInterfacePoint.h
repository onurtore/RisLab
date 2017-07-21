#pragma once

#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/SbBasic.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoMaterial.h>
#include "MathCB.h"

typedef SbVec3f Point;
typedef SbVec3f Vector;

class  HapticInterfacePoint
{
public:
	
	float posX, posY, posZ, velX, velY, velZ, accX, accY, accZ;
	float mass;
	float radius;

	Vector fOnMe;
	SbMatrix transMatX, transMatZ, rotateMatX, rotateMatZ;

	//HapticInterfacePoint();
	HapticInterfacePoint(Point pos = Point(0,0,0), Vector vel = Vector(0,0,0), Vector acc = Vector(0,0,0));
	void updatePosition(Point newPos);
	void updateVelocity(Vector newVel);
	void updateAcceleration(Vector newAcc);
};

