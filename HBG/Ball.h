#pragma once

#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/SbBasic.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoMaterial.h>
#include "MathCB.h"

typedef SbVec3f Point;
typedef SbVec3f Vector;

class  Ball
{
public:
	float posX, posY, posZ, velX, velY, velZ, accX, accY, accZ;
	float prevPosX, prevPosY, prevPosZ;
	SbMatrix transMatX, transMatZ;

	Ball();
	Ball(Point position, float radius, float mass);
	bool setPosition(Point newPos, bool allowJumps = false);
	bool incrementPosition(Point newPos);
	void setVelocity(Vector newVel);
	void setAcceleration(Vector newAcc);

	Point getPosition();
	Vector getVelocity();
	Vector getAcceleration();
	float getRadius();
	float getMass();
	Vector getFInertia();

private:
	float mass;
	float radius;

	Vector fOnMe;

};

