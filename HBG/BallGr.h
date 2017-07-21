#pragma once
//#include "Public.h"
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/SbBasic.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoMaterial.h>
#include "Ball.h"


typedef SbVec3f Point;
typedef SbVec3f Vector;

class  BallGr
{
public:

	Ball *ball;
	SoTransform *transfMat;
	SbMatrix *transMat;
	SbMatrix *rotateMat;
	SoSphere *shape;
	SoMaterial *mat;

	BallGr();
	BallGr(SoSeparator *root, Point position, float radius, float mass, Vector color);

	void setTranslate(Vector tr);
	void setPosition(Point pt, bool allowJumps = false);
	void incrementPosition(Point pt);
	void setVelocity(Vector vel);
	void setAcceleration(Vector vel);

	Point getPosition();
	Vector getVelocity();
	Vector getAcceleration();
	float getRadius();
	float getMass();
	Ball* getBall();
	Vector getFInertia();
};
