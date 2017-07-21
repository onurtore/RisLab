#pragma once
#include "HapticInterfacePoint.h"
#include "Public.h"

typedef SbVec3f Point;
typedef SbVec3f Vector;

class  hIPgraphics
{
public:

	HapticInterfacePoint *ip;
	SoTransform *transfMat;
	SbMatrix *transMat;
	SbMatrix *rotateMat;
	SoSphere *shape;
	SoMaterial *mat;

	hIPgraphics();
	hIPgraphics(SoSeparator *root, 
				Point pos = Point(0,0,0), 
				float radius = 2.0f, 
				Vector color = COLOR_NIP);


	Point getPosition();
	Vector getVelocity();
	Vector getAcceleration();
	float getRadius();

	void setTranslate(Vector tr);
	void setPosition(Point pt);
	void setVelocity(Vector vel);
	void setAcceleration(Vector vel);
	void setRadius(float r);
};
