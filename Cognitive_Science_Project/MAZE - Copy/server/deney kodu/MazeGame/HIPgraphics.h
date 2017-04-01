#pragma once
#include "HapticInterfacePoint.h"
#include "Public.h"


class  HIPgraphics
{
public:

	HapticInterfacePoint *ip;
	SoTransform *transfMat;
	SbMatrix *transMat;
	SbMatrix *rotateMat;
	SoSphere *shape;
	SoMaterial *mat;

	HIPgraphics();
	HIPgraphics(SoSeparator *root, 
				Point pos= Point(0,0,0), 
				float radius = 2.0f, 
				Vector color = COLOR_NIP,float trans=transhc);


	Point getPosition();
	Vector getVelocity();
	Vector getAcceleration();
	float getRadius();

	void setTranslate(Vector tr);
	void setPosition(Point pt, int runs);
	void setVelocity(Vector vel);
	void setAcceleration(Vector vel);
	void setRadius(float r);
	
};
