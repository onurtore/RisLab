#pragma once
#include "Public.h"
#include <Inventor/SbBasic.h>

class  BoardGraphics
{
public:

	Vector boundary;
	float boundaryThickness;
	float boundaryDim;
	float boundaryHeight;
	float floorHeight;
	float floorWidth;
	float floorDepth;

	SoSeparator *shape;

	BoardGraphics(float bT, float bD, float bH);

	SoSeparator * constructBoardShape();
	Point getPosition();
};
