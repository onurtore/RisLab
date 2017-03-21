#pragma once
#include "Public.h"
#include <Inventor/SbBasic.h>

class  Wall
{
public:
	// translation values
	float tx;
	float tz;
	float ty;

	float wall_Height;
	float wall_Width;
	float wall_Depth;
	
	bool hitW; // true if the wall is hit by the ball
	
	SoSeparator *wallShape;
	SoMaterial *wallMat;

	Wall(float wH, float wW, float wD,float tx,float ty,float tz);

	SoSeparator * constructWallShape();
	void setHit(bool hitOrMiss);
	bool isHit();	
};