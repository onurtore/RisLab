#pragma once
#include "Public.h"
#include <Inventor/SbBasic.h>

class  wall
{
public:
	Vector wallT;
	float tx;
	float wallTy;
	float tz;
	float ty;
	float wall_Height;
	float wall_Width;
	float wall_Depth;
	bool hitW;
	

	SoSeparator *wallShape;
	SoMaterial *wallMat;
	

	wall(float wH, float wW, float wD,float tx,float ty,float tz);

	SoSeparator * constructWallShape();
	void setHit(bool hitOrMiss);
	bool isHit();
	
};