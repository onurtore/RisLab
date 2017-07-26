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
	Vector boardBoundaryColor;
	bool hitB;
	bool arrived1;
	bool arrived2;

	SoSeparator *shape;
	/*SoSeparator *boardBoundaryRoot1; 
	SoSeparator *boardBoundaryRoot2; 
	SoSeparator *boardBoundaryRoot3;
	SoSeparator *boardBoundaryRoot4; 
	*/
	SoMaterial *boardBoundaryMat;
	SoMaterial *targetMat1; 
	SoMaterial *targetMat2; 
	
	BoardGraphics(float bT, float bD, float bH);

	SoSeparator * constructBoardShape();
	//void setColor(Vector colorb);
	//void setColorVector(Vector cb);
	//Vector getColorVector();
	Point getPosition();
	void setHit(bool hitOrMiss);
	void setArrived1(bool arrivedOrNot);
	void setArrived2(bool arrivedOrNot);

	bool isHit();
	bool isArrived1();
	bool isArrived2();
	
};