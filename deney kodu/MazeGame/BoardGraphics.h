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

	int targetCount;
	
	

	Vector boardBoundaryColor;
	bool hitB;
	bool arrived;
	bool hitObstacle;


	int trialBoardGame;
	SoTransform *targetTrans;

	SoSeparator *shape;
	/*SoSeparator *boardBoundaryRoot1; 
	SoSeparator *boardBoundaryRoot2; 
	SoSeparator *boardBoundaryRoot3;
	SoSeparator *boardBoundaryRoot4; 
	*/
	SoMaterial *boardBoundaryMat;
	SoMaterial *boardMat;
	SoMaterial *targetMat; 
	SoMaterial *ObstacleMat;
	bool initialization;
	
	BoardGraphics(float bT, float bD, float bH, int scenario, int cond);

	SoSeparator * constructBoardShape(int scenario, int cond);
	SoSeparator * constructBoardShape4Straight();
	SoSeparator * constructBoardShape4Rotational(int cond);
	SoSeparator * constructBoardShape4Mixed();

	Point getPosition();
	void setHit(bool hitOrMiss);
	void setArrived(bool arrivedOrNot);
	
	bool isHit();
	bool isArrived();
	void setTrial(int t);
	int getTrial();
	bool isHitObstacle();
	void setHitObstacle(bool x);

	void setInit(bool seti);
	bool getInit();
};