#pragma once
#include "Public.h"
#include "HapticInterfacePoint.h"
#include "MathCB.h"

// define obstacle types

class  Obstacle
{
public:
	Obstacle(int level);
	Obstacle(Point pos, float width, float depth, int level, float angle = 0);

	Point getPosition();
	float getWidth();
	float getDepth();
	float getAngle();
	
	float getTunnelWidthFactor();
	int   getLevel();
	void  setLevel(int level);

	float distanceSq(Point pt); 
	float distance(Point pt); 

private:
	float posX;
	float posY;
	float posZ;
	float width;
	float depth;
	float angle;
	float tunnelWidthFactor;
	int	  level;
};
