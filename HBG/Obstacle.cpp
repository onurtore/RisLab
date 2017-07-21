#pragma once
#include "Obstacle.h"

Obstacle::Obstacle(Point pos, float width, float depth, int level, float angle)
{
	posX = pos[0];	posY = pos[1];	posZ = pos[2];

	this->width = width;
	this->depth = depth;
	this->angle = angle;

	this->level = level;
	setLevel(level);
}

Obstacle::Obstacle(int level)
{
	// defaults
	Obstacle(Point(POSZERO,POSZERO,POSZERO), OBSTACLE_WIDTH, OBSTACLE_DEPTH, level, 0);
}


Point Obstacle::getPosition()
{
	return Point(posX, posY, posZ);
}


float Obstacle::getWidth()
{
	return width;
}

float Obstacle::getDepth()
{
	return depth;
}

float Obstacle::getAngle()
{
	return angle;
}

float Obstacle::getTunnelWidthFactor()
{
	return tunnelWidthFactor;
}


int Obstacle::getLevel()
{
	return level;
}

void Obstacle::setLevel(int level)
{
	this->level = level;
	
	if (level == LEVEL_EASY)
		tunnelWidthFactor = 0.50;//0.40;
	else if (level == LEVEL_NORMAL)
		tunnelWidthFactor = 0.43;//0.38;
	else if (level == LEVEL_HARD)
		tunnelWidthFactor = 0.38;//0.35;
	else if (level == LEVEL_HARDEST)
		tunnelWidthFactor = 0.30;//0.31;
	else
	{
		setLevel(LEVEL_DEFAULT);
		this->level = LEVEL_DEFAULT;
	}
}

//TODO: correct this, assumes sphere and point
float Obstacle::distance(Point pt) {

	Point posEnd( posX + width*cos(TO_RADIANS(angle)), 0, posZ - width*sin(TO_RADIANS(angle)) );
	Point posMid( (posX+posEnd[0])/2, 0, (posZ+posEnd[2])/2 );

	//	float d = EUC_DIST(posX, posZ, pt[0], pt[2]) - (width + 0);
	float d = abs( (posX - posEnd[0])*(posEnd[2] - pt[2]) - (posEnd[0] - pt[0])*(posZ-posEnd[2]) ) /
			  EUC_DIST(posX, posZ, posEnd[0], posEnd[2]);

	if (EUC_DIST(pt[0], pt[2], posMid[0], posMid[2]) > width)
		d = EUC_DIST(pt[0], pt[2], posMid[0], posMid[2]);

	return d > 0?  d: 0.0000001;
	// distance = dot product ( plane.normal , sphere.pos )
}

float Obstacle::distanceSq(Point pt) {
	float d = distance(pt);
	return SQUARE(d);
}