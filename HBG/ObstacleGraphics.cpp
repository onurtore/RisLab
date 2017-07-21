#pragma once
#include "ObstacleGraphics.h"

ObstacleGraphics::ObstacleGraphics(int level)
{
	// defaults
	obs = new Obstacle(level);
}

ObstacleGraphics::ObstacleGraphics(SoSeparator *root, Point pos, int level, 
								   float angle, float w, float d)
{
	obs					= new Obstacle(pos, w, d, level, angle);
	
	SoSeparator *obsSep	= new SoSeparator;
	transfMat			= new SoTransform;
	mat1				= new SoMaterial;
	mat2				= new SoMaterial;
	trans1				= new SoTransform;
	trans2				= new SoTransform;

	for (int i = 0; i < NUM_OBS_CORNERS; i++)
		obstacleCornerPts[i] = Point(0,0,0);
	
	obsCornersDrawStyle  = new SoDrawStyle;

	cornerMat = new SoMaterial;

	cornerMat->diffuseColor.setValue(COLOR_TARGET_OFF);//(1,0,0);
	cornerMat->transparency = 0.5;

	shape = constructShape();

	transfMat->translation.setValue(pos);
	transfMat->rotation.setValue(SbVec3f (0, 1, 0), TO_RADIANS(angle) );
 
	obsSep->addChild(transfMat);
	obsSep->addChild(shape);

	// add cylinder to root
	root->addChild(obsSep);
}

ObstacleGraphics::~ObstacleGraphics()
{
	delete obs;
}


SoSeparator* ObstacleGraphics::constructShape()
{  
	SoSeparator* sep	= new SoSeparator;

	mat1->transparency = 0.7;
	mat1->ambientColor.setValue(OBSTACLE_COLOR);
	mat2->transparency = 0.7;
	mat2->ambientColor.setValue(OBSTACLE_COLOR);

	SoSeparator* boundSep1	= new SoSeparator;
	SoCube* bound1			= new SoCube;
	
	SoSeparator* boundSep2	= new SoSeparator;
	SoCube* bound2			= new SoCube;

	float toWall = BOARD_WIDTH / 2.0f - abs(obs->getPosition()[0]);

	trans1->translation.setValue(obs->getWidth()/2.0f - toWall, 0, -obs->getDepth() * obs->getTunnelWidthFactor());
	bound1->height.setValue(OBSTACLE_HEIGHT/2);
	bound1->width.setValue(obs->getWidth()*1.1);
	bound1->depth.setValue(OBSTACLE_HEIGHT);

	trans2->translation.setValue(obs->getWidth()/2.0f - toWall, 0, obs->getDepth() * obs->getTunnelWidthFactor());
	bound2->height.setValue(OBSTACLE_HEIGHT/2);
	bound2->width.setValue(obs->getWidth()*1.1);
	bound2->depth.setValue(OBSTACLE_HEIGHT);

	boundSep1->addChild(trans1);
	boundSep1->addChild(mat1);
	boundSep1->addChild(bound1);
	boundSep2->addChild(trans2);
	boundSep2->addChild(mat2);
	boundSep2->addChild(bound2);

	sep->addChild(boundSep1);
	sep->addChild(boundSep2);

	return sep;
}



void ObstacleGraphics::changeColor(float r, float g, float b, int whichBound)
{
	if (whichBound == 0)
		mat2->ambientColor.setValue(r, g, b);
	else if (whichBound == 1)
		mat1->ambientColor.setValue(r, g, b);
}

float ObstacleGraphics::getWidth()
{
	return obs->getWidth();
}

float ObstacleGraphics::getDepth()
{
	return obs->getDepth();
}

float ObstacleGraphics::getAngle()
{
	return obs->getAngle();
}

int ObstacleGraphics::getLevel()
{
	return obs->getLevel();
}

void ObstacleGraphics::setLevel(int level)
{
	obs->setLevel(level);
}


void ObstacleGraphics::rotate(float angle) // rotate obstacle around y in cw by angle radians
{
	transfMat->rotation.setValue(0, 1, 0, angle /*TO_RADIANS(57.5)*/);
}

float ObstacleGraphics::distanceSq(Point pt) {
	return obs->distanceSq(pt);
}

float ObstacleGraphics::distance(Point pt) {
	float d = obs->distance(pt);
	return d > 0?  d : 0.0000001;
}

Point ObstacleGraphics::getPosition() {
	return obs->getPosition();
}