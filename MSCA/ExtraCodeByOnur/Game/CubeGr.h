/**********************************************************************
Onur Berk Töre
Yeditepe University, RIS Lab
**********************************************************************/
#pragma once
#include "Public.h"
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/SbBasic.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoCube.h>
#include "Public.h"
#include <vector>

class  CubeGr
{
public:

	CubeGr(SoSeparator *root, Point position, float width,float height,float depth,float radius, float mass, Vector color);
	SoCube * shape;

	SoTransform * transform1;
	SoMaterial  * material1;
	SoSeparator * CubeSeparator;


	float posX,posY,posZ;
	float velX,velY,velZ;
	float accX,accY,accZ;

	float prevPosX,prevPosY,prevPosZ;
	float prevVelX,prevVelY,prevVelZ;
	
	float angle,prevAngle;
	float angleVel,prevAngleVel;
	float angleAcc;

	float boundaryThickness;
	float inertiaBody;




	//Functions
	SoCube* getCube();

	Point getPos();
	Vector getVel();
	Vector getAcc();

	void setTranslate(Vector translate);
	void setRotation(float angle);
	void setPosition(Point pt); 
	

	Point getPosition();
	Point getPrevPosition();
	
	Vector getVelocity();
	Vector getPrevVelocity();

	Vector getAcceleration();

	float getWidth();
	float getHeight();
	float getDepth();
	float getMass();

	float getRadius();
	float getInertia();
	
	Vector getFInertia();


	float getAngleAcc(Vector f1,Vector f2);
	float getAngleVel(Vector f1,Vector f2);
	float getAngle();
	float calculateAngle();
	float getMoment(Vector f1, Vector f2);
	
private: 
	float mass;
	float radius;
	float width;
	float height;
	float depth;
	Vector fOnME;

};
	