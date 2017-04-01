#pragma once

#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/SbBasic.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoMaterial.h>
#include "MathCB.h"
#include "Public.h"


class  Ball
{
public:
	float posX, posY, posZ, velX, velY, velZ, accX, accY, accZ;
	float prevPosX, prevPosY, prevPosZ;
	float prevVelX,prevVelY,prevVelZ;
	SbMatrix transMatX, transMatZ;
	float angle,angVel,angAcc;
	float prevAngle;
	float prevAngV;
    float boundaryThickness;
	float Ibody;
	Vector Ballpoints1,Ballpoints2,Ballpoints3,Ballpoints4;
    Vector boundary;
	Ball();
	Ball(Point position, float width,float height,float depth,float radius, float mass);
	bool setPosition(Point newPos, bool allowJumps = false);
	bool incrementPosition(Point newPos);
	void setVelocity(Vector newVel);
	void setAcceleration(Vector newAcc);
	void Ball::setAngle(float newAngle);
	int runss;


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
	Vector getBallpoints1(Point position,float angle);
	Vector getBallpoints2(Point position,float angle);
	Vector getBallpoints3(Point position,float angle);
	Vector getBallpoints4(Point position,float angle);
    Vector checkBallPointColl(Vector p);
    Vector  setBallpoints1(Vector pp1,float angle);
    Vector  setBallpoints2(Vector pp2,float angle);
	Vector  setBallpoints3(Vector pp3,float angle);
	Vector  setBallpoints4(Vector pp4,float angle);
	//void checkBallPointsCol(float[4][3] Ballpoints);
	float getAngleAcc(Vector f1,Vector f2);
	float getAngleVel(Vector f1,Vector f2);
	float getAngle();
	float calculateAngle(Vector f1,Vector f2);
	float  getMoment(Vector f1,Vector f2);
    float getAngle_col(Vector f1,Vector f2);
private:
	float mass;
	float radius;
	float width;
	float height;
	float depth;
	Vector fOnMe;


};

