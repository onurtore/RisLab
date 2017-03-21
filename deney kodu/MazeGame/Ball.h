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
	float posX, posY, posZ;
	float velX, velY, velZ;
	float accX, accY, accZ;
	float angle, angVel, angAcc;
	SbMatrix transMatX, transMatZ;
	

	float boundaryThickness;
	float Ibody;
    Vector boundary;

	//CIGIL
	bool ballCollision;
	bool angularCollision;


	Ball();
	Ball(Point position, float width,float height,float depth,float radius, float mass);
	bool setPosition(Point newPos);
	bool incrementPosition(Point newPos);
	void setVelocity(Vector newVel);
	void setAcceleration(Vector newAcc);
	void setAngVelocity(float vel);
	void setAngle(float newAngle);
	void setAngAcceleration(float newAngAcc);

	Point getPosition();
	Vector getVelocity();
	Vector getAcceleration();
	float getAngVelocity();
	

	float getWidth();
	float getHeight();
	float getDepth();
	float getMass();

	float getRadius();
	float calculateInertia();

	
	//CIGIL
	bool isCollide();
	void setCollision(bool collision);

	bool isAngleCollide();
	void setAngleCollision(bool collision);


	float calculateAngleAcc(Vector f1,Vector f2);
	float calculateAngleVel(Vector f1,Vector f2);
	float getAngle();
	float getAngleVel();
	float getAngleAcc();
	void  calculateAngle(Vector f1,Vector f2);
	float calculateMoment(Vector f1,Vector f2);
    
private:
	float mass;
	float radius;
	float width;
	float height;
	float depth;
	Vector fOnMe;


};

