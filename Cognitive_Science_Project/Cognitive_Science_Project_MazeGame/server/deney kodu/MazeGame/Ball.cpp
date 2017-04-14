#pragma once
#include "Ball.h"

Ball::Ball()
{
	posX = 0.0f;		posY = 0.0f;		posZ = 0.0f;
	
	mass = BALL_MASS;
	velX = 0.0f;	velY = 0.0f;	velZ = 0.0f;
	accX = 0.0f;	accY = 0.0f;	accZ = 0.0f; 
	angle  = 0.0f;
	angVel = 0.0f;
	angAcc = 0.0f;
	
	radius = 1;
	width = BALL_WIDTH;
	height = BALL_HEIGHT;
	depth = BALL_DEPTH;
	boundaryThickness = BOARD_THC;

	//CIGIL
	ballCollision = false;
	angularCollision = false;

	boundary[0] = BOARD_WIDTH	*0.5+boundaryThickness;
	boundary[1] = BOARD_WIDTH*0.5+boundaryThickness;
	boundary[2] = BOARD_WIDTH*0.5+boundaryThickness;

	fOnMe = Vector(0,0,0);
}

Ball::Ball(Point position, float width,float height,float depth,float radius, float mass)
{
	posX = position[0];	posY = position[1];	posZ = position[2];	
	angle=0.0f;
	angVel=0.0f;
	angAcc=0.0f;
   	boundaryThickness=BOARD_THC;

	boundary[0] = BOARD_WIDTH * 0.5 + boundaryThickness;
	boundary[1] = BOARD_WIDTH * 0.5 + boundaryThickness;
	boundary[2] = BOARD_WIDTH * 0.5 + boundaryThickness;

	this->mass = mass;
	velX = 0.0f;	velY = 0.0f;	velZ = 0.0f;
	accX = 0.0f;	accY = 0.0f;	accZ = 0.0f; 

	this->width= width;
	this->height=height;
	this->depth=depth;
	this->radius=radius;
	fOnMe = Vector(0,0,0);
}

bool Ball::incrementPosition(Point newPos)
{
	return setPosition( Point(  posX + newPos[0],
								posY + newPos[1],
								posZ + newPos[2]) ) ;
}

bool Ball::setPosition(Point newPos)
{

	//prevent sudden jumps

	if (abs(newPos[0] - posX) <= radius &&  abs(newPos[2] - posZ) <= radius) {


		posX = newPos[0];
		posY = newPos[1];
		posZ = newPos[2];
	}
	else {

		//printf("set position error\n" );
		return false;
	}


	return true;
}


void Ball::setAngle(float newAngle)
{
	//float prevAngle = angle; 
	//float prevAngV = angVel;
	
	angle = fmod(newAngle,(2*PI));
}


void Ball::setVelocity(Vector newVel)
{
	velX = newVel[0];
	velY = newVel[1];
	velZ = newVel[2];
}

void Ball::setAcceleration(Vector newAcc)
{
	accX = newAcc[0];
	accY = newAcc[1];
	accZ = newAcc[2];
}

void Ball::setAngVelocity(float vel)
{
	angVel = vel;
}

void Ball::setAngAcceleration(float newAngAcc)
{ 
	angAcc = newAngAcc;
}

Point Ball::getPosition()
{
	return Point(posX, posY, posZ);
}

Vector Ball::getVelocity()
{
	return Vector(velX, velY, velZ);
}


Vector Ball::getAcceleration()
{ 
	return Vector(accX, accY, accZ);
}

float Ball::getAngVelocity()
{
	return angVel;
}




float Ball::getWidth()
{
	return width;
}

float Ball::getHeight()
{
	return height;
}
float Ball::getDepth()
{
	return depth;
}
float Ball::getMass()
{
	return mass;
}
float Ball::getRadius()
{
	return radius;
}
float Ball::calculateInertia()
{
	float X0=this->width;
	float Y0=this->height;
	float Z0=this->depth;
	float M=this->mass;
	Ibody= (M/12) * (X0*X0+Z0*Z0);//*1000;

	return Ibody;
}



float Ball::calculateMoment(Vector f1,Vector f2)
{   
	float Mom;
	Mom=0.0f;
	
	Mom = ( f1[2]*BALL_WIDTH*0.5*cos(angle)+f1[0]*BALL_WIDTH*0.5*sin(angle)) +
		  (-f2[2]*BALL_WIDTH*0.5*cos(angle)-f2[0]*BALL_WIDTH*0.5*sin(angle));
	float MomResistance=0.0f;

	// apply rotational friction
	 if(abs(angVel)== 0.0f)
	{
		MomResistance = calculateInertia() * COEFFICIENT_FRICTION_ROT_STATIC *SSIGN(Mom);

		Mom -= MomResistance;
 
	}
	else if(abs(angVel) > 0.0f)
	{		
		MomResistance = calculateInertia() * COEFFICIENT_FRICTION_ROT * SSIGN(angVel);

		Mom -= MomResistance;
	}
	
	return Mom;
}

float Ball::calculateAngleAcc(Vector f1,Vector f2)//f1 for cip
{
	angAcc = Ball::calculateMoment(f1,f2)/Ball::calculateInertia();
	angAcc = fmod(angAcc,(2*PI));
	return angAcc;
}


float Ball::calculateAngleVel(Vector f1,Vector f2)
{
	float angleV = 0.0f;
	angleV = angVel + (DELTA_T*(Ball::calculateAngleAcc(f1,f2)));
	angVel = fmod(angleV,(2*PI));
	return angVel;
}

void Ball::calculateAngle(Vector f1,Vector f2)
{
	float angleV = Ball::calculateAngleVel(f1,f2);
	angle = angle + (DELTA_T * angleV) + calculateAngleAcc(f1,f2) * 0.5 * pow(DELTA_T, 2);
	angle = fmod(angle, (2*PI));
}


float Ball::getAngle()
{
	return angle;
}
float	Ball::getAngleVel()
{
	return angVel;
}
float Ball::getAngleAcc()
{
	return angAcc;
}
//CIGIL
bool Ball::isCollide()
{
	return ballCollision;
}

//CIGIL

void Ball::setCollision(bool collision)
{
	ballCollision = collision;
}

bool Ball::isAngleCollide()
{
	return angularCollision;
}
void Ball::setAngleCollision(bool collision)
{
	angularCollision=collision;
}