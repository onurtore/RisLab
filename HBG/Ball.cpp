#pragma once
#include "Ball.h"

Ball::Ball()
{
	posX = 0;		posY = 0;		posZ = 0;
	prevPosX = 0;	prevPosY = 0;	prevPosZ = 0;
	mass = 1;
	velX = 0;		velZ = 0;	
	accX	= 0;	accZ = 0; 
	radius = 1;
	fOnMe = Vector(0,0,0);
}

Ball::Ball(Point position, float radius, float mass)
{
	posX = position[0];	posY = position[1];	posZ = position[2];	
	prevPosX = posX;	prevPosY = posY;	prevPosZ = posZ;
	this->mass = mass;
	velX = 0;	velY = 0;	velZ = 0;
	accX = 0;	accY = 0;	accZ = 0; 
	this->radius = radius;
	fOnMe = Vector(0,0,0);
}

bool Ball::incrementPosition(Point newPos)
{
	return setPosition( Point( posX + newPos[0],
							   posY + newPos[1],
							   posZ + newPos[2]) ) ;
}

bool Ball::setPosition(Point newPos, bool allowJumps)
{
	// prevent sudden jumps
	if (!allowJumps)
	{
		if ( abs ( ( long) (newPos[0] - posX) )  <= radius &&  abs ( (long) (newPos[2] - posZ) )  <= radius )
		{
			prevPosX = posX; 
			prevPosY = posY; 
			prevPosZ = posZ; 

			posX = newPos[0];
			posY = newPos[1];
			posZ = newPos[2];
			return true;
		}
		else
		{
			//printf("set position error\n" );
			return false;
		}
	}
	else
	{
		prevPosX = posX; 
		prevPosY = posY; 
		prevPosZ = posZ; 

		posX = newPos[0];
		posY = newPos[1];
		posZ = newPos[2];
		return true;
	}
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

float Ball::getRadius()
{
	return radius;
}

float Ball::getMass()
{
	return mass;
}

Vector Ball::getFInertia()
{
	return Ball::getAcceleration() * Ball::getMass();
}