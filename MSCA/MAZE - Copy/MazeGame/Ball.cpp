#pragma once
#include "Ball.h"

Ball::Ball()
{
	posX = 0;		posY = 0;		posZ = 0;
	prevPosX = 0;	prevPosY = 0;	prevPosZ = 0;

	mass = 1;
	velX = 0;		velZ = 0;	
	prevVelX=0;		prevVelY =0;		prevVelZ=0;
	accX	= 0;	accZ = 0; 
	angle=0;
	angVel=0;
	angAcc=0;
	prevAngle=0;
	prevAngV=0;
	
	radius = 1;
	width=ball_width;
	height=ball_height;
	depth=ball_depth;
	boundaryThickness=BOARD_THC;
	 boundary[0]=BOARD_WIDTH	*0.5+boundaryThickness;
	boundary[1]=BOARD_WIDTH*0.5+boundaryThickness;
	boundary[2]=BOARD_WIDTH*0.5+boundaryThickness;
	Ballpoints1=Vector(posX-width*0.5*sin(angle)+depth*0.5*cos(angle),posY,posZ+width*0.5*cos(angle)+depth*0.5*sin(angle));
	Ballpoints2=Vector(posX-width*0.5*sin(angle)-depth*0.5*cos(angle),posY,posZ+width*0.5*cos(angle)-depth*0.5*sin(angle));
	Ballpoints3=Vector(posX+width*0.5*sin(angle)-depth*0.5*cos(angle),posY,posZ-width*0.5*cos(angle)-depth*0.5*sin(angle));
	Ballpoints4=Vector(posX+width*0.5*sin(angle)+depth*0.5*cos(angle),posY,posZ+width*0.5*cos(angle)+depth*0.5*sin(angle));

	fOnMe = Vector(0,0,0);
}

Ball::Ball(Point position, float width,float height,float depth,float radius, float mass)
{
	posX = position[0];	posY = position[1];	posZ = position[2];	
	prevPosX = posX;	prevPosY = posY;	prevPosZ = posZ;
	angle=angle;
	angVel=angVel;
	prevAngle=angle;
	prevAngV=angVel;
   	boundaryThickness=BOARD_THC;
	 boundary[0]=BOARD_WIDTH	*0.5+boundaryThickness;
	boundary[1]=BOARD_WIDTH*0.5+boundaryThickness;
	boundary[2]=BOARD_WIDTH*0.5+boundaryThickness;
	runss=0;
	this->mass = mass;
	velX = 0;	velY = 0;	velZ = 0;
	accX = 0;	accY = 0;	accZ = 0; 
	angVel=0;
	angAcc=0;
	this->width= width;
	this->height=height;
	this->depth=depth;
	this->radius=radius;
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
		if ( abs(newPos[0] - posX) <= radius &&  abs(newPos[2] - posZ) <= radius )
		{
			prevPosX = posX; 
			prevPosY = posY; 
			prevPosZ = posZ; 

			posX = newPos[0];
			posY = newPos[1];
			posZ = newPos[2];
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
	}

	float prevVelX = velX;
	float prevVelY = velY;
	float prevVelZ = velZ;

	velX = posX - prevPosX;
	velY = posY - prevPosY;
	velZ = posZ - prevPosZ;

	accX = velX - prevVelX;
	accY = velY - prevVelY;
	accZ = velZ - prevVelZ;
	return true;
}


void Ball::setAngle(float newAngle)
{
	// prevent sudden jumps
	newAngle=fmod(newAngle,(2*PI2));
		//if ( abs(newAngle - angle) <= (0.5*PI))
		//{
			prevAngle = angle; 
		
			angle = newAngle;
			
		//}
		//else
		//{   prevAngle=prevAngle;
		//	angle=angle;
		//}
	
	

    prevAngV = angVel;
	

	angVel = angle - prevAngle;
	
	angAcc= angVel - prevAngV;

	
}

void Ball::setVelocity(Vector newVel)
{
	/*float*/ prevVelX = velX;
	/*float*/ prevVelY = velY;
	/*float*/ prevVelZ = velZ;

	velX = newVel[0];
	velY = newVel[1];

	velZ = newVel[2];

	accX = velX - prevVelX;
	accY = velY - prevVelY;
	accZ = velZ - prevVelZ;

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

Vector Ball::getPrevVelocity()
{
	return Vector(prevVelX, prevVelY, prevVelZ);
}

Vector Ball::getPrevPosition()
{
	return Vector(prevPosX, prevPosY, prevPosZ);
}
Vector Ball::getAcceleration()
{ 
	return Vector(accX, accY, accZ);
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
float Ball::getInertia()
{
	float X0=this->width;
	float Y0=this->height;
	float Z0=this->depth;
	float M=this->mass;
	//M=25;
	Ibody= (M/12) * (X0*X0+Z0*Z0)*1000;//*4000;

	return Ibody;
}

Vector Ball::getFInertia()
{   Vector A=Ball::getAcceleration() * Ball::getMass();
return Ball::getAcceleration() * Ball::getMass();
}
float Ball::getMoment(Vector f1,Vector f2)
{   
	float Mom;
	Mom=0.0f;
	//Vector ff;
	//ff=getFInertia();
	Mom=(f1[2]*ball_width*0.5*cos(angle)+f1[0]*ball_width*0.5*sin(angle));
	Mom+=(-f2[2]*ball_width*0.5*cos(angle)-f2[0]*ball_width*0.5*sin(angle));
   
	return Mom;
}

float Ball::getAngleAcc(Vector f1,Vector f2)//f1 for cip
{
	float angleA;
	angleA=Ball::getMoment(f1,f2)/Ball::getInertia();
	// cout<<"Mom  "<< Ball::getMoment(f1,f2)<<"   inertia    "<< Ball::getInertia()<<endl;
	
	angAcc=angleA;
	return angleA;
}
float Ball::getAngleVel(Vector f1,Vector f2)
{
	float angleV;
	angleV=prevAngV+Ball::getAngleAcc(f1,f2);
	angVel=fmod(angleV,(2*PI2));
	return angVel;//angleV;
}
float Ball::getAngle()//(Vector f1,Vector f2)
{
	//float anglee;
	//anglee=fmod(( prevAngle+Ball::getAngleVel(f1,f2) ),(2*PI2));
	//angle=anglee;
	//cout<<angle<<" vel  "<<Ball::getAngleVel(f1,f2)<<endl;
	return angle;
	/*runss++;*/
	/*float ang_d;
	float cx,cz,hx,hz;
	cx=c[0];cz=c[2];hx=h[0];hz=h[2];
	float tanf;
	tanf=(hz-cz)/(hx-cx);
	//tanf=1/tanf;
	ang_d=atan2((hx-cx),(hz-cz));
	//ang_d=atan(tanf);
	return ang_d;*/
}
float Ball::calculateAngle(Vector f1,Vector f2)
{
	float angleCal;
	angleCal=fmod(( prevAngle+Ball::getAngleVel(f1,f2) ),(2*PI2));
	//angle=angleCal;
	//cout<<angle<<" vel  "<<Ball::getAngleVel(f1,f2)<<endl;
	return angleCal;
}
float Ball::getAngle_col(Vector f1,Vector f2)
{
	float anglee;
	anglee=prevAngle+Ball::getAngleVel(f1,f2);
	angle=fmod(anglee,(2*PI2));
	return anglee;
	
}