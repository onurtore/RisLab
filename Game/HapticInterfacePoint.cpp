/**********************************************************************
Onur Berk Töre
Yeditepe University, RIS Lab
**********************************************************************/
#include "HapticInterfacePoint.h"


HapticInterfacePoint::HapticInterfacePoint(Point pos, Vector vel, Vector acc)
{
	posX = pos[0];	posY = pos[1];	posZ = pos[2];	
	mass = 1;
	velX = vel[0];	velY = vel[1]; velZ = vel[2];	
	accX = acc[0];	accY = acc[1];	accZ = acc[2]; 

	radius = 3;

}

void HapticInterfacePoint::updatePosition(Point newPos)
{
	posX = newPos[0];
	posY = newPos[1];
	posZ = newPos[2];
}

void HapticInterfacePoint::updateVelocity(Vector newVel)
{
	velX = newVel[0];
	velY = newVel[1];
	velZ = newVel[2];
}

void HapticInterfacePoint::updateAcceleration(Vector newAcc)
{
	accX = newAcc[0];
	accX = newAcc[1];
	accZ = newAcc[2];
}

Point HapticInterfacePoint::getPos(){
	Point hapticPos;
	hapticPos[0] = posX;
	hapticPos[1] = posY;
	hapticPos[2] = posZ;
	return hapticPos;
}