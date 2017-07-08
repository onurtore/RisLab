#include "HapticInterfacePoint.h"


HapticInterfacePoint::HapticInterfacePoint(Point pos)
{
	posX = pos[0];	posY = pos[1];	posZ = pos[2];	
	mass = 1;
	velX = 0.0f;	velY = 0.0f; velZ = 0.0f;	
	windowVelX = 0.0f;  windowVelZ = 0.0f;
	
	accX = 0.0f;	accY = 0.0f;	accZ = 0.0f; 
	prevPosX = 0.0f; prevPosZ = 0.0f;
	nextPosX = 0.0f; nextPosZ = 0.0f;
	radius = 3;
	fOnMe = Vector(0,0,0);
}

void HapticInterfacePoint::updatePosition(Point newPos,int runs)
{	
	//INTERPOLATION
	/*
	int interval = 10; 
	if(runs % interval == 0)
	{
	 prevPosX = newPos[0];
	 prevPosZ = newPos[2];
	}
	else if(runs % interval== 9)
	{
	nextPosX =	newPos[0];
	nextPosZ=  newPos[2];
	}
		
	velX = prevPosX + ((nextPosX - prevPosX)/interval*fmod(runs,10.0)) ;
	velZ = prevPosZ + ((nextPosZ - prevPosZ)/interval*fmod(runs,10.0));
	*/
	
	
	int interval = 10; 
	
	if (runs < interval)
	{
		nextPosX = prevPosX;
		nextPosZ = prevPosZ;
	}
	else
	{
		if(runs % interval == 0)
		{
			
			prevPosX = nextPosX;
			prevPosZ = nextPosZ;

			nextPosX =	newPos[0];
			nextPosZ =  newPos[2];
		}
	}
		
	
	windowVelX = (nextPosX - prevPosX) / (DELTA_T * interval);
	windowVelZ = (nextPosZ - prevPosZ) / (DELTA_T * interval);

	float prevVelX = velX;
	float prevVelZ = velZ;
	
	velX = windowVelX;//(newPos[0] - posX)/DELTA_T;;
	velZ = windowVelZ;//(newPos[2] - posZ)/DELTA_T;;

	//accX = velX - prevVelX;
	//accZ = velZ - prevVelZ;
	
	posX = newPos[0];
	posZ = newPos[2];

}

void HapticInterfacePoint::updateVelocity(Vector newVel)
{	
	float prevVelX = velX;
	float prevVelZ = velZ;
	
	velX = newVel[0];
	velZ = newVel[2];

	accX = velX - prevVelX;
	accZ = velZ - prevVelZ;
}

void HapticInterfacePoint::updateAcceleration(Vector newAcc)
{
	accX = newAcc[0];
	accZ = newAcc[2];
}
Vector HapticInterfacePoint::getWindowVel()

{
	return Vector(windowVelX,0,windowVelZ);
}