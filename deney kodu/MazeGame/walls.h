#pragma once
#include "Public.h"
#include <Inventor/SbBasic.h>

class  walls
{
public:
    int  numWalls;
	SoSeparator *wallsShape;
	for (int i=0;i<numWalls;i++){
	wallsShape->addChild(wall);
	}

	
	

	
	
};