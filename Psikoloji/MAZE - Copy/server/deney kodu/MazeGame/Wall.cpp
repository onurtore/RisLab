#include "GraphicsController.h"
#include "wall.h"
#include "Public.h"

Wall::Wall(float wH, float wW, float wD, float wTx, float wTy, float wTz)
{
	wall_Width = wW;
    wall_Height = wH;
	wall_Depth = wD;
	
	tx = wTx;
	tz = wTz;
	ty = wTy;


	wallMat=new SoMaterial();
	hitW=false;

	wallShape = constructWallShape();
}


SoSeparator* Wall::constructWallShape()
{
	SoSeparator* wallShape = new SoSeparator;

	//SoMaterial *wallMat = new SoMaterial;
	wallMat ->diffuseColor.setValue(COLOR_BOARD);
	wallMat ->transparency.setValue(1.0f);
	
	SoCube *wallCube = new SoCube;
    SoTransform *wallTrans = new SoTransform;

	wallTrans->translation.setValue(tx,ty,tz);
	
	wallCube->height.setValue(wall_Height);
	wallCube->width.setValue(wall_Width);//-3.5);
	wallCube->depth.setValue(wall_Depth);//-1);

    
	wallShape->addChild(wallTrans);
	wallShape->addChild(wallMat);
	wallShape->addChild(wallCube);
    
	return wallShape;
}

void Wall::setHit(bool hitOrMiss)
{
	hitW=hitOrMiss;
}
bool Wall::isHit()
{
	return hitW;
}