/*
Yeditepe University - RIS Laboratary 

Authors : Onur Berk Töre

Below code written for showing cube on the path while
playing the game.
*/



#include "pathCube.h"

pathCube::pathCube(SoSeparator *root, Point position, float width,float height,float depth,float radius, float mass, Vector color)
{

	posX = position[0];
	posY = position[1];
	posZ = position[2];

	transfMat = new SoTransform();
	transf    = new SoTransform();
	transfn   = new SoTransform();

	shape     = new SoSphere();
	
	mat       = new SoMaterial();

	mat->diffuseColor.setValue(color);

	SoSeparator *BSep  = new SoSeparator;

	root->addChild(BSep);

	shape->radius.setValue(1);

	
    transf->translation.setValue(Point(2*(BALL_WIDTH*0.5+IP_RADIUS),0,0));
	transfn->translation.setValue(Point(-(BALL_WIDTH*0.5+IP_RADIUS),0,0));
	transfMat->translation.setValue(position);

	BSep->addChild(transfMat);
	BSep->addChild(mat);
	BSep->addChild(shape);
	BSep->addChild(transfn);
	BSep->addChild(transf);


}


void pathCube::setTranslate(Vector tr)
{
	transfMat->translation.setValue(tr);
}

void pathCube::setRotation(float angg)
{
	transfMat->rotation.setValue(SbVec3f(0,1,0),angg);
}






void pathCube::setPosition(float x,float z){

	posX = x;
	posZ = z;
}

float  pathCube::getPositionX(){

	return posX;

}

float  pathCube::getPositionZ(){

	return posZ;

}