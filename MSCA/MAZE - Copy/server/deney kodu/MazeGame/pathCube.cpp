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
	transf = new SoTransform();
    transfn=new SoTransform();
	shape = new SoCube();
    ips=new SoSphere();
	mat = new SoMaterial();
	//mat1 = new SoMaterial();
	//mat2 = new SoMaterial();
	mat->diffuseColor.setValue(color);
	//mat1->diffuseColor.setValue(COLOR_CIP);
	//mat2->diffuseColor.setValue(COLOR_HIP);

	SoSeparator *BSep = new SoSeparator;
	root->addChild(BSep);
	ballWidth=width-0.05;
	ballDepth=depth-0.05;
	ballHeight=height;
	shape->width.setValue(width);//cigil
	shape->height.setValue(height);
	shape->depth.setValue(depth);//cigil
	ips->radius.setValue(IP_RADIUS);
    transf->translation.setValue(Point(2*(BALL_WIDTH*0.5+IP_RADIUS),0,0));
	transfn->translation.setValue(Point(-(BALL_WIDTH*0.5+IP_RADIUS),0,0));
	transfMat->translation.setValue(position);

	BSep->addChild(transfMat);
	BSep->addChild(mat);
	BSep->addChild(shape);
	BSep->addChild(transfn);
	//BSep->addChild(mat2);//ARKADAKÝ CIP
	BSep->addChild(ips);
	BSep->addChild(transf);
	//BSep->addChild(mat1);//(CIP)
	//BSep->addChild(ips);
}

void pathCube::setTranslate(Vector tr)
{
	transfMat->translation.setValue(tr);
}

void pathCube::setRotation(float angg)
{
	transfMat->rotation.setValue(SbVec3f(0,1,0),angg);
}

void pathCube::setWidth(float w)
{
	shape->width.setValue(w);
}

void pathCube::setDepth(float w)
{
	shape->depth.setValue(w);
}

void pathCube::setHeight(float w)
{
	shape->height.setValue(w);
}
void pathCube::setHandleRadius(float w)
{
	ips->radius.setValue(w);
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