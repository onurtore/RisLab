/**********************************************************************
Onur Berk Töre
Yeditepe University, RIS Lab
**********************************************************************/
#include "CubeGr.h"
#include <Inventor/nodes/SoCube.h>
#include "Public.h"
#include <vector>

CubeGr::CubeGr(SoSeparator *root, Point position, float width,float height,float depth,float radius, float mass, Vector color){

	shape = new SoCube();
	transform1 = new SoTransform();

	posX = position[0];
	posY = position[1];
	posZ = position[2];

	material1 = new SoMaterial();
	material1->diffuseColor.setValue(color);

	SoSeparator *CubeSeparator = new SoSeparator;
	root->addChild(CubeSeparator);
	
	shape->width.setValue(width-0.05);// -0.05 Çünkü Çýðýl Böyle istemiþ
	shape->height.setValue(height);
	shape->depth.setValue(depth-0.05);// -0.05 Çünkü Çýðýl böyle istemiþ
	
	transform1->translation.setValue(position);
	
	CubeSeparator->addChild(transform1);
	CubeSeparator->addChild(material1);
	CubeSeparator->addChild(shape);

}


SoCube* CubeGr::getCube(){

	return shape;

}

Point CubeGr::getPos(){

	return Point(posX, posY, posZ);
	
}

Vector CubeGr::getVel(){

	return Vector(velX,velY,velZ);
	
}

Vector CubeGr::getAcc(){
	
	return Vector(accX,accY,accZ);
}

void CubeGr::setTranslate(Vector tr){
	transform1->translation.setValue(tr);
}

void CubeGr::setRotation(float angle){
	transform1->rotation.setValue(SbVec3f(0,1,0),angle);
}

void CubeGr::setPosition(Point pt){
	posX = pt[0];
	posY = pt[1];
	posZ = pt[2];
}

float CubeGr::getAngle(){

}

float CubeGr::calculateAngle(Vector f1,Vector f2){

}

float CubeGr::getAngleVel(Vector f1,Vector f2){


}

float CubeGr::getAngleAcc(f1,f2){

}

void CubeGr::getMoment(f1,f2){


}

void CubeGr::getInertia(){


}