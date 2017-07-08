#include "BallGr.h"

BallGr::BallGr(SoSeparator *root, Point position, float width,float height,float depth,float radius, float mass, Vector color)
{
	ball = new Ball(position, width,height,depth,radius,mass); 
    
	transfMat = new SoTransform();
	transf = new SoTransform();
    transfn=new SoTransform();
	shape = new SoCube();
    ips=new SoSphere();
	mat = new SoMaterial();
	mat1 = new SoMaterial();
	mat2 = new SoMaterial();
	mat->diffuseColor.setValue(color);
	mat1->diffuseColor.setValue(COLOR_CIP);
	mat2->diffuseColor.setValue(COLOR_HIP);
    angleBallGr=0;
	SoSeparator *BSep = new SoSeparator;
	root->addChild(BSep);
	shape->width.setValue(width-0.05);//cigil
	shape->height.setValue(height);
	shape->depth.setValue(depth-0.05);//cigil
	ips->radius.setValue(IP_RADIUS);
    transf->translation.setValue(Point(2*(ball_width*0.5+IP_RADIUS),0,0));
	transfn->translation.setValue(Point(-(ball_width*0.5+IP_RADIUS),0,0));
	transfMat->translation.setValue(position);
    transfMat->rotation.setValue(SbVec3f(0,1,0),angleBallGr);
	BSep->addChild(transfMat);
	BSep->addChild(mat);
	BSep->addChild(shape);
   BSep->addChild(transfn);
   BSep->addChild(mat2);//ARKADAKÝ CIP
	BSep->addChild(ips);
	BSep->addChild(transf);
	BSep->addChild(mat1);//(CIP)
	BSep->addChild(ips);
}

void BallGr::setTranslate(Vector tr)
{
	transfMat->translation.setValue(tr);
}
void BallGr::setRotation(float angg)
{
	transfMat->rotation.setValue(SbVec3f(0,1,0),angg);
}
void BallGr::setPosition(Point pt, bool allowJumps)
{
	ball->setPosition(pt, allowJumps);
}






void BallGr::incrementPosition(Point pt)
{
	ball->incrementPosition(pt);
}

void BallGr::setVelocity(Vector vel)
{
	ball->setVelocity(vel);
}

void BallGr::setAcceleration(Vector acc)
{
	ball->setAcceleration(acc);
}

Point BallGr::getPosition()
{
	return ball->getPosition();
}
Point BallGr::getPrevPosition()
{
	return ball->getPrevPosition();
}

Vector BallGr::getVelocity()
{
	return ball->getVelocity();
}
Vector BallGr::getPrevVelocity()
{
	return ball->getPrevVelocity();
}
Vector BallGr::getAcceleration()
{
	return ball->getAcceleration();
}
float BallGr::getRadius()
{
	return ball->getRadius();
}
float BallGr::getWidth()
{
	return ball->getWidth();
}
float BallGr::getHeight()
{
	return ball->getHeight();
}
float BallGr::getDepth()
{
	return ball->getDepth();
}

float BallGr::getMass()
{
	return ball->getMass();
}

Ball* BallGr::getBall()
{
	return ball;
}

float BallGr::calculateAngleBallGr(Vector f1,Vector f2)
{    
	float bangle=ball->calculateAngle(f1,f2);
	angleBallGr=fmod(bangle,(2*PI2));
    ball->setAngle(angleBallGr); 
	BallGr::setAngleBallGr(angleBallGr);
	return fmod(bangle,(2*PI2));
}

void BallGr::setAngleBallGr(float angle)
{    angleBallGr=fmod(angle,(2*PI2));
    ball->setAngle(angleBallGr); 
	
	
}
float BallGr::ggetAngleBallGr()
{    
	return ball->getAngle();
	
}
