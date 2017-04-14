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

	SoSeparator *BSep = new SoSeparator;
	root->addChild(BSep);
	ballWidth=width-0.05;
	ballDepth=depth-0.05;
	ballHeight=height;
	shape->width.setValue(ballWidth);//cigil
	shape->height.setValue(ballHeight);
	shape->depth.setValue(ballDepth);//cigil
	ips->radius.setValue(IP_RADIUS);
    transf->translation.setValue(Point(2*(BALL_WIDTH*0.5+IP_RADIUS),0,0));
	transfn->translation.setValue(Point(-(BALL_WIDTH*0.5+IP_RADIUS),0,0));
	transfMat->translation.setValue(position);
	transfMat->rotation.setValue(SbVec3f(0,1,0),ball->getAngle());
	BSep->addChild(transfMat);
	BSep->addChild(mat);
	BSep->addChild(shape);
	BSep->addChild(transfn);
	BSep->addChild(mat2);//ARKADAKÝ CIP
	//BSep->addChild(ips);
	BSep->addChild(transf);
	BSep->addChild(mat1);//(CIP)
	//BSep->addChild(ips);
}

void BallGr::setTranslate(Vector tr)
{
	transfMat->translation.setValue(tr);
}

void BallGr::setRotation(float angg)
{
	transfMat->rotation.setValue(SbVec3f(0,1,0),angg);
}

void BallGr::setPosition(Point pt)
{
	ball->setPosition(pt);
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

void BallGr::setAngVelocity(float vel)
{
	ball->setAngVelocity(vel);
}
void BallGr::setAngAcc(float acc)
{
	ball->setAngAcceleration(acc);
}

Point BallGr::getPosition()
{
	return ball->getPosition();
}

Vector BallGr::getVelocity()
{
	return ball->getVelocity();
}

Vector BallGr::getAcceleration()
{
	return ball->getAcceleration();
}

float BallGr::getAngVelocity()
{
	return ball->getAngVelocity();
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

void BallGr::calculateAngleBallGr(Vector f1,Vector f2)
{    
	ball->calculateAngle(f1,f2);

	//return fmod(bangle, (2*PI));
}

void BallGr::setAngleBallGr(float angle)
{    
    ball->setAngle(fmod(angle,(2*PI))); 	
}

float BallGr::getAngleBallGr()
{    
	return ball->getAngle();
	
}
void BallGr::setWidth(float w)
{
	shape->width.setValue(w);
}

void BallGr::setDepth(float w)
{
	shape->depth.setValue(w);
}

void BallGr::setHeight(float w)
{
	shape->height.setValue(w);
}
void BallGr::setHandleRadius(float w)
{
	ips->radius.setValue(w);
}