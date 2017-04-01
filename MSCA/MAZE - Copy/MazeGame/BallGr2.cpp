#include "BallGr2.h"

BallGr2::BallGr2(SoSeparator *root, Point position, float width,float height,float depth,float radius, float mass, Vector color)
{
	ball2 = new Ball(position, width,height,depth,radius,mass); 
    
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

void BallGr2::setTranslate(Vector tr)
{
	transfMat->translation.setValue(tr);
}
void BallGr2::setRotation(float angg)
{
	transfMat->rotation.setValue(SbVec3f(0,1,0),angg);
}
void BallGr2::setPosition(Point pt, bool allowJumps)
{
	ball2->setPosition(pt, allowJumps);
}






void BallGr2::incrementPosition(Point pt)
{
	ball2->incrementPosition(pt);
}

void BallGr2::setVelocity(Vector vel)
{
	ball2->setVelocity(vel);
}

void BallGr2::setAcceleration(Vector acc)
{
	ball2->setAcceleration(acc);
}

Point BallGr2::getPosition()
{
	return ball2->getPosition();
}
Point BallGr2::getPrevPosition()
{
	return ball2->getPrevPosition();
}

Vector BallGr2::getVelocity()
{
	return ball2->getVelocity();
}
Vector BallGr2::getPrevVelocity()
{
	return ball2->getPrevVelocity();
}
Vector BallGr2::getAcceleration()
{
	return ball2->getAcceleration();
}
float BallGr2::getRadius()
{
	return ball2->getRadius();
}
float BallGr2::getWidth()
{
	return ball2->getWidth();
}
float BallGr2::getHeight()
{
	return ball2->getHeight();
}
float BallGr2::getDepth()
{
	return ball2->getDepth();
}

float BallGr2::getMass()
{
	return ball2->getMass();
}

Ball* BallGr2::getBall()
{
	return ball2;
}

float BallGr2::calculateAngleBallGr(Vector f1,Vector f2)
{    
	float bangle=ball2->calculateAngle(f1,f2);
	angleBallGr=fmod(bangle,(2*PI2));
    ball2->setAngle(angleBallGr); 
	BallGr2::setAngleBallGr(angleBallGr);
	return fmod(bangle,(2*PI2));
}

void BallGr2::setAngleBallGr(float angle)
{    angleBallGr=fmod(angle,(2*PI2));
    ball2->setAngle(angleBallGr); 
	
	
}
float BallGr2::ggetAngleBallGr()
{    
	return ball2->getAngle();
	
}
