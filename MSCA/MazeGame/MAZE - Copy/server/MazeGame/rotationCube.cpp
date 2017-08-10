#include "rotationCube.h"

rotationCube::rotationCube(SoSeparator *root, Point position, float width,float height,float depth,float radius, float mass, Vector color)
{
	ball = new Ball(position, width,height,depth,radius,mass); 
    
	transfMat = new SoTransform();
	transf = new SoTransform();
    transfn=new SoTransform();
	shape = new SoCube();
   // ips=new SoSphere();
	mat = new SoMaterial();
	mat1 = new SoMaterial();
	mat2 = new SoMaterial();
	mat->diffuseColor.setValue(color);
	//mat1->diffuseColor.setValue(COLOR_CIP);
	//mat2->diffuseColor.setValue(COLOR_HIP);

	SoSeparator *BSep = new SoSeparator;
	root->addChild(BSep);
	ballWidth=width-0.05;
	ballDepth=depth-0.05;
	ballHeight=height;
	
	shape->width.setValue(ballWidth);//cigil
	shape->height.setValue(ballHeight);
	shape->depth.setValue(ballDepth);//cigil
	//ips->radius.setValue(IP_RADIUS);
    //transf->translation.setValue(Point(2*(BALL_WIDTH+IP_RADIUS),0,0));
	//transfn->translation.setValue(Point(-(BALL_WIDTH+IP_RADIUS),0,0));
	transfMat->translation.setValue(position);
	transfMat->rotation.setValue(SbVec3f(0,1,0),ball->getAngle());
	BSep->addChild(transfMat);
	BSep->addChild(mat);
	BSep->addChild(shape);
	//BSep->addChild(transfn);
	//BSep->addChild(mat2);//ARKADAKÝ CIP
	//BSep->addChild(ips);
	//BSep->addChild(transf);
//	BSep->addChild(mat1);//(CIP)
//	BSep->addChild(ips);
}

void rotationCube::setTranslate(Vector tr)
{
	transfMat->translation.setValue(tr);
}

void rotationCube::setRotation(float angg)
{
	transfMat->rotation.setValue(SbVec3f(0,1,0),angg);
}

void rotationCube::setPosition(Point pt)
{
	ball->setPosition(pt);
}

void rotationCube::incrementPosition(Point pt)
{
	ball->incrementPosition(pt);
}

void rotationCube::setVelocity(Vector vel)
{
	ball->setVelocity(vel);
}

void rotationCube::setAcceleration(Vector acc)
{
	ball->setAcceleration(acc);
}

void rotationCube::setAngVelocity(float vel)
{
	ball->setAngVelocity(vel);
}
void rotationCube::setAngAcc(float acc)
{
	ball->setAngAcceleration(acc);
}

Point rotationCube::getPosition()
{
	return ball->getPosition();
}

Vector rotationCube::getVelocity()
{
	return ball->getVelocity();
}

Vector rotationCube::getAcceleration()
{
	return ball->getAcceleration();
}

float rotationCube::getAngVelocity()
{
	return ball->getAngVelocity();
}

float rotationCube::getRadius()
{
	return ball->getRadius();
}
float rotationCube::getWidth()
{
	return ball->getWidth();
}
float rotationCube::getHeight()
{
	return ball->getHeight();
}
float rotationCube::getDepth()
{
	return ball->getDepth();
}

float rotationCube::getMass()
{
	return ball->getMass();
}

Ball* rotationCube::getBall()
{
	return ball;
}

void rotationCube::calculateAngleBallGr(Vector f1,Vector f2)
{    
	ball->calculateAngle(f1,f2);

	//return fmod(bangle, (2*PI));
}

void rotationCube::setAngleBallGr(float angle)
{    
    ball->setAngle(fmod(angle,(2*PI))); 	
}

float rotationCube::getAngleBallGr()
{    
	return ball->getAngle();
	
}
void rotationCube::setWidth(float w)
{
	shape->width.setValue(w);
}

void rotationCube::setDepth(float w)
{
	shape->depth.setValue(w);
}

void rotationCube::setHeight(float w)
{
	shape->height.setValue(w);
}