#include "BallGr.h"

BallGr::BallGr(SoSeparator *root, Point position, float radius, float mass, Vector color)
{
	ball = new Ball(position, radius, mass); 

	transfMat = new SoTransform();
	shape = new SoSphere();
	mat = new SoMaterial();
	mat->diffuseColor.setValue(color);

	SoSeparator *BSep = new SoSeparator;

	root->addChild(BSep);
	shape->radius.setValue(radius);
	transfMat->translation.setValue(position);

	BSep->addChild(transfMat);
	BSep->addChild(mat);
	BSep->addChild(shape);
}

void BallGr::setTranslate(Vector tr)
{
	transfMat->translation.setValue(tr);
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

Vector BallGr::getVelocity()
{
	return ball->getVelocity();
}

Vector BallGr::getAcceleration()
{
	return ball->getAcceleration();
}

float BallGr::getRadius()
{
	return ball->getRadius();
}

float BallGr::getMass()
{
	return ball->getMass();
}

Ball* BallGr::getBall()
{
	return ball;
}

