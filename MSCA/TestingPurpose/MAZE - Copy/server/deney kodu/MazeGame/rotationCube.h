#pragma once
#include "Public.h"
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/SbBasic.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoMaterial.h>
#include "Ball.h"

class  rotationCube
{
public:
	Ball *ball;
	SoTransform *transfMat;
    SoTransform *transf;
	SoTransform *transfn;
	SbMatrix *transMat;
	SbMatrix *rotateMat;
	SoCube *shape;
	SoSphere *ips;
	SoMaterial *mat;
	SoMaterial *mat1;
	SoMaterial *mat2;
	float ballWidth;
	float ballDepth;
	float ballHeight;

	rotationCube(SoSeparator *root, Point position, float width,float height,float depth,float radius, float mass, Vector color);

	void setTranslate(Vector tr);
	void setPosition(Point pt);
	void incrementPosition(Point pt);
	void setVelocity(Vector vel);
	void setAcceleration(Vector vel);
    void setAngVelocity(float vel);
	void setAngAcc(float acc);
	

	void setRotation(float angg);
	void setWidth(float w);
	void setDepth(float w);
	void setHeight(float w);
	void setHandleRadius(float w);
	void setAngleBallGr(float angle);
   
	Point  getPosition();
	Point  getPrevPosition();
	Vector getVelocity();
	Vector getPrevVelocity();
	Vector getAcceleration();
	float  getAngVelocity();
	
	float  getWidth();
	float  getDepth();
	float  getHeight();
	float  getRadius();
	float  getMass();
	
	void  calculateAngleBallGr(Vector f1,Vector f2);
	float  calculateAngleBallGr_col(Vector f1,Vector f2);
	float  getAngleBallGr();
	Ball*  getBall();
	
};
