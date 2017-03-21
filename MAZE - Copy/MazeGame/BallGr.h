#pragma once
#include "Public.h"
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/SbBasic.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoMaterial.h>
#include "Ball.h"

class  BallGr
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
	Vector BallGrpoints[4];
    float angleBallGr;
	BallGr();
	BallGr(SoSeparator *root, Point position, float width,float height,float depth,float radius, float mass, Vector color);

	void setTranslate(Vector tr);
	void setPosition(Point pt, bool allowJumps = true);
	void incrementPosition(Point pt);
	void setVelocity(Vector vel);
	void setAcceleration(Vector vel);
    void setRotation(float angg);
	void BallGr::setAngleBallGr(float angle);
    Vector setBallGrpoints1(Vector position,float angle);
	Vector setBallGrpoints2(Vector position,float angle);
	Vector setBallGrpoints3(Vector position,float angle);
	Vector setBallGrpoints4(Vector position,float angle);



    
    Vector checkBallGrpointColl(Vector pp);
	Vector getBallGrpoints1(Point position,float angle);	
	Vector getBallGrpoints2(Point position,float angle);	
	Vector getBallGrpoints3(Point position,float angle);
    Vector getBallGrpoints4(Point position,float angle);
	Point getPosition();
	Point getPrevPosition();
	Vector getVelocity();
	Vector getPrevVelocity();
	Vector getAcceleration();
	float getWidth();
	float getDepth();
	float getHeight();
	float getRadius();
	float getMass();
	float calculateAngleBallGr(Vector f1,Vector f2);
	float calculateAngleBallGr_col(Vector f1,Vector f2);
	float ggetAngleBallGr();
	Ball* getBall();
	Vector getFInertia();
};
