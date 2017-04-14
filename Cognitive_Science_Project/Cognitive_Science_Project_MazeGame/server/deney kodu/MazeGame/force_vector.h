#pragma once
#include "Public.h"
#include <Inventor/SbBasic.h>

class force_vector
{
public:
	force_vector();
	force_vector(float vD,Vector tt,Vector Color);
	//Vector wallT;
	//float tx;
	//float wallTy;
	//float tz;
	//float ty;
	float vx,vy,vz;
	//float vector_Height;
	//float vector_Width;
	float vector_Depth;
	Vector vector_color;

	float anglerotation;
	SoTransform *transfMat;
	SoCube *vectorCube;

	SoSeparator *vectorShape;
	

	

	SoSeparator * constructVectorShape();
	//void setVectorWidth(float vW);
	void setVectorDepth(float vD);
	void setVectorT(Vector tr);
	
	float getVectorDepth();
	//float getVectorWidth();

	Vector getVectorT();
	
	void setTranslate(Vector tr);
	void setRotation(float angle);
	void setCubeDepth(float depth);
	void setCubeWidthHeight(float w,float h);
    void setVectorAngle(float vangle);
	float getVectorAngle();
	void correctT();
	
};