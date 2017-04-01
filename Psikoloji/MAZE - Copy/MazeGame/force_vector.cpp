#include "GraphicsController.h"
#include "force_vector.h"
#include "Public.h"

force_vector::force_vector(float vD,Vector tt,Vector Color)
{

	vx=tt[0];
	vy=tt[1];
	vz=tt[2];
	vector_color=Color;

	//vector_Width=vW;
	vector_Depth=vD;
	anglerotation=2.0f;



	vectorCube=new SoCube();
	transfMat = new SoTransform();
	transfMat->translation.setValue(tt);
	vectorShape = constructVectorShape();
}


SoSeparator* force_vector::constructVectorShape()
{
	SoSeparator* vectorShape= new SoSeparator;



	SoMaterial *vectorMat = new SoMaterial;
	vectorMat ->diffuseColor.setValue(vector_color);

	vectorCube = new SoCube;
	transfMat = new SoTransform;

	transfMat->translation.setValue(vx,vy,vz);
	transfMat->rotation.setValue(Vector(0,1,0),0.0f);

	vectorCube->height.setValue(0.001f);
	vectorCube->width.setValue(0.001f);
	vectorCube->depth.setValue(vector_Depth);//-1);


	vectorShape->addChild(transfMat);
	vectorShape->addChild(vectorMat);
	vectorShape->addChild(vectorCube);



	return vectorShape;
}
/*
void force_vector::setVectorWidth(float vWidth)
{
vector_Width=vWidth;
}*/
void force_vector::setVectorDepth(float vD)
{

	vector_Depth=vD;

}
/*
float force_vector::getVectorWidth()
{
return vector_Width;
}*/
float force_vector::getVectorDepth()
{

	return vector_Depth;
}

void force_vector::setVectorT(Vector tr)
{
	vx=tr[0];
	vy=tr[1];
	vz=tr[2];


}


Vector force_vector::getVectorT()
{
	return Vector( vx,vy,vz);
}
void force_vector::setTranslate(Vector tr)
{
	transfMat->translation.setValue(tr);
}

void force_vector::setVectorAngle(float vangle)
{
	anglerotation=vangle;
}
float force_vector::getVectorAngle()
{
	return anglerotation;
}

void force_vector::setCubeDepth(float depth)
{
	vectorCube->depth.setValue(depth);
}

void force_vector::setCubeWidthHeight(float w,float h)
{
	vectorCube->width.setValue(w);
	vectorCube->height.setValue(h);
}
void force_vector::setRotation(float angle)
{

	transfMat->rotation.setValue(Vector(0,1,0),angle);
}

void force_vector::correctT()
{
	float d1=getVectorDepth();
	float a1=getVectorAngle();
	
	Vector tr;
	tr=getVectorT();
	vx=tr[0]-d1*0.5*sin(a1);
	vy=tr[1];
	vz=tr[2]-d1*0.5*cos(a1);
	setVectorT(Vector(vx,vy,vz));
}