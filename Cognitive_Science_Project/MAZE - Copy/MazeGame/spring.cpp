#pragma once

#include "spring.h"
#include "MathCB.h"

spring::spring(){}

spring::spring(SoSeparator *root, Point pos, float distx,float distz, int who)
{
	lineSet = new SoLineSet;
	vertx = new SoVertexProperty;
	numOfLines = 100;
	initLineSet(root, pos, distx,distz, who);
}
void spring::initLineSet(SoSeparator *root, Point pos, float distx,float distz, int who)
{
	float T;
	int L, idx = 0;
	mat = new SoMaterial();
	dS = new SoDrawStyle();
	mat->diffuseColor.setValue( 0.1f, 0.1f, 0.1f );
	dS->lineWidth.setValue(3.0f);
	float dist=sqrt(distx*distx+distz*distz);
	float angle_s=atan2(distx,distz);
	T = fabs(dist/peakNum);
	
	coords	= (SbVec3f*)malloc(sizeof(SbVec3f)*peakNum*2*10*T+1); 
	pts		= (int*)malloc(sizeof(int)*peakNum*T*10); 

	for (int i = 0; i < peakNum*T*10; i++)
	{
		pts[i] = 2;
	}

	coords[idx++] = SbVec3f(pos[0], pos[1], pos[2]);

	for ( float i = 0.1; i <= peakNum*T; i+=0.1f ){
		coords[idx++] = SbVec3f(i*sin(angle_s)+pos[0], pos[1]+sinf(2*PI*i/(T*peakNum)), pos[2]+i*cos(angle_s));
		coords[idx++] = SbVec3f(i*sin(angle_s)+pos[0], pos[1]+sinf(2*PI*i/(T*peakNum)), pos[2]+i*cos(angle_s));
	}

	numOfLines = idx/2;
	//printf("idx: %d,\tnumOfL: %d\n", idx, numOfLines); 
	lineSet->numVertices.setValues(0, numOfLines, pts);
	if ( who == _CIP )
		lineSet->setName("LSC");
	else if ( who == _HIP )
		lineSet->setName("LSH");
	else if ( who == _CIP2 )
		lineSet->setName("LSC2");
	else if ( who == _HIP2 )
		lineSet->setName("LSH2");
	vertx->vertex.setValues(0, 2*numOfLines, coords);
	lineSet->vertexProperty.setValue(vertx);
	root->addChild(dS);
	root->addChild(mat);
	root->addChild(lineSet);
}
// pos: position of the interface point (ip). shift the spring by that amount
// dist: determines the length of the spring
void spring::drawSpring(SoSeparator *sprSep, Point pos, float distx,float distz, int who)
{
	int numOfLines;
	int *pts; 
	SbVec3f *coords;
	float T;
	int L, idx = 0;
	
	float dist=sqrt(distx*distx+distz*distz);
	distx=-distx;
	distz=-distz;
	float angle_s=atan2(distx,distz);

	T = fabs(dist/peakNum);

	coords	= (SbVec3f*)malloc(sizeof(SbVec3f)*peakNum*2*10*ceil(T)+1); 
	pts		= (int*)malloc(sizeof(int)*peakNum*ceil(T)*10); 

	for (int i = 0; i < peakNum*T*10; i++)
	{
		pts[i] = 2;
	}
	
	

	if ( who == _CIP )
		lineSet = (SoLineSet*)sprSep->getByName("LSC");
		
	else if ( who == _HIP )
		lineSet = (SoLineSet*)sprSep->getByName("LSH");
	else if ( who == _CIP2 )
		lineSet = (SoLineSet*)sprSep->getByName("LSC2");
		
	else if ( who == _HIP2 )
		lineSet = (SoLineSet*)sprSep->getByName("LSH2");
		
	coords[idx++] = SbVec3f(pos[0], pos[1], pos[2]);

	for ( float i = 0.1; i <= peakNum*T; i+=0.1f )
	{
		coords[idx++] = SbVec3f(i*sin(angle_s)+pos[0], pos[1]+sinf(2*PI*i/T), pos[2]+i*cos(angle_s));//cigil
		coords[idx++] = SbVec3f(i*sin(angle_s)+pos[0], pos[1]+sinf(2*PI*i/T), pos[2]+i*cos(angle_s));//cigil
		
	}

	numOfLines = idx/2;
	lineSet->vertexProperty.cleanupClass();
	lineSet->numVertices.deleteValues(0,-1);
	lineSet->numVertices.setValues(0, numOfLines, pts);
	vertx->vertex.setValues(0, 2*numOfLines, coords);
	lineSet->vertexProperty.setValue(vertx);
	delete [] coords;
	delete [] pts;
	//printf("T:%f,\tidx: %d,\tnumOfL: %d\n", T, idx, numOfLines); 
}