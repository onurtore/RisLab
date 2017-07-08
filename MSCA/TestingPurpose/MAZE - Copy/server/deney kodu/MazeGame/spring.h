#pragma once
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/SbBasic.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoVertexProperty.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include "Public.h"

typedef SbVec3f Point;
typedef SbVec3f Vector;

class  Spring
{
public:

	SoTransform *transfMat;
	SoMaterial *mat; 
	SoDrawStyle *dS; 

	Spring(SoSeparator *root, Point pos, float distx,float distz, int who);

	SoLineSet *lineSet;
	SoVertexProperty *vertx;
	
	int numOfLines;
	int *pts; 
	static const int peakNum = 10;
	SbVec3f *coords;

	void initLineSet(SoSeparator *root, Point pos, float distx,float distz, int who);
	void drawSpring(SoSeparator *sprSep, Point pos, float distx,float distz, int who);
};
