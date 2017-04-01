#pragma once
#include <Inventor/nodes/SoComplexity.h>
#include <Inventor/nodes/SoNurbsCurve.h>

#include "Public.h"
#include "Tunnel.h"

#define INACTIVE_COLOR	1.0, 0.5, 0.0//1.0, 0.5, 1.0
#define ACTIVE_COLOR	0.0, 0.0, 1.0

class TunnelGraphics
{
public:
	Tunnel *tunnel;

	// define obstacle shape and material
	SoSeparator *shape;
	SoMaterial *mat;
	SoTransform *transfMat;
	
	// angle holds degrees of rotation around y axis
	TunnelGraphics(SoSeparator *root, Point *keypointPos, int keypointCount, float angle = 0);
	~TunnelGraphics();
	
	SoSeparator* constructShape();
	SoSeparator* fitNurbsCurve(Point* nurbsPoints);
	void changeColor(float r, float g, float b);
};
