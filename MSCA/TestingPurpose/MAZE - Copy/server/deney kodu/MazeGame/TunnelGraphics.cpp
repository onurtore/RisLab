#include "TunnelGraphics.h"

TunnelGraphics::TunnelGraphics(SoSeparator *root, Point *keypointPos, int keypointCount, float angle)
{
	tunnel				= new Tunnel(keypointPos, keypointCount, angle);

	SoSeparator *sep	= new SoSeparator;
	transfMat			= new SoTransform;
	mat					= new SoMaterial;
	shape				= constructShape();

	transfMat->translation.setValue(keypointPos[0]);
	transfMat->rotation.setValue(SbVec3f (0, 1, 0), TO_RADIANS(angle) );
  
	sep->addChild(transfMat);
	sep->addChild(shape);

	// add shape to root
	root->addChild(sep);
}

TunnelGraphics::~TunnelGraphics()
{
	delete tunnel;
}


SoSeparator* TunnelGraphics::constructShape()
{  
	SoSeparator* sep	= new SoSeparator;

	SoSeparator* curveSep1 = new SoSeparator;

	SoTransform	*transf1 = new SoTransform;
	transf1->translation.setValue(Point(0, 0, -5));
	
	SoSeparator* curveSep2 = new SoSeparator;
	SoTransform	*transf2 = new SoTransform;
	transf2->translation.setValue(Point(0, 0, 5));

	curveSep1->addChild(transf1);
	curveSep1->addChild(fitNurbsCurve(tunnel->getKeypoints()));
	curveSep2->addChild(transf2);
	curveSep2->addChild(fitNurbsCurve(tunnel->getKeypoints()));

	sep->addChild(curveSep1);
	sep->addChild(curveSep2);

	SoTransform	*transf = new SoTransform;
	//transf->translation.setValue(tunnel->getKeypoints()[0]);
	SoCube* shape = new SoCube;

	sep->addChild(transf);
	sep->addChild(shape);

	for (int i=1; i < tunnel->getKeypointCount(); i++)
	{	
		SoTransform	*transf = new SoTransform;
		transf->translation.setValue(tunnel->getKeypoints()[i]-tunnel->getKeypoints()[i-1]);
		SoCube* shape = new SoCube;
	
		sep->addChild(transf);
		sep->addChild(shape);
	}
	return sep;
}


SoSeparator* TunnelGraphics::fitNurbsCurve(Point* nurbsPts)
{
	SoSeparator* curveSep = new SoSeparator;
	SoComplexity  *complexity = new SoComplexity;
	SoCoordinate3 *nurbsControlPts = new SoCoordinate3;
	SoNurbsCurve  *curve      = new SoNurbsCurve;
	SoTransform	*transf = new SoTransform;
	transf->translation.setValue(Point(-5,0, -5));
	transf->translation.setValue(-1*nurbsPts[0]);
	curveSep->addChild(transf);

	float* knots = new float[tunnel->getKeypointCount()+4];

	SoMaterial *curveColor = new SoMaterial();
	curveColor->diffuseColor.setValue(55,55,55);

	curveSep->addChild(curveColor);

	// Set the draw style of the curve.
	SoDrawStyle *drawStyle  = new SoDrawStyle;
	drawStyle->lineWidth = 4;
	curveSep->addChild(drawStyle);

	// knots for the NURBS curve

	for (int i = 0; i < tunnel->getKeypointCount()+4; i++)
	{
		/* Create knot vector for the uniform 
		cubic B-spline that passes through endpoints
		fill with 4 zeros at the beginning, 
		4 ones at the end, uniformly spaced in between */
		if (i < 4)
			knots[i] = 0;
		else if (i >= tunnel->getKeypointCount())
			knots[i] = tunnel->getKeypointCount() - 3;
		else 
			knots[i] = 1;
	}

	// Define the NURBS curve including the control points
	// and a complexity.
	complexity->value = 1.0;
	
	nurbsControlPts->point.setValues(0, tunnel->getKeypointCount(), nurbsPts);
	
	curve->numControlPoints = tunnel->getKeypointCount();
	curve->knotVector.setValues(0, tunnel->getKeypointCount()+4, knots);
	
	curveSep->addChild(complexity);
	curveSep->addChild(nurbsControlPts);
	curveSep->addChild(curve);

	curveSep->unrefNoDelete();
	return curveSep;
}


void TunnelGraphics::changeColor(float r, float g, float b)
{
	mat->ambientColor.setValue(r, g, b);
}