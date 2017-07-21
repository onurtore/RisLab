#include "GraphicsController.h"
#include "BoardGraphics.h"
#include "TargetGraphics.h"

BoardGraphics::BoardGraphics(float bT, float bD, float bH)
{
	boundaryThickness	= bT;
	boundaryDim			= bD;
	boundaryHeight		= bH;

	float len	= boundaryDim+boundaryThickness;
	boundary	= Vector(len, len, len);

	floorHeight	= 5;
	floorWidth	= (boundary[0]-boundaryThickness)*2;
	floorDepth	= (boundary[2]-boundaryThickness)*2;

	shape = constructBoardShape();
}


SoSeparator* BoardGraphics::constructBoardShape()
{
	SoSeparator* shape = new SoSeparator;

	// add floor
	SoSeparator *board = new SoSeparator;
	shape->addChild(board);

	// AYSE: disable board texture
	//SoTexture2* boardTexture	= new SoTexture2;
	//boardTexture->filename.setValue(TUNNEL_IMAGE);
	//board->addChild(boardTexture);

	SoMaterial *boardMat = new SoMaterial;
	boardMat ->diffuseColor.setValue(COLOR_BOARD);
	SoTransform *boardTrans = new SoTransform;
	SoCube *boardCube = new SoCube;

	boardTrans->translation.setValue(0.0f, -floorHeight/2, 0.0f);
	boardCube->height.setValue(floorHeight);
	boardCube->width.setValue(floorWidth);
	boardCube->depth.setValue(floorDepth);

	board->addChild(boardMat);
	board->addChild(boardTrans);
	board->addChild(boardCube);

	// add walls
	SoMaterial *boardBoundaryMat = new SoMaterial;
	boardBoundaryMat->diffuseColor.setValue(0.7f,0.8f,0.4f);

	SoSeparator *boardBoundaryRoot1 = new SoSeparator;
	SoTransform *boardBoundaryTrans1 = new SoTransform;
	boardBoundaryTrans1->translation.setValue( 
		boundary[0] - boundaryThickness/2.0f, 0.0f, 0.0f);
	SoCube *boardBoundary1 = new SoCube;
	boardBoundary1->width.setValue(boundaryThickness);
	boardBoundary1->height.setValue(boundaryHeight);
	boardBoundary1->depth.setValue(boundary[2]*2);
	boardBoundaryRoot1->addChild(boardBoundaryTrans1);
	boardBoundaryRoot1->addChild(boardBoundaryMat);
	boardBoundaryRoot1->addChild(boardBoundary1);
	shape->addChild(boardBoundaryRoot1);

	SoSeparator *boardBoundaryRoot2 = new SoSeparator;
	SoTransform *boardBoundaryTrans2 = new SoTransform;
	boardBoundaryTrans2->translation.setValue(
		-boundary[0] + boundaryThickness/2.0f, 0.0f, 0.0f);
	SoCube *boardBoundary2 = new SoCube;
	boardBoundary2->width.setValue(boundaryThickness);
	boardBoundary2->height.setValue(boundaryHeight);
	boardBoundary2->depth.setValue(boundary[2]*2.0f);
	boardBoundaryRoot2->addChild(boardBoundaryTrans2);
	boardBoundaryRoot2->addChild(boardBoundaryMat);
	boardBoundaryRoot2->addChild(boardBoundary2);
	shape->addChild(boardBoundaryRoot2);

	SoSeparator *boardBoundaryRoot3 = new SoSeparator;
	SoTransform *boardBoundaryTrans3 = new SoTransform;
	boardBoundaryTrans3->translation.setValue(
		0.0f, 0.0f, boundary[2] - boundaryThickness/2.0f);
	SoCube *boardBoundary3 = new SoCube;
	boardBoundary3->width.setValue(boundary[0]*2.0f);
	boardBoundary3->height.setValue(boundaryHeight);
	boardBoundary3->depth.setValue(boundaryThickness);
	boardBoundaryRoot3->addChild(boardBoundaryTrans3);
	boardBoundaryRoot3->addChild(boardBoundaryMat);
	boardBoundaryRoot3->addChild(boardBoundary3);
	shape->addChild(boardBoundaryRoot3);

	SoSeparator *boardBoundaryRoot4 = new SoSeparator;
	SoTransform *boardBoundaryTrans4 = new SoTransform;
	boardBoundaryTrans4->translation.setValue(
		0.0f, 0.0f, -boundary[2] + boundaryThickness/2.0f);
	SoCube *boardBoundary4 = new SoCube;
	boardBoundary4->width.setValue(boundary[0]*2.0f);
	boardBoundary4->height.setValue(boundaryHeight);
	boardBoundary4->depth.setValue(boundaryThickness);
	boardBoundaryRoot4->addChild(boardBoundaryTrans4);
	boardBoundaryRoot4->addChild(boardBoundaryMat);
	boardBoundaryRoot4->addChild(boardBoundary4);
	shape->addChild(boardBoundaryRoot4);

	return shape;
}

