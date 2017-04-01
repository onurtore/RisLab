#include "GraphicsController.h"
#include "BoardGraphics.h"

BoardGraphics::BoardGraphics(float bT, float bD, float bH)
{
	boundaryThickness	= bT;
	boundaryDim			= bD;
	boundaryHeight		= bH;

	float len	= boundaryDim+boundaryThickness;
	float lenz  =new_depth*0.5+boundaryThickness;
	boundary	= Vector(len, len, lenz);

	floorHeight	= 5;
	floorWidth	= (boundary[0]-boundaryThickness)*2;
	floorDepth	= (boundary[2]-boundaryThickness)*2;

	hitB=false;
	arrived1=false;
	arrived2=false;





	/*boardBoundaryRoot1=new SoSeparator(); 
	boardBoundaryRoot2=new SoSeparator(); 
	boardBoundaryRoot3=new SoSeparator(); 
	boardBoundaryRoot4=new SoSeparator();*/

	//boardBoundaryColor=Vector(0.7f,0.8f,0.4f);//COLOR_BOARD_BOUNDARY;
	boardBoundaryMat = new SoMaterial();
	targetMat1=new SoMaterial();
	targetMat2=new SoMaterial();
	

	shape = constructBoardShape();

}


SoSeparator* BoardGraphics::constructBoardShape()
{
	SoSeparator* shape = new SoSeparator;

	// add floor
	SoSeparator *board = new SoSeparator;
	shape->addChild(board);

	SoMaterial *boardMat = new SoMaterial;
	boardMat ->diffuseColor.setValue(COLOR_BOARD);
	SoTransform *boardTrans = new SoTransform;
	SoCube *boardCube = new SoCube;
	float recHeight=4.0f;
	SoSeparator *target_rec1=new SoSeparator;
	SoCube *rectangles=new SoCube;
	rectangles->width.setValue(TARGET_DIM);//cigil
	rectangles->height.setValue(recHeight);
	rectangles->depth.setValue(TARGET_DIM);
	SoTransform *targetTrans1 = new SoTransform;
	
	targetTrans1->translation.setValue(
		fwx+floorWidth*0.5-8-INITIAL_TARGET, -recHeight*0.5+0.5,  boundary[2]- boundaryThickness/2.0f+fw-new_depth*0.5-0.5);//cigil
	//SoMaterial *targetMat1 = new SoMaterial;
	targetMat1 ->diffuseColor.setValue(COLOR_GREEN);
	target_rec1->addChild(targetTrans1);
	target_rec1->addChild(targetMat1);
	target_rec1->addChild(rectangles);
	shape->addChild(target_rec1);

	//boardTrans->translation.setValue(0.0f, -floorHeight/2, 0.0f);//cigil
	boardTrans->translation.setValue(fwx, -floorHeight/2, fw);
	boardCube->height.setValue(floorHeight);
	boardCube->width.setValue(floorWidth);//cigil
	boardCube->depth.setValue(new_depth);//floorDepth);//cigil

	board->addChild(boardMat);
	board->addChild(boardTrans);
	board->addChild(boardCube);
	
	// add walls
	//SoMaterial *boardBoundaryMat = new SoMaterial();
	boardBoundaryMat->diffuseColor.setValue(COLOR_BOARD_BOUNDARY);

	SoSeparator *boardBoundaryRoot1 = new SoSeparator;
	SoTransform *boardBoundaryTrans1 = new SoTransform;
	boardBoundaryTrans1->translation.setValue( 
		boundary[0] - boundaryThickness/2.0f+fwx, 0.0f, fw);//cigil
	SoCube *boardBoundary1 = new SoCube;
	boardBoundary1->width.setValue(boundaryThickness);
	boardBoundary1->height.setValue(boundaryHeight);
	boardBoundary1->depth.setValue(boundary[2]*2);//cigil
	boardBoundaryRoot1->addChild(boardBoundaryTrans1);
	boardBoundaryRoot1->addChild(boardBoundaryMat);
	boardBoundaryRoot1->addChild(boardBoundary1);
	shape->addChild(boardBoundaryRoot1);

	SoSeparator *boardBoundaryRoot2 = new SoSeparator;
	SoTransform *boardBoundaryTrans2 = new SoTransform;
	boardBoundaryTrans2->translation.setValue(
		-boundary[0] + boundaryThickness/2.0f+fwx, 0.0f, fw);//cigil
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
		fwx, 0.0f, boundary[2] - boundaryThickness/2.0f+fw);
	SoCube *boardBoundary3 = new SoCube;
	boardBoundary3->width.setValue(boundary[0]*2.0f);//c,
	boardBoundary3->height.setValue(boundaryHeight);
	boardBoundary3->depth.setValue(boundaryThickness);
	boardBoundaryRoot3->addChild(boardBoundaryTrans3);
	boardBoundaryRoot3->addChild(boardBoundaryMat);
	boardBoundaryRoot3->addChild(boardBoundary3);
	shape->addChild(boardBoundaryRoot3);

	SoSeparator *boardBoundaryRoot4 = new SoSeparator;
	SoTransform *boardBoundaryTrans4 = new SoTransform;
	boardBoundaryTrans4->translation.setValue(
		fwx, 0.0f, -boundary[2] + boundaryThickness/2.0f+fw);//cigil
	SoCube *boardBoundary4 = new SoCube;
	boardBoundary4->width.setValue(boundary[0]*2.0f);//cigil
	boardBoundary4->height.setValue(boundaryHeight);
	boardBoundary4->depth.setValue(boundaryThickness);
	boardBoundaryRoot4->addChild(boardBoundaryTrans4);
	boardBoundaryRoot4->addChild(boardBoundaryMat);
	boardBoundaryRoot4->addChild(boardBoundary4);
	shape->addChild(boardBoundaryRoot4);

	/////////////////////////////////////////////////////////////////////

	SoSeparator *board2 = new SoSeparator;
	shape->addChild(board2);

	SoTransform *boardTrans2 = new SoTransform;
	//SoCube *boardCube = new SoCube;

	SoSeparator *target_rec2=new SoSeparator;
	//SoCube *rectangles=new SoCube;
	//rectangles->width.setValue(10.0f);//cigil
	//rectangles->height.setValue(recHeight);
	//rectangles->depth.setValue(10.0f);
	SoTransform *targetTrans2 = new SoTransform;
	//SoMaterial *targetMat2 = new SoMaterial;
	//targetMat2->diffuseColor.setValue(COLOR_GREEN);
	targetTrans2->translation.setValue(
		fwx+floorWidth*0.5-8+OFFSET, -recHeight*0.5+0.5,  boundary[2]- boundaryThickness/2.0f+fw-new_depth*0.5-0.5);//cigil
	//SoMaterial *targetMat = new SoMaterial;
	targetMat2 ->diffuseColor.setValue(COLOR_GREEN);
	target_rec2->addChild(targetTrans2);
	target_rec2->addChild(targetMat2);
	target_rec2->addChild(rectangles);
	shape->addChild(target_rec2);

	//boardTrans->translation.setValue(0.0f, -floorHeight/2, 0.0f);//cigil
	boardTrans2->translation.setValue(fwx+OFFSET, -floorHeight/2, fw);
	//boardCube->height.setValue(floorHeight);
	//boardCube->width.setValue(floorWidth);//cigil
	//boardCube->depth.setValue(new_depth);//floorDepth);//cigil

	board2->addChild(boardMat);
	board2->addChild(boardTrans2);
	board2->addChild(boardCube);

	// add walls
	//SoMaterial *boardBoundaryMat = new SoMaterial();
	//boardBoundaryMat->diffuseColor.setValue(COLOR_BOARD_BOUNDARY);

	SoSeparator *boardBoundaryRoot5 = new SoSeparator;
	SoTransform *boardBoundaryTrans5 = new SoTransform;
	boardBoundaryTrans5->translation.setValue( 
		boundary[0] - boundaryThickness/2.0f+fwx+OFFSET, 0.0f, fw);//cigil
	//SoCube *boardBoundary1 = new SoCube;
	//boardBoundary1->width.setValue(boundaryThickness);
	//boardBoundary1->height.setValue(boundaryHeight);
	//boardBoundary1->depth.setValue(boundary[2]*2);//cigil
	boardBoundaryRoot5->addChild(boardBoundaryTrans5);
	boardBoundaryRoot5->addChild(boardBoundaryMat);
	boardBoundaryRoot5->addChild(boardBoundary1);
	shape->addChild(boardBoundaryRoot5);

	SoSeparator *boardBoundaryRoot6 = new SoSeparator;
	SoTransform *boardBoundaryTrans6= new SoTransform;
	boardBoundaryTrans6->translation.setValue(
		-boundary[0] + boundaryThickness/2.0f+fwx+OFFSET, 0.0f, fw);//cigil
	//SoCube *boardBoundary2 = new SoCube;
	//boardBoundary2->width.setValue(boundaryThickness);
	//boardBoundary2->height.setValue(boundaryHeight);
	//boardBoundary2->depth.setValue(boundary[2]*2.0f);
	boardBoundaryRoot6->addChild(boardBoundaryTrans6);
	boardBoundaryRoot6->addChild(boardBoundaryMat);
	boardBoundaryRoot6->addChild(boardBoundary2);
	shape->addChild(boardBoundaryRoot6);

	SoSeparator *boardBoundaryRoot7 = new SoSeparator;
	SoTransform *boardBoundaryTrans7 = new SoTransform;
	boardBoundaryTrans7->translation.setValue(
		fwx+OFFSET, 0.0f, boundary[2] - boundaryThickness/2.0f+fw);
	//SoCube *boardBoundary3 = new SoCube;
	//boardBoundary3->width.setValue(boundary[0]*2.0f);//c,
	//boardBoundary3->height.setValue(boundaryHeight);
	//boardBoundary3->depth.setValue(boundaryThickness);
	boardBoundaryRoot7->addChild(boardBoundaryTrans7);
	boardBoundaryRoot7->addChild(boardBoundaryMat);
	boardBoundaryRoot7->addChild(boardBoundary3);
	shape->addChild(boardBoundaryRoot7);

	SoSeparator *boardBoundaryRoot8 = new SoSeparator;
	SoTransform *boardBoundaryTrans8 = new SoTransform;
	boardBoundaryTrans8->translation.setValue(
		fwx+OFFSET, 0.0f, -boundary[2] + boundaryThickness/2.0f+fw);//cigil
	//SoCube *boardBoundary4 = new SoCube;
	//boardBoundary4->width.setValue(boundary[0]*2.0f);//cigil
	//boardBoundary4->height.setValue(boundaryHeight);
	//boardBoundary4->depth.setValue(boundaryThickness);
	boardBoundaryRoot8->addChild(boardBoundaryTrans8);
	boardBoundaryRoot8->addChild(boardBoundaryMat);
	boardBoundaryRoot8->addChild(boardBoundary4);
	shape->addChild(boardBoundaryRoot8);

#ifdef START_TARGET
	SoSeparator *start_rec=new SoSeparator;
	SoSeparator *target_rec=new SoSeparator;
	SoCube *rectangles=new SoCube;
	rectangles->width.setValue(10.0f);//cigil
	rectangles->height.setValue(boundaryHeight);
	rectangles->depth.setValue(10.0f);
	SoTransform *startTrans = new SoTransform;
	SoTransform *targetTrans = new SoTransform;
	startTrans->translation.setValue(
		fwx-floorWidth*0.5+8, -boundaryHeight+3,  - boundaryThickness/2.0f+fw-28);//cigil
	SoMaterial *startMat = new SoMaterial;
	startMat ->diffuseColor.setValue(COLOR_BLUE);
	start_rec->addChild(startTrans);
	start_rec->addChild(startMat);
	start_rec->addChild(rectangles);
	shape->addChild(start_rec);

	targetTrans->translation.setValue(
		fwx+floorWidth*0.5-8, -boundaryHeight+3,  - boundaryThickness/2.0f+fw-28);//cigil
	SoMaterial *targetMat = new SoMaterial;
	targetMat ->diffuseColor.setValue(COLOR_GREEN);
	target_rec->addChild(targetTrans);
	target_rec->addChild(targetMat);
	target_rec->addChild(rectangles);
	shape->addChild(target_rec);
#endif
	return shape;
}
/*
void BoardGraphics::setColor(Vector colorb)
{
//boardBoundaryMat->diffuseColor.setValue(colorb);

}
void BoardGraphics::setColorVector(Vector cb)
{
//boardBoundaryColor=cb;
}
Vector BoardGraphics::getColorVector()
{
return 0;//boardBoundaryColor;
}*/

void BoardGraphics::setHit(bool hitOrMiss)
{
	hitB=hitOrMiss;
}
void BoardGraphics::setArrived1(bool arrivedOrNot)
{
	arrived1=arrivedOrNot;
}
void BoardGraphics::setArrived2(bool arrivedOrNot)
{
	arrived2=arrivedOrNot;
}
bool BoardGraphics::isHit()
{
	return hitB;
}
bool BoardGraphics::isArrived1()
{
	return arrived1;
}

bool BoardGraphics::isArrived2()
{
	return arrived2;
}