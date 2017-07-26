#include "GraphicsController.h"
#include "BoardGraphics.h"

BoardGraphics::BoardGraphics(float bT, float bD, float bH, int scenario, int cond)
{
	boundaryThickness	= bT;
	boundaryDim			= bD;
	boundaryHeight		= bH;

	float len	= boundaryDim+boundaryThickness;
	float lenz;
	if( scenario != MIXED)
	{
		lenz = BOARD_DEPTH_STR * 0.5 + boundaryThickness;
	}
	else
	{
		lenz = BOARD_DEPTH_MIXED * 0.5 + boundaryThickness;
	}

	

	floorHeight	= 5;
	if(scenario!= ROTATIONAL )
	{
		boundary	= Vector(len, len, lenz);
		floorWidth	= (boundary[0]-boundaryThickness)*2;
		floorDepth	= (boundary[2]-boundaryThickness)*2;

	}
	else
	{	
		boundary	= Vector(BOARD_RADIUS, len, BOARD_RADIUS);
		floorWidth = BOARD_RADIUS*2;
		floorDepth = BOARD_RADIUS*2;
	}
	hitB=false;
	arrived=false;
	hitObstacle = false;


	trialBoardGame=1;
	targetTrans = new SoTransform;
	initialization=false;

	/*boardBoundaryRoot1=new SoSeparator(); 
	boardBoundaryRoot2=new SoSeparator(); 
	boardBoundaryRoot3=new SoSeparator(); 
	boardBoundaryRoot4=new SoSeparator();*/

	//boardBoundaryColor=Vector(0.7f,0.8f,0.4f);//COLOR_BOARD_BOUNDARY;
	boardBoundaryMat = new SoMaterial();
	ObstacleMat = new SoMaterial();
	boardMat = new SoMaterial();
	targetMat =new SoMaterial();

	targetCount = 0;

	shape = constructBoardShape(scenario, cond);
}

SoSeparator * BoardGraphics::constructBoardShape(int scenario, int cond)
{
	if (scenario == STRAIGHT)
		return constructBoardShape4Straight();
	else if ( scenario == MIXED)
		return constructBoardShape4Mixed();
	else if (scenario == ROTATIONAL)
		return constructBoardShape4Rotational(cond);
	else 
		return NULL;
}


SoSeparator * BoardGraphics::constructBoardShape4Rotational(int cond)
{
	SoSeparator* shape = new SoSeparator;

	// add floor
	SoSeparator *board = new SoSeparator;
	shape->addChild(board);

	//SoMaterial *boardMat = new SoMaterial;
	boardMat ->diffuseColor.setValue(COLOR_BOARD);
	SoTransform *boardTrans = new SoTransform;
	SoCube *boardBody = new SoCube;

	SoSeparator *target_rec=new SoSeparator;
	SoCube *rectangles=new SoCube;
	rectangles->width.setValue(TARGET_WIDTH);//cigil
	rectangles->height.setValue(TARGET_HEIGHT);
	rectangles->depth.setValue(TARGET_DEPTH);

	targetTrans->translation.setValue(-500, 0,0);
	targetMat ->diffuseColor.setValue(COLOR_GREEN);
	target_rec->addChild(targetTrans);
	target_rec->addChild(targetMat);
	target_rec->addChild(rectangles);

	SoSeparator *targetCy1 = new SoSeparator;
	SoSeparator *targetCy2 = new SoSeparator;

	SoCylinder *cylinder1 = new SoCylinder;
	SoCylinder *cylinder2 = new SoCylinder;
	cylinder1->radius.setValue(IP_RADIUS);
	cylinder1->height.setValue(IP_TARGET_HEIGHT);
	cylinder2->radius.setValue(IP_RADIUS);
	cylinder2->height.setValue(IP_TARGET_HEIGHT);

	SoTransform *targetCyTrans1 = new SoTransform;
	targetCyTrans1->translation.setValue(-TARGET_WIDTH/2, BALL_RADIUS/2, 0);

	SoTransform *targetCyTrans2 = new SoTransform;
	targetCyTrans2->translation.setValue(TARGET_WIDTH/2, BALL_RADIUS/2, 0);

	SoMaterial *targetCyMat1 = new SoMaterial;
	targetCyMat1->diffuseColor.setValue(COLOR_BLUE_MATTE);

	SoMaterial *targetCyMat2 = new SoMaterial;
	targetCyMat2 ->diffuseColor.setValue(COLOR_GREEN_MATTE);

	targetCy1->addChild(targetCyMat1);
	targetCy1->addChild(targetCyTrans1);
	targetCy1->addChild(cylinder1);

	targetCy2->addChild(targetCyMat2);
	targetCy2->addChild(targetCyTrans2);
	targetCy2->addChild(cylinder2);

	target_rec->addChild(targetCy1);
	target_rec->addChild(targetCy2);

	shape->addChild(target_rec);

	boardTrans->translation.setValue(SHIFT_X, -floorHeight/2, SHIFT_Z);
	boardBody->height.setValue(floorHeight);
	boardBody->width.setValue(BOARD_RADIUS*2);//cigil
	boardBody->depth.setValue(BOARD_RADIUS*2);//cigil

	board->addChild(boardMat);
	board->addChild(boardTrans);
	board->addChild(boardBody);

	// add obstacles

	//float middleGap = BALL_WIDTH * 4 / 5.0; // the gap between two obstacles
	//float obstacleDepth = BOARD_RADIUS - middleGap/2;
	boardBoundaryMat->diffuseColor.setValue(COLOR_BOARD_BOUNDARY);


	SoSeparator *boardBoundaryRoot1 = new SoSeparator;
	SoTransform *boardBoundaryTrans1 = new SoTransform;
	boardBoundaryTrans1->translation.setValue(boundary[0] - boundaryThickness/2.0f+SHIFT_X, 0.0f, SHIFT_Z);//cigil
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
	boardBoundaryTrans2->translation.setValue(-boundary[0] + boundaryThickness/2.0f+SHIFT_X, 0.0f, SHIFT_Z);//cigil
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
	boardBoundaryTrans3->translation.setValue(SHIFT_X, 0.0f, boundary[2] - boundaryThickness/2.0f+SHIFT_Z);
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
	boardBoundaryTrans4->translation.setValue(SHIFT_X, 0.0f, -boundary[2] + boundaryThickness/2.0f+SHIFT_Z);//cigil
	SoCube *boardBoundary4 = new SoCube;
	boardBoundary4->width.setValue(boundary[0]*2.0f);//cigil
	boardBoundary4->height.setValue(boundaryHeight);
	boardBoundary4->depth.setValue(boundaryThickness);
	boardBoundaryRoot4->addChild(boardBoundaryTrans4);
	boardBoundaryRoot4->addChild(boardBoundaryMat);
	boardBoundaryRoot4->addChild(boardBoundary4);
	shape->addChild(boardBoundaryRoot4);

	//Wall *w1 = new Wall(boundaryHeight, boundaryThickness, obstacleDepth, 
	//					SHIFT_X, 0, SHIFT_Z + (obstacleDepth + middleGap) * 0.5);
	//shape->addChild(w1->wallShape);

	//Wall *w2 = new Wall(boundaryHeight, boundaryThickness, obstacleDepth, 
	//					SHIFT_X, 0, SHIFT_Z - (obstacleDepth + middleGap) * 0.5);
	//shape->addChild(w1->wallShape);

	SoSeparator *obstacleRoot1 = new SoSeparator;
	SoTransform *obstacleTrans1 = new SoTransform;
	ObstacleMat->diffuseColor.setValue(COLOR_BOARD_BOUNDARY);

	obstacleTrans1->translation.setValue(SHIFT_X, 0.0f, SHIFT_Z + (obstacleDepth + middleGap) * 0.5);//cigil
	SoCube *obstacle1 = new SoCube;
	obstacle1->width.setValue(boundaryThickness);
	obstacle1->height.setValue(boundaryHeight);
	obstacle1->depth.setValue(obstacleDepth);
	obstacleRoot1->addChild(obstacleTrans1);
	obstacleRoot1->addChild(ObstacleMat);
	obstacleRoot1->addChild(obstacle1);

	shape->addChild(obstacleRoot1);

	// two walls in full conflict
	if (cond == FULL_CONFLICT)
	{
		SoSeparator *obstacleRoot2 = new SoSeparator;
		SoTransform *obstacleTrans2 = new SoTransform;

		obstacleTrans2->translation.setValue(SHIFT_X, 0.0f, SHIFT_Z - (obstacleDepth + middleGap) * 0.5);//cigil
		SoCube *obstacle2 = new SoCube;
		obstacle2->width.setValue(boundaryThickness);
		obstacle2->height.setValue(boundaryHeight);
		obstacle2->depth.setValue(obstacleDepth);
		obstacleRoot2->addChild(obstacleTrans2);
		obstacleRoot2->addChild(boardBoundaryMat);
		obstacleRoot2->addChild(obstacle2);

		shape->addChild(obstacleRoot2);
	}
	return shape;
}


SoSeparator * BoardGraphics::constructBoardShape4Mixed()
{	
	SoSeparator* shape = new SoSeparator;
	SoSeparator *board = new SoSeparator;
	shape->addChild(board);

	//SoMaterial *boardMat = new SoMaterial;
	boardMat ->diffuseColor.setValue(COLOR_BOARD);
	SoTransform *boardTrans = new SoTransform;
	SoCube *boardBody = new SoCube;

	SoSeparator *target_rec=new SoSeparator;
	SoCube *rectangles=new SoCube;
	rectangles->width.setValue(TARGET_WIDTH);//cigil
	rectangles->height.setValue(TARGET_HEIGHT);
	rectangles->depth.setValue(TARGET_DEPTH);

	targetTrans->translation.setValue(-500, 0,0);
	targetMat ->diffuseColor.setValue(COLOR_GREEN);
	target_rec->addChild(targetTrans);
	target_rec->addChild(targetMat);
	target_rec->addChild(rectangles);

	SoSeparator *targetCy1 = new SoSeparator;
	SoSeparator *targetCy2 = new SoSeparator;

	SoCylinder *cylinder1 = new SoCylinder;
	SoCylinder *cylinder2 = new SoCylinder;
	cylinder1->radius.setValue(IP_RADIUS);
	cylinder1->height.setValue(IP_TARGET_HEIGHT);
	cylinder2->radius.setValue(IP_RADIUS);
	cylinder2->height.setValue(IP_TARGET_HEIGHT);

	SoTransform *targetCyTrans1 = new SoTransform;
	targetCyTrans1->translation.setValue(-TARGET_WIDTH/2, BALL_RADIUS/2, 0);

	SoTransform *targetCyTrans2 = new SoTransform;
	targetCyTrans2->translation.setValue(TARGET_WIDTH/2, BALL_RADIUS/2, 0);

	SoMaterial *targetCyMat1 = new SoMaterial;
	targetCyMat1->diffuseColor.setValue(COLOR_BLUE_MATTE);

	SoMaterial *targetCyMat2 = new SoMaterial;
	targetCyMat2 ->diffuseColor.setValue(COLOR_GREEN_MATTE);

	targetCy1->addChild(targetCyMat1);
	targetCy1->addChild(targetCyTrans1);
	targetCy1->addChild(cylinder1);

	targetCy2->addChild(targetCyMat2);
	targetCy2->addChild(targetCyTrans2);
	targetCy2->addChild(cylinder2);

	target_rec->addChild(targetCy1);
	target_rec->addChild(targetCy2);


	shape->addChild(target_rec);

	boardTrans->translation.setValue(SHIFT_X, -floorHeight/2, SHIFT_Z);
	boardBody->height.setValue(floorHeight);
	boardBody->width.setValue(floorWidth);//cigil
	boardBody->depth.setValue(BOARD_DEPTH_MIXED);//floorDepth);//cigil

	board->addChild(boardMat);
	board->addChild(boardTrans);
	board->addChild(boardBody);

	// add walls
	//SoMaterial *boardBoundaryMat = new SoMaterial();
	boardBoundaryMat->diffuseColor.setValue(COLOR_BOARD_BOUNDARY);

	SoSeparator *boardBoundaryRoot1 = new SoSeparator;
	SoTransform *boardBoundaryTrans1 = new SoTransform;
	boardBoundaryTrans1->translation.setValue(boundary[0] - boundaryThickness/2.0f+SHIFT_X, 0.0f, SHIFT_Z);//cigil
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
	boardBoundaryTrans2->translation.setValue(-boundary[0] + boundaryThickness/2.0f+SHIFT_X, 0.0f, SHIFT_Z);//cigil
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
	boardBoundaryTrans3->translation.setValue(SHIFT_X, 0.0f, boundary[2] - boundaryThickness/2.0f+SHIFT_Z);
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
	boardBoundaryTrans4->translation.setValue(SHIFT_X, 0.0f, -boundary[2] + boundaryThickness/2.0f+SHIFT_Z);//cigil
	SoCube *boardBoundary4 = new SoCube;
	boardBoundary4->width.setValue(boundary[0]*2.0f);//cigil
	boardBoundary4->height.setValue(boundaryHeight);
	boardBoundary4->depth.setValue(boundaryThickness);
	boardBoundaryRoot4->addChild(boardBoundaryTrans4);
	boardBoundaryRoot4->addChild(boardBoundaryMat);
	boardBoundaryRoot4->addChild(boardBoundary4);
	shape->addChild(boardBoundaryRoot4);


	return shape;
}

SoSeparator* BoardGraphics::constructBoardShape4Straight()
{
	SoSeparator* shape = new SoSeparator;

	// add floor
	SoSeparator *board = new SoSeparator;
	shape->addChild(board);

	//SoMaterial *boardMat = new SoMaterial;
	boardMat ->diffuseColor.setValue(COLOR_BOARD);
	SoTransform *boardTrans = new SoTransform;
	SoCube *boardBody = new SoCube;

	SoSeparator *target_rec=new SoSeparator;
	SoCube *rectangles=new SoCube;
	rectangles->width.setValue(TARGET_WIDTH);//cigil
	rectangles->height.setValue(TARGET_HEIGHT);
	rectangles->depth.setValue(TARGET_DEPTH);

	targetTrans->translation.setValue(-500, 0,0);
	targetMat ->diffuseColor.setValue(COLOR_GREEN);
	target_rec->addChild(targetTrans);
	target_rec->addChild(targetMat);
	target_rec->addChild(rectangles);

	SoSeparator *targetCy1 = new SoSeparator;
	SoSeparator *targetCy2 = new SoSeparator;

	SoCylinder *cylinder1 = new SoCylinder;
	SoCylinder *cylinder2 = new SoCylinder;
	cylinder1->radius.setValue(IP_RADIUS);
	cylinder1->height.setValue(IP_TARGET_HEIGHT);
	cylinder2->radius.setValue(IP_RADIUS);
	cylinder2->height.setValue(IP_TARGET_HEIGHT);

	SoTransform *targetCyTrans1 = new SoTransform;
	targetCyTrans1->translation.setValue(-TARGET_WIDTH/2, BALL_RADIUS/2, 0);

	SoTransform *targetCyTrans2 = new SoTransform;
	targetCyTrans2->translation.setValue(TARGET_WIDTH/2, BALL_RADIUS/2, 0);

	SoMaterial *targetCyMat1 = new SoMaterial;
	targetCyMat1->diffuseColor.setValue(COLOR_BLUE_MATTE);

	SoMaterial *targetCyMat2 = new SoMaterial;
	targetCyMat2 ->diffuseColor.setValue(COLOR_GREEN_MATTE);

	targetCy1->addChild(targetCyMat1);
	targetCy1->addChild(targetCyTrans1);
	targetCy1->addChild(cylinder1);

	targetCy2->addChild(targetCyMat2);
	targetCy2->addChild(targetCyTrans2);
	targetCy2->addChild(cylinder2);

	target_rec->addChild(targetCy1);
	target_rec->addChild(targetCy2);


	shape->addChild(target_rec);

	boardTrans->translation.setValue(SHIFT_X, -floorHeight/2, SHIFT_Z);
	boardBody->height.setValue(floorHeight);
	boardBody->width.setValue(floorWidth);//cigil
	boardBody->depth.setValue(BOARD_DEPTH_STR);//floorDepth);//cigil

	board->addChild(boardMat);
	board->addChild(boardTrans);
	board->addChild(boardBody);

	// add walls
	//SoMaterial *boardBoundaryMat = new SoMaterial();
	boardBoundaryMat->diffuseColor.setValue(COLOR_BOARD_BOUNDARY);

	SoSeparator *boardBoundaryRoot1 = new SoSeparator;
	SoTransform *boardBoundaryTrans1 = new SoTransform;
	boardBoundaryTrans1->translation.setValue(boundary[0] - boundaryThickness/2.0f+SHIFT_X, 0.0f, SHIFT_Z);//cigil
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
	boardBoundaryTrans2->translation.setValue(-boundary[0] + boundaryThickness/2.0f+SHIFT_X, 0.0f, SHIFT_Z);//cigil
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
	boardBoundaryTrans3->translation.setValue(SHIFT_X, 0.0f, boundary[2] - boundaryThickness/2.0f+SHIFT_Z);
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
	boardBoundaryTrans4->translation.setValue(SHIFT_X, 0.0f, -boundary[2] + boundaryThickness/2.0f+SHIFT_Z);//cigil
	SoCube *boardBoundary4 = new SoCube;
	boardBoundary4->width.setValue(boundary[0]*2.0f);//cigil
	boardBoundary4->height.setValue(boundaryHeight);
	boardBoundary4->depth.setValue(boundaryThickness);
	boardBoundaryRoot4->addChild(boardBoundaryTrans4);
	boardBoundaryRoot4->addChild(boardBoundaryMat);
	boardBoundaryRoot4->addChild(boardBoundary4);
	shape->addChild(boardBoundaryRoot4);

	return shape;
}

void BoardGraphics::setHit(bool hitOrMiss)
{
	hitB = hitOrMiss;
}

void BoardGraphics::setArrived(bool arrivedOrNot)
{
	arrived = arrivedOrNot;
}


bool BoardGraphics::isHit()
{
	return hitB;
}

bool BoardGraphics::isArrived()
{
	return arrived;
}

void BoardGraphics::setTrial(int t)
{
	trialBoardGame=t;
}
int BoardGraphics::getTrial()
{
	return trialBoardGame;
}
bool BoardGraphics::getInit()
{
	return initialization;
}
void BoardGraphics::setInit(bool seti)
{
	initialization=seti;
}

bool BoardGraphics::isHitObstacle()
{
	return hitObstacle;

}
void BoardGraphics::setHitObstacle(bool x)
{
	hitObstacle = x;
}