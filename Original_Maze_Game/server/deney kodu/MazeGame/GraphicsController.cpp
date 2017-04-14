#include "GraphicsController.h"
#include "HIPgraphics.h"
#include "wall.h"
#include "Public.h"
#include "force_vector.h"
#include "spring.h"

GraphicsController::GraphicsController(int scenario, int cond)
{
	root1 = new SoSeparator();
	root2 = new SoSeparator();
	root1->ref();

	myUnits = new SoUnits;
	myUnits->units = SoUnits::METERS;
	root1->addChild(myUnits);


    sprC = new SoSeparator; 
	sprH = new SoSeparator; 
	sprC2 = new SoSeparator; 
	sprH2 = new SoSeparator; 
	sprC->setName("SPRC"); 
	sprH->setName("SPRH"); 
	sprC->setName("SPRC2"); 
	sprH->setName("SPRH2"); 
	sprGrC  = new Spring( sprC,Point(10,0,10), 0.1f,0.1f, _CIP); 
	sprGrH  = new Spring(sprH,Point(10,0,10), 0.1f,0.1f, _HIP); 
	sprGrC2 = new Spring( sprC2,Point(10,0,10), 0.1f,0.1f, _CIP2); 
	sprGrH2 = new Spring(sprH2,Point(10,0,10), 0.1f,0.1f, _HIP2); 

	boardGr			= new BoardGraphics(2, BOARD_WIDTH/2, 5, scenario, cond);
	boardGr2		= new BoardGraphics(2, BOARD_WIDTH/2, 5, scenario, cond);
	wall1 = new Wall(wallHeight, wallWidth, wallDepth ,wallTx,wallTy,wallTz);
	wall2 = new Wall(wallHeight, wallWidth23, wallDepth23 ,wallTx2,wallTy,wallTz2);
	wall3 = new Wall(wallHeight, wallWidth23, wallDepth23 ,wallTx3,wallTy,wallTz3);
	


}

SoSeparator* GraphicsController::getRoot1()
{
	return root1;
}


SoSeparator* GraphicsController::getRoot2()
{
	return root2;
}

void GraphicsController::addBall()
{
	ballGr = new BallGr(root1, Point(SHIFT_X,0.8f,SHIFT_Z), BALL_WIDTH, BALL_HEIGHT, BALL_DEPTH, BALL_RADIUS, BALL_MASS, Vector(0.9f, 0.2f, 0.4f));
	ballGr2 = new BallGr(root2, Point(SHIFT_X,0.8f,SHIFT_Z), BALL_WIDTH,BALL_HEIGHT, BALL_DEPTH, BALL_RADIUS, BALL_MASS, Vector(0.9f, 0.2f, 0.4f));
}

void GraphicsController::addVector()
{
		fvector = new force_vector(10.0f,Vector(BALL_WIDTH*0.5,0.8f,BALL_DEPTH*0.5),Vector(0.9f, 0.2f, 0.4f));
		root1->addChild(fvector->vectorShape);
		root2->addChild(fvector->vectorShape);
		
		hvector=new force_vector(10.0f,Vector(BALL_WIDTH*0.5,0.8f,BALL_DEPTH*0.5),COLOR_BLUE);
		root1->addChild(hvector->vectorShape);
		root2->addChild(hvector->vectorShape);
		
		cvector=new force_vector(10.0f,Vector(BALL_WIDTH*0.5,0.8f,BALL_DEPTH*0.5),COLOR_GREEN);
		root1->addChild(cvector->vectorShape);
		root2->addChild(cvector->vectorShape);
		
		reacV=new force_vector(10.0f,Vector(BALL_WIDTH*0.5,0.8f,BALL_DEPTH*0.5),Vector(0.3f, 0.2f, 0.5f));
		root1->addChild(reacV->vectorShape);
		root2->addChild(reacV->vectorShape);
}


void GraphicsController::addInterfacePoints()
{
	HIPgraphics *cipGr = new HIPgraphics(root1, 
		Point(0,3,0), 0, COLOR_CIP,transhc);
	HIPgraphics *cipGr2 = new HIPgraphics(root2, 
		Point(0,3,0), 0, COLOR_CIP,transhc);
	cip = *cipGr;
	cip2 = *cipGr2;

	HIPgraphics *hipGr = new HIPgraphics(root1, 
			Point(0,3,0), 0, COLOR_HIP,transhc);
	HIPgraphics *hipGr2 = new HIPgraphics(root2, 
			Point(0,3,0), 0, COLOR_HIP,transhc);
	hip = *hipGr;
	hip2 = *hipGr2;

	HIPgraphics *nipGr = new HIPgraphics(root1, 
			Point(0,3,0), 0.005, COLOR_NIP,transn);
		
	nip = *nipGr;


	HIPgraphics *handleCGr = new HIPgraphics(root1, 
			Point(0,2,0), 0, COLOR_CIP,transn);
	HIPgraphics *handleCGr2 = new HIPgraphics(root2, 
			Point(0,2,0), 0, COLOR_CIP,transn);
	handleC = *handleCGr;
	handleC2 = *handleCGr2;
	
	HIPgraphics *handleHGr = new HIPgraphics(root1, 
		Point(0,2,0), 0, COLOR_HIP,transn);
	HIPgraphics *handleHGr2 = new HIPgraphics(root2, 
		Point(0,2,0), 0, COLOR_HIP,transn);
			
	handleH = *handleHGr;
	handleH2 = *handleHGr2;
}

void GraphicsController::addBoard()
{
	root1->addChild(boardGr->shape);
	root2->addChild(boardGr2->shape);
}


void GraphicsController::addWall()
{
	root1->addChild(wall1->wallShape);
	root1->addChild(wall2->wallShape);
	root1->addChild(wall3->wallShape);

	root2->addChild(wall1->wallShape);
	root2->addChild(wall2->wallShape);
	root2->addChild(wall3->wallShape);
}

void GraphicsController::addSpring()
{
	root1->addChild(sprC);
	root1->addChild(sprH);
	root2->addChild(sprC2);
	root2->addChild(sprH2);
}


