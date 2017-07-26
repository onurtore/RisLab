#include "GraphicsController.h"
#include "HIPgraphics.h"
#include "wall.h"
#include "Public.h"
#include "force_vector.h"
#include "spring.h"

GraphicsController::GraphicsController()
{

	root = new SoSeparator();
	root->ref();

	myUnits = new SoUnits;
	myUnits->units = SoUnits::METERS;
	root->addChild(myUnits);


    sprC = new SoSeparator; 
	sprH = new SoSeparator; 
	sprC2 = new SoSeparator; 
	sprH2 = new SoSeparator; 
	sprC->setName("SPRC"); 
	sprH->setName("SPRH"); 
	sprC->setName("SPRC2"); 
	sprH->setName("SPRH2"); 
	sprGrC = new spring( sprC,Point(10,0,10), 0.1f,0.1f, _CIP); 
	sprGrH = new spring(sprH,Point(10,0,10), 0.1f,0.1f, _HIP); 
	sprGrC2 = new spring( sprC2,Point(10+OFFSET,0,10), 0.1f,0.1f, _CIP2); 
	sprGrH2 = new spring(sprH2,Point(10+OFFSET,0,10), 0.1f,0.1f, _HIP2); 

	boardGr		= new BoardGraphics(2, BOARD_WIDTH/2, 5);
	wall1= new wall(wallHeight, wallWidth, wallDepth ,wallTx,wallTy,wallTz);
	wall2= new wall(wallHeight, wallWidth, wallDepth ,wallTx2,wallTy,wallTz2);
	wall3= new wall(wallHeight, wallWidth, wallDepth ,wallTx3,wallTy,wallTz3);
	
}

SoSeparator* GraphicsController::getRoot()
{
	return root;
}

void GraphicsController::addBall()
{
	//ballGr = new BallGr(root, Point(0,0.8f,0), ball_width,ball_height,ball_depth, BALL_RADIUS,25, Vector(0.9f, 0.2f, 0.4f));
	ballGr = new BallGr(root, Point(-BOARD_WIDTH/2+8+fwx,0.8f,-new_depth/2+fw+5), ball_width,ball_height,ball_depth, BALL_RADIUS,25, Vector(0.9f, 0.2f, 0.4f));
	ballGr2 = new BallGr2(root, Point(-BOARD_WIDTH/2+8+fwx+OFFSET,0.8f,-new_depth/2+fw+5), ball_width,ball_height,ball_depth, BALL_RADIUS,25, Vector(0.9f, 0.2f, 0.4f));

}

void GraphicsController::addVector()
{//fvector->getVectorDepth(),fvector->getVectorWidth()
		fvector = new force_vector(10.0f,Vector(ball_width*0.5,0.8f,ball_depth*0.5),Vector(0.9f, 0.2f, 0.4f));
		root->addChild(fvector->vectorShape);
		hvector=new force_vector(10.0f,Vector(ball_width*0.5,0.8f,ball_depth*0.5),COLOR_BLUE);
		root->addChild(hvector->vectorShape);
		cvector=new force_vector(10.0f,Vector(ball_width*0.5,0.8f,ball_depth*0.5),COLOR_GREEN);
		root->addChild(cvector->vectorShape);
		reacV=new force_vector(10.0f,Vector(ball_width*0.5,0.8f,ball_depth*0.5),Vector(0.3f, 0.2f, 0.5f));
		root->addChild(reacV->vectorShape);
}


void GraphicsController::addInterfacePoint(int ipID)
{
	if (ipID == 0) {	// cip
		HIPgraphics *cipGr = new HIPgraphics(root, 
			Point(-BOARD_WIDTH/2+fwx+8,2,fw+-new_depth/2+8), 0, COLOR_CIP,transhc);
		cip = *cipGr;
	}
	else if (ipID == 1) {	// hip
		HIPgraphics *hipGr = new HIPgraphics(root, 
			Point(-BOARD_WIDTH/2+fwx+8,4,fw-new_depth/2+8), 0, COLOR_HIP,transhc);
		hip = *hipGr;
	}
	else if (ipID == 2) {	// nip
		HIPgraphics *nipGr = new HIPgraphics(root, 
			Point(0,3,0), 0.005, COLOR_NIP,transn);
		
		
		nip = *nipGr;
	}
		else if (ipID == 4) {	// hip
		HIPgraphics *hipGr2 = new HIPgraphics(root, 
			Point(-BOARD_WIDTH/2+8+fwx+OFFSET,4,fw-new_depth/2+8), 0, COLOR_HIP,transhc);
		hip2 = *hipGr2;
	}
	else if (ipID == 3) {	// cip
		HIPgraphics *cipGr2 = new HIPgraphics(root, 
			Point(-BOARD_WIDTH/2+8+fwx+OFFSET,2,fw-new_depth/2+8), 0, COLOR_CIP,transhc);
		cip2 = *cipGr2;
	}
	else if (ipID == 20) {	// nip
		HIPgraphics *handleCGr = new HIPgraphics(root, 
			Point(-BOARD_WIDTH/2+fwx+8,3,-new_depth/2+ball_depth*0.5+0.5+fw+8), 0, COLOR_CIP,transn);
		
		
		handleC = *handleCGr;
	}
		else if (ipID == 21) {	// nip
		HIPgraphics *handleHGr = new HIPgraphics(root, 
			Point(-BOARD_WIDTH/2+fwx+8,3,-new_depth/2-ball_depth*0.5-0.5+fw+8), 0, COLOR_HIP,transn);
		
		
		handleH = *handleHGr;
	}
}


void GraphicsController::addBoard()
{
	root->addChild(boardGr->shape);
}
void GraphicsController::addWall()
{
	root->addChild(wall1->wallShape);
	root->addChild(wall2->wallShape);
	root->addChild(wall3->wallShape);
}

void GraphicsController::addSpring()
{
	
	root->addChild(sprC);
	root->addChild(sprH);
	root->addChild(sprC2);
	root->addChild(sprH2);
}


