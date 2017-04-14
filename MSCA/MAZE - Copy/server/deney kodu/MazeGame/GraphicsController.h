#pragma once
#include "Public.h"
#include "BoardGraphics.h"
#include "HIPgraphics.h"
#include "BallGr.h"
#include "wall.h"
#include "force_vector.h"
#include "spring.h"


class GraphicsController
{
public:	
	BoardGraphics *boardGr;
	BoardGraphics *boardGr2;
	BallGr *ballGr;
	BallGr *ballGr2;
	Wall *wall1;
	Wall *wall2;
	Wall *wall3;

	force_vector *fvector;
	force_vector *hvector;
	force_vector *cvector;
	force_vector *reacV;

	Spring *sprGrC; 
	Spring *sprGrH; 
	
	SoSeparator *sprC;  
	SoSeparator *sprH;  

	Spring *sprGrC2; 
	Spring *sprGrH2; 
	
	SoSeparator *sprC2;  
	SoSeparator *sprH2; 

	SoSeparator *root1;
	SoSeparator *root2;



	SoUnits *myUnits;
	HIPgraphics hip;
	HIPgraphics cip;
	HIPgraphics hip2;
	HIPgraphics cip2;
	HIPgraphics nip;
	HIPgraphics handleC;
	HIPgraphics handleH;
	HIPgraphics handleC2;
	HIPgraphics handleH2;

	GraphicsController(int scenario, int cond);

	SoSeparator* getRoot1();
	SoSeparator* getRoot2();

	void addBall();
	//Onur
	void addBall(double x, double y);
	//Onur
	void addInterfacePoints();
	void addBoard();
	void addWall();
	void addSpring();
	void addVector();
	bool ballIntersectsLine(float wall_x, float wall_z);
    bool nipIntersectsLine(float wall_x, float wall_z);
	
};
