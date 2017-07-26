#pragma once
#include "Public.h"
#include "BoardGraphics.h"
#include "HIPgraphics.h"
#include "BallGr.h"
#include "BallGr2.h"
#include "wall.h"
#include "force_vector.h"
#include "spring.h"

extern float ballRadius;

class GraphicsController
{
public:	
	BoardGraphics *boardGr;
	BallGr *ballGr;
	BallGr2 *ballGr2;
	wall *wall1;
	wall *wall2;
	wall *wall3;

	force_vector *fvector;
	force_vector *hvector;
	force_vector *cvector;
	force_vector *reacV;

	spring *sprGrC; 
	spring *sprGrH; 
	
	SoSeparator *sprC;  
	SoSeparator *sprH;  

	spring *sprGrC2; 
	spring *sprGrH2; 
	
	SoSeparator *sprC2;  
	SoSeparator *sprH2; 


	SoSeparator *root;
	SoUnits *myUnits;
	HIPgraphics hip;
	HIPgraphics cip;
	HIPgraphics hip2;
	HIPgraphics cip2;
	HIPgraphics nip;
	HIPgraphics handleC;
	HIPgraphics handleH;

	GraphicsController();

	SoSeparator* getRoot();

	void addBall();
	void addInterfacePoint(int ipID);
	void addBoard();
	void addWall();
	void addSpring();
	void addVector();
	bool ballIntersectsLine(float wall_x, float wall_z);
    bool nipIntersectsLine(float wall_x, float wall_z);
	
};
