#pragma once

#include "Public.h"
#include <Inventor/SbBasic.h>
#include "Obstacle.h"


#define OBSTACLE_COLOR			0.5, 0.5, 0.5
#define OBSTACLE_CONTACT_COLOR	2, 0, 0

class ObstacleGraphics
{
public:
	Obstacle *obs;

	SoTransform *transfMat;

	// define obstacle shape and material
	SoSeparator *shape;
	// two materials for two boundaries
	SoMaterial *mat1; 
	SoMaterial *mat2; 

	// two transforms for two boundaries
	SoTransform *trans1; 
	SoTransform *trans2;

	SoMaterial *cornerMat;
	SoDrawStyle *obsCornersDrawStyle;

	Point obstacleCornerPts [NUM_OBS_CORNERS];
	
	ObstacleGraphics(int level);
	// angle holds degrees of rotation around y axis
	ObstacleGraphics(SoSeparator *root, Point pos, int level, float angle = 0, 
					 float w = OBSTACLE_WIDTH, float d = OBSTACLE_DEPTH);
	~ObstacleGraphics();
	
	SoSeparator* constructShape();
	void changeColor(float r, float g, float b, int whichBound);
	Point getPosition();
	float getWidth();
	float getHeight();
	float getDepth();
	float getAngle();


	int   getLevel();
	void  setLevel(int level);

	void rotate(float angle); // rotate obstacle around y in cw by angle degrees

	float distanceSq(Point pt); 
	float distance(Point pt); 
};
         