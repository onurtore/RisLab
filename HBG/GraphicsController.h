#pragma once
#include "Public.h"
#include "TargetGraphics.h"
#include "ObstacleGraphics.h"
#include "BoardGraphics.h"
#include "hIPgraphics.h"
#include "BallGr.h"


typedef SbVec3f Point;
typedef SbVec3f Vector;
extern float ballRadius;
const int closeEnough = 10;

class GraphicsController
{
public:
	vector<TargetGraphics*>		*targetGrs;
	vector<TargetGraphics*>		*masterTargetGrs;
	// AYSE: add obstacles to the game
	vector<ObstacleGraphics*>	*obstacleGrs;
	vector<ObstacleGraphics*>	*masterObstacleGrs;
	
	BoardGraphics *boardGr;
	BallGr *ballGr;
	int activeObs; //AYSE: id of the active obstacle

	SoSeparator *root;
	SoUnits *myUnits;
	hIPgraphics iPs[3];
	bool hitBounds[OBSTACLE_COUNT][NUM_OBS_CORNERS-1];
	bool rescueFromPit;

	GraphicsController(int level = LEVEL_DEFAULT);

	SoSeparator* getRoot();
	void addTarget(Point pos);
	void addObstacle(Point pos, float angle = 0);
	void addBonus(Point pos, float pt);
	void addTunnel(Point* keypointPos, int keypointCount, float angle = 0);
	void setActiveObstacle(int index);
	void resetActiveObstacles();

	// AYSE: get/set the level of the game
	int  getLevel();
	void setLevel(int level);

	void addBall();
	void addStylus();
	void addInterfacePoint(int ipID);
	void addBoard();

	bool ballIntersectsLine(Point endPt1, Point endPt2);

	vector<TargetGraphics*> getTargetVec();
private:
	int level;
};
