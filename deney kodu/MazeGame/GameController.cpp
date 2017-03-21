#include "GameController.h"

GameController::GameController()
{
}

void GameController::addGraphicsController(GraphicsController *gC)
{
	grCtrl = (GraphicsController*)malloc( sizeof(*gC) );
	*grCtrl = *gC;
}

void GameController::moveCtrller()
{

}

void GameController::calculatePF()
{
	vector<TargetGraphics*> targetVec;
	vector<TargetGraphics*>::iterator targetIter;

	//for( int i=0; i < 10; i++ ) the_vector.push_back(i);
	int total = 0;
	targetIter = targetVec.begin();
	while( targetIter != targetVec.end() ) 
	{
		total += 1;// AYSE: what is this? *targetIter;
		++targetIter;
	}
}

int GameController::getLevel()
{
	// Assume all obstacles are set to the same level
	if (OBSTACLE_COUNT > 0)
		return grCtrl->obstacleGrs->at(0)->getLevel();
	else
		return -1;
}


void GameController::setLevel(int level)
{
	for (int i = 0; i < OBSTACLE_COUNT; i++)
	{
		grCtrl->obstacleGrs->at(i)->setLevel(level);
	}
}
