#include "GraphicsController.h"
#include "hIPgraphics.h"

GraphicsController::GraphicsController(int level)
{

	root = new SoSeparator();
	root->ref();
	
	myUnits = new SoUnits;
	myUnits->units = SoUnits::METERS;
	root->addChild(myUnits);

	targetGrs	= new vector<TargetGraphics*>;
	obstacleGrs = new vector<ObstacleGraphics*>;

	masterTargetGrs	= new vector<TargetGraphics*>;
	masterObstacleGrs = new vector<ObstacleGraphics*>;

	boardGr		= new BoardGraphics(2, BOARD_WIDTH/2, 5);
	activeObs	= -1;

	rescueFromPit = false;

	for (int i=0; i < OBSTACLE_COUNT; i++)
	{
		for (int j = 0; j < NUM_OBS_CORNERS; j++)
		{
			hitBounds[i][j] = false;
		}
	}

	// AYSE: level of the game is controlled here
	this->level = level;
}

SoSeparator* GraphicsController::getRoot()
{
	return root;
}

vector<TargetGraphics*> GraphicsController::getTargetVec()
{
	return *targetGrs;
}

void GraphicsController::addTarget(Point pos)
{
	TargetGraphics *targetGr = new TargetGraphics(root, pos); 
	masterTargetGrs->push_back(targetGr);
	targetGrs->push_back(targetGr);
}

void GraphicsController::addObstacle(Point pos, float angle)
{
	ObstacleGraphics *obsGr = new ObstacleGraphics(root, pos, level, angle); 
	masterObstacleGrs->push_back(obsGr);
	obstacleGrs->push_back(obsGr);
}

void GraphicsController::setActiveObstacle(int i)
{
	activeObs = i;
}

void GraphicsController::resetActiveObstacles()
{
	activeObs = -1;
}

int GraphicsController::getLevel()
{
	return level;
}


void GraphicsController::setLevel(int level)
{
	this->level = level;
	for (int i = 0; i < OBSTACLE_COUNT; i++)
	{
		obstacleGrs->at(i)->setLevel(level);
	}
}

void GraphicsController::addBall()
{
	ballGr = new BallGr(root, Point(0,0.8f,0), ballRadius, 1, Vector(0.8f, 0.2f, 0.2f));
}

void GraphicsController::addStylus(){}

void GraphicsController::addInterfacePoint(int ipID)
{
	if (ipID == 0) {	// cip
		hIPgraphics *cipGr = new hIPgraphics(root, 
			Point(0,2,0), 0, COLOR_CIP);
		iPs[0] = *cipGr;
	}
	else if (ipID == 1) {	// hip
		hIPgraphics *hipGr = new hIPgraphics(root, 
			Point(0,4,0), 0, COLOR_HIP);
		iPs[1] = *hipGr;
	}
	else if (ipID == 2) {	// nip
		hIPgraphics *nipGr = new hIPgraphics(root, 
			Point(0,3,0), 0, COLOR_NIP);
		iPs[2] = *nipGr;
	}
}


void GraphicsController::addBoard()
{
	root->addChild(boardGr->shape);
}


bool GraphicsController::ballIntersectsLine(Point endPt1, Point endPt2)
{
	// direction vector of line, from to endPt1 to endPt2
	Vector d = endPt2 - endPt1;

	// vector from center of ball to ray endPt1
	Vector f = endPt1 - ballGr->getPosition(); 

	float a = d.dot(d) ;
	float b = 2*f.dot(d) ;
	float c = f.dot(f) - pow(ballGr->getRadius(),2) ;

	float discriminant = b*b - 4*a*c;
	
	if( discriminant < 0 )
	{
		// no intersection
		return false;
	}
	else
	{
	  // there is a solution to the equation.
	  discriminant = sqrt( discriminant );
	  float t1 = (-b + discriminant)/(2*a);
	  float t2 = (-b - discriminant)/(2*a);

	  if( t1 >= 0 && t1 <= 1 )
	  {
		  // solution is on line.
		  return true;
	  }
	  else
	  {
		  // solution "out of range" of line
		  return false;
	  }
	  
	  //use t2 for second point
	  if( t2 >= 0 && t2 <= 1 )
	  {		  
		  // solution is on line.
		  return true;
	  }
	  else
	  {
		  // solution "out of range" of line
		  return false;
	  }
	}
}