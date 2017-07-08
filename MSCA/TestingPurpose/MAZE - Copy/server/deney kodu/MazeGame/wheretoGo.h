/**********************************************************************
Onur Berk Töre
Yeditepe University, RIS Lab

Potantial Field Method for point, in 2D Environment, with wavefront algorithm

**********************************************************************/

#include <iostream>
#include <vector>
#include <fstream>


using namespace std;


class wheretoGo{

public: 

	wheretoGo();


	vector<vector<double> > potantielFieldMatrix; 
	vector<vector<double> > waveFrontMatrix;
	
	vector< pair<int, int>> neighbors;


	void initializeMatrices();
		void addStartPoint(int x,int y);
		void addTargetPoint(int x,int y);
		void addWall();
		void addBarrier(int x,int y);
	
	void AddPotentialField();
	void calculateWavefront();
	void writepotantielFieldMatrixToFile(string filename);
	void writewaveFrontMatrixToFile(string filename);
	void writePathToFile(string filename);
	void writeImprovedPathToFile(string filename);
	void constructScenerio();
	void sumMatrices();
	void calculatePath();
	vector<double> findMinPath(int x, int y);
	int barrier; // wall is a barrier;
	int target;
	int freeCell;

	float repelForce(double x, double y);
	vector<double> atargetsX;
	vector<double> atargetsY;
	vector<double> btargetsX;
	vector<double> btargetsY;
	vector<double> ctargetsX;
	vector<double> ctargetsY;
	vector<double> dtargetsX;
	vector<double> dtargetsY;
	vector<bool> isVisited;
	void pathImprove();
	void findSlope();
	float sign(vector<int> p1, vector<int> p2, vector<int> p3);
	int isInTarget(int i , int j );

	vector<double> barriersX;
	vector<double> barriersY;
	int	minValue(double x, double y);
	double attractiveConstant;
	double repulsiveConstant;
	double distanceOfInfluence;
	double targetRadius;
	void selectPath(int i);
	vector<double> calculateForce(double res_x, double res_y, double res_x_vel, double res_y_vel);


	//----------------------



	
//	vector<double> calculateForce(double x, double y);
	void saveMe();
	void fAtt(double x, double y);
	void toGo();
//	void printMatrix();
//	void YAxisprintMatrix();
//	void printNetForce();

	void clearForces();

	int borderOut();

	
	

	bool isStuck();	


	double fattx;
	double fatty;
	double frepelx;
	double frepely;
	double fnetx;
	double fnety;
	double pointMass;
	double pointRadius;
	double GameTargetRadius;
	double firstTime;
	double targetCounter;
	double currentTarget;
	double matrixHeight;
	double matrixWidth;
	double pointx;
	double pointy;
	double pointvelx;
	double pointvely;
	double targetx;
	double targety;
	double runs;
	double runsss;
	double forceCounter;
	double min_res_x;
	double min_res_z;

	
	/*
	Dýstance  influence imposed  by obstacles; its value depends on the condition of
	the obstacle and the goal point of the robot, and is usually less than half distances
	between the obstacles or shortest length from the destination to the obstacles
	*/


	


};
