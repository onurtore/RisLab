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

	~wheretoGo();

	void initializeMatrix();
	
	void addStartPoint(int x,int y);
	void addTargetPoint(int x,int y);
	void add4Wall();
	void addBarrier(int x,int y);
	
	std::vector<double> calculatePath(float x, float y);
	
	int isInTarget();
	
	std::vector<double> calculateForce(float x, float y);
	
	void calculateWavefront();
	
	void calculateWavefrontPath();
	
	vector<double> findMinPath(int x,int y);
	
	int	minValue(double x, double y);
	
	void pathImprove();
	
	void findSlope();
	
	float sign(vector<int> p1, vector<int> p2, vector<int> p3);
	
	void AddPotantielField();

	bool isStuck();
	
	void saveMe();
  
	
	void fRepel(double x, double y);
	void fAtt(double x, double y);


	double fattx;
	double fatty;

	double frepelx;
	double frepely;
	
	double fnetx;
	double fnety;

	double pointMass;

	double attractiveConstant;
	double repulsiveConstant;
	
	double distanceOfInfluence;
	/*
	Dýstance  influence imposed  by obstacles; its value depends on the condition of
	the obstacle and the goal point of the robot, and is usually less than half distances
	between the obstacles or shortest length from the destination to the obstacles
	*/

	int borderOut();

	void toGo();

	void printMatrix();
	void ZAxisprintMatrix();

	vector<vector<vector<double> > > pathMatrix;

	
	double firstTime; 
	
	double targetRadius;
	double pointRadius;
	double GameTargetRadius;
	vector<double> targetsX;
	vector<double> targetsY;

	vector<double> improvedsX;
	vector<double> improvedsY;

	vector<double> lastX;
	vector<double> lastY;

	vector<double> barriersX;
	vector<double> barriersY;

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

	double runsss ;

	ofstream myfile1;
	ofstream myfile2;
	ofstream myfile3;
	ofstream myfile4;
	ofstream myfile5;
	ofstream myfile6;
	ofstream myfile7;

	vector< pair<int, int>> neighborhood;




};
