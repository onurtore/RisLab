/**********************************************************************
Onur Berk Töre
Yeditepe University, RIS Lab

Potantial Field Method for point, 2D Environment

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
	void calculatePath();
	int isInTarget();
	void calculateDistance();
	void calculateForce();

//For saving our point from get stuck somewhere
	bool isStuck();
//	void saveMe(int forceConstant);	//Hard To Implement

	void saveMe();


	double minX;
	double maxX;
	double minY;
	double maxY;
	double runs;
//	bool repetitiveStuck;	//Hard To Implement
	// Get bigger if repetitive Stuck happens 
//	bool forceConstant;		//Hard To Implement

//For saving our point from get stuck somewhere

	void fAtt();
	void fRepel();
	
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

	int borderOut();

	void toGo();

	void printMatrix();
	vector<vector<vector<float> > > pathMatrix;

	
	int firstTime; 
	
	int targetRadius;
	int pointRadius;

	vector<int> targetsX;
	vector<int> targetsY;

	vector<int> barriersX;
	vector<int> barriersY;

	int targetCounter;
	int currentTarget;
	
	int matrixHeight;
	int matrixWidth;

	double pointx;
	double pointy;

	double pointvelx;
	double pointvely;

	int targetx; 
	int targety;

	

	ofstream myfile1;
	ofstream myfile2;
	ofstream myfile3;
};
