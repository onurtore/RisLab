/**********************************************************************
Onur Berk Töre
Yeditepe University, RIS Lab

Potantial Field Method for point, in 2D Environment, with wavefront algorithm

Sorry for the people seen all the white space...
Also sorry for uncommented code please contact me if you need help
onurberk_t@hotmail.com
**********************************************************************/

#include <iostream>
#include <vector>
#include <fstream>

using namespace std;


class wheretoGo{

public:

    wheretoGo();

    double repulsiveConstant;
    double distanceOfInfluence;
    int targetCounter;
    int matrixHeight;
    int matrixWidth;
    int barrier;
    int target;
    int freeCell;

    int foundPaths();

    void generatePaths();
    void initializeMatrices();
    void addStartPoint(int x,int y);
    void addTargetPoint(int x,int y);

    vector< pair<int,int>> neighbors;

    vector<vector<double> > potantielFieldMatrix; 
    vector<vector<double> > waveFrontMatrix;
    
    void AddPotentialField(int i);
    void calculateWavefront(int i );


    void addWall();
    void addBarrier(int x,int y);


    float repelForce(double x, double y);
    
    void writepotantielFieldMatrixToFile(string filename);
    int	minValue(double x, double y);
    void sumMatrices(int i);
    
    void calculatePath(int i);
    int isInTarget(int i , int j );
    
    vector<double> findMinPath(int x, int y);
    
    vector<double> atargetsX;
	vector<double> atargetsY;
    
    void pathImprove();
	void findSlope();
    
    
    
    
    
    int getPaths();
    void clearMatrix();
	void writewaveFrontMatrixToFile(string filename);
	void writePathToFile(string filename);
	void writeImprovedPathToFile(string filename);
	void constructScenerio();
	vector<double> btargetsX;
	vector<double> btargetsY;
	vector<double> ctargetsX;
	vector<double> ctargetsY;
	vector<double> dtargetsX;
	vector<double> dtargetsY;
	vector<double> target1X;
	vector<double> target2X;
	vector<double> target3X;
	vector<double> target1Z;
	vector<double> target2Z;
	vector<double> target3Z;
	vector<bool> isVisited;
	float sign(vector<int> p1, vector<int> p2, vector<int> p3);
	vector<double> barriersX;
	vector<double> barriersY;
	double attractiveConstant;
	double repulsiveConstant;
	double distanceOfInfluence;
	double targetRadius;
	void selectPath(int i);
	void saveMe();
	void fAtt(double x, double y);
	void toGo();
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
};
