/**********************************************************************
Onur Berk Töre
Yeditepe University, RIS Lab

Potantial Field Method for point, in 2D Environment, with wavefront algorithm


sorry for uncommented code please contact me if you need help
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
    void generatePaths();
    void initializeMatrices();
    void addStartPoint(int x,int y);
    void addTargetPoint(int x,int y);
    void AddPotentialField(int i);
    void calculateWavefront(int i );
    void addWall();
    void addBarrier(int x,int y);
    void writepotantielFieldMatrixToFile(string filename);
    void writewaveFrontMatrixToFile(string filename);
    void sumMatrices(int i);
    void calculatePath(int i);
    void pathImprove();
    void findSlope();
    void writePathToFile(string filename);
    void clearMatrix();
    void selectPath(int i);
    void printPath(int i);
    int foundPaths();
    int minValue(double x, double y);
    int isInTarget(int i , int j );
    int getPaths();
    vector< pair<int,int>> neighbors;
    vector<vector<double> > potantielFieldMatrix; 
    vector<vector<double> > waveFrontMatrix;
    float repelForce(double x, double y);
    vector<double> findMinPath(int x, int y);
    vector<double> atargetsX;
    vector<double> atargetsY;
    vector<double> target1X;
    vector<double> target2X;
    vector<double> target3X;
    vector<double> target1Z;
    vector<double> target2Z;
    vector<double> target3Z;
    vector<double> dtargetsX;
    vector<double> dtargetsY;
    string ConvertO(float number);
    double startx;
    double starty;
    double targetx;
    double targety;
    vector<double> barriersX;
    vector<double> barriersY;



    //void writeImprovedPathToFile(string filename);
    //vector<double> btargetsX;
    //vector<double> btargetsY;
    //vector<double> ctargetsX;
    //vector<double> ctargetsY;
    //float sign(vector<int> p1, vector<int> p2, vector<int> p3);

    //int borderOut();
    //double frepelx;
    //double frepely;
    //double targetCounter;


};
