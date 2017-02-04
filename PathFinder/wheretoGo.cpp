/**********************************************************************
Onur Berk Töre
Yeditepe University, RIS Lab

Potantial Field Method for point, in 2D Environment, with wavefront algorithm

**********************************************************************/

#include "wheretoGo.h"
#include <stdlib.h>
#include <fstream>
#include <stdio.h>
#include <math.h>

/*
pathMatrix[i][matrixWidth-1].at(0) = :

		1 =	Barrier : wall,obstacle
		2 =	Target 
		0 = empty
*/

wheretoGo::wheretoGo(){
	

	neighborhood.push_back(make_pair(-1, -1));
	neighborhood.push_back(make_pair(-1, 0));
	neighborhood.push_back(make_pair(-1, 1));
	neighborhood.push_back(make_pair(0, -1));
	neighborhood.push_back(make_pair(0, 1));
	neighborhood.push_back(make_pair(1, -1));
	neighborhood.push_back(make_pair(1, 0));
	neighborhood.push_back(make_pair(1, 1));


	myfile1.open("matrix.txt");
	myfile2.open("coordinates.txt");
	myfile3.open("forces.txt");
	myfile4.open("wavefront.txt");
	
	attractiveConstant = 100.0f;
	repulsiveConstant = 75.0f;
	distanceOfInfluence = 300.0f;
	
	matrixHeight = 100;
	matrixWidth = 100;

	targetRadius = 1.5;
	 
	firstTime = 1;

	pointMass = 25;

	currentTarget = 0;
	targetCounter = 0;

	initializeMatrix();

	//add4Wall();
	
	pointvelx = 0 ;
	pointvely = 0 ;

	
	addStartPoint(5, 5);
	
	
	addBarrier(15, 25);
	addBarrier(16, 26);
	addBarrier(17, 27);
	addBarrier(18, 28);
	addBarrier(19, 29);

	

	addBarrier(20, 30);
	addBarrier(21, 29);
	addBarrier(22, 28);
	addBarrier(23, 27);
	addBarrier(24, 26);
	addBarrier(25, 25);
	addBarrier(26, 24);
	addBarrier(27, 23);
	addBarrier(28, 22);
	addBarrier(29, 21);
	addBarrier(30, 20);

	addBarrier(29, 19);
	addBarrier(28, 18);
	addBarrier(27, 17);
	addBarrier(26, 16);
	addBarrier(25, 15);




	
	
	addTargetPoint(90,90);

	//add4Wall();

	runs = 0;
}

wheretoGo::~wheretoGo(){
	
	myfile1.close();
	myfile2.close();
	myfile3.close();
	myfile4.close();
}




void wheretoGo::initializeMatrix(){

	vector<float> matrix1D;
	vector<vector<float> > matrix2D;

	for ( int k = 0 ; k < matrixHeight; k++){
		pathMatrix.push_back( vector<vector<float> >() );
		for(int i = 0; i < matrixWidth; i++){
			pathMatrix[k].push_back(vector<float> () );
			for(int j = 0; j < 3; j++){
				pathMatrix[k][i].push_back(0);
			}
		}
	}

}

void wheretoGo::add4Wall(){
	   // top wall
    for(int i = 0 ; i < matrixWidth; i++)
    {
        pathMatrix[0][i].at(0) =  1;
    }
    //left wall
    for(int i = 0 ; i < matrixHeight; i++)
    {
        pathMatrix[i][0].at(0) = 1;  
    }
    //right wall
    for(int i = 0; i < matrixHeight; i++)
    {
        pathMatrix[i][matrixWidth-1].at(0) = 1;
    }
    //bottom wall 
    for(int i = 0; i < matrixWidth; i++)
    {
        pathMatrix[matrixHeight-1][i].at(0) = 1;
    }
}


void wheretoGo::addStartPoint(int x, int y){

	pointx = x;
	pointy = y;

	//pathMatrix[x][y].at(0) = -99;

}

void wheretoGo::addTargetPoint(int x, int y){


	pathMatrix[x][y].at(0) = 2;
	


	targetx = x;
	targety = y;

	

}


void wheretoGo::addBarrier(int x, int y) {

	/** Onceki hali
	barriersX.push_back(x);
	barriersY.push_back(y);


	pathMatrix[x][y].at(0) = 1;
	*/


	//4 lü ekle
	for (int k = 0; k < neighborhood.size(); k++) {

		//cout << neighborhood.at(k).first << " " << neighborhood.at(k).second << "\n";

		int i = x + neighborhood.at(k).first;

		int j = y + neighborhood.at(k).second;

		if ((i >= 0 && i < matrixHeight) && (j >= 0 && j < matrixWidth)) {

			pathMatrix[i][j].at(0) = 1;
			barriersX.push_back(i);
			barriersY.push_back(j);

		}



	}

}





/**/
void wheretoGo::calculatePath(){


	if(firstTime){
		targetx = targetsX.at(currentTarget);
		targety = targetsY.at(currentTarget);

		firstTime = false;

	}

	cout << "TargetCounter is" << targetCounter <<  "\n";
	while(1){
		cout << "targetx is: " << targetx << " targety is: " << targety << "\n";
		
		if ( 100 == isInTarget() ){
			cout << "Game is finished\n";
			return;
		}
		if (borderOut() == -1) {
			cout << "Out of the border";
			return;
		}
		calculateForce();
		
		myfile2 << pointx << " " << pointy << "\n";

		

	}
}
int wheretoGo::borderOut() {

	if (  (abs(pointx) > matrixHeight) || (abs(pointy) > matrixWidth) )
	{

		return -1;

	}
	return 1;

}


int wheretoGo::isInTarget(){

	

	if ( (pointx > targetx - targetRadius  && pointx < targetx+ targetRadius ) && (pointy > targety - targetRadius  && pointy < targety + targetRadius ) ){
		
		currentTarget++;
		
		if(currentTarget >= targetCounter ){
			//cout << "currentTarget is : " << currentTarget << "\n";

			return 100;
		}
		
		else{
		targetx = targetsX.at(currentTarget);
		targety = targetsY.at(currentTarget);
		
		}



	}
	else
		return 1;
}



void wheretoGo::calculateForce() {

	fattx = 0.0f;
	fatty = 0.0f;

	frepelx = 0.0f;
	frepely = 0.0f;
	
	fnetx = 0.0f;
	fnety = 0.0f;

	fAtt();
	fRepel();



	myfile3 << "pointx is " << pointx << "\t" << "pointy is: " << pointy << "\n";
	myfile3 << "target is" << targetx << "\t" << "targety is" << targety << "\n";
 	myfile3 << "fattx is: " << fattx << "\t" << "fatty is: " << fatty << "\n";
	myfile3 << "frepelx is: " << frepelx << "\t" << "frepely is: " << frepely << "\n";

	fnetx =  (fattx + frepelx ) ;
	fnety =  (fatty + frepely ) ;

	myfile3 << "fnetx is: " << fnetx << "\t" << "fnety is: " << fnety << "\n\n" ;
	myfile3 << "\n";
	
	toGo();

	if (isStuck() == true) {
		cout << "I'm stuck\n";
		//saveMe();
	}
	
}

bool wheretoGo::isStuck(){

	if (fnetx < 0.0009 && fnety < 0.0009 && isInTarget() ) {
		//Local Minima Occurs
		if (runs > 10) {
			return true;
		}
		else {
			runs++;
			return false;
		}
	}

	

}


void wheretoGo::saveMe(){
	runs = 0;
}



void wheretoGo::fAtt() {

	fattx = -1.0f * attractiveConstant * (pointx - targetx);
	fatty = -1.0f * attractiveConstant * (pointy - targety);
}



void wheretoGo::fRepel() {

	for (unsigned int i = 0; i < barriersX.size(); i++) {
		double euclidean_distance = sqrt(pow(pointx - barriersX.at(i), 2) + pow(pointy - barriersY.at(i), 2));

		if (euclidean_distance > 0 && euclidean_distance < distanceOfInfluence) {
			frepelx += repulsiveConstant *  ( (1.0f / euclidean_distance) - (1.0f / distanceOfInfluence) ) *  (1.0f / pow(euclidean_distance, 2) ) * ( (pointx - barriersX.at(i) ) / euclidean_distance );
			frepely += repulsiveConstant *  ( (1.0f / euclidean_distance) - (1.0f / distanceOfInfluence) ) *  (1.0f / pow(euclidean_distance, 2) ) * ( (pointy - barriersY.at(i) ) / euclidean_distance );
		}

		else {
			frepelx += 0;
			frepely += 0;
		}

	}
	

}



void wheretoGo::toGo() {

/* Not working ~ Why Look 
	// Semi-implicit Euler Method
	//0.001 = time-step
	

	//The position update in the second step uses the next velocity and the implicit method
	//Usually not as accurate as RK4 beause order is still o(time-step) but cheaper and similar stability
	//Ver popular choice for game physics engine


	pointvelx += (fnetx / pointMass) * 0.001;
	pointvely += (fnety / pointMass) * 0.001;

	pointx += pointvelx * 0.001;
	pointy += pointvely * 0.001;
*/

	pointx += fnetx * 0.01f;
	pointy += fnety * 0.01f;


}

void wheretoGo::calculateWavefront() {


	
	//going down in the matrix
	for (int x = targetx; x < matrixHeight; x++) {
		//going right in the matrix
		for (int y = targety; y < matrixWidth; y++) {
			//cout << y << "\n"; // just for test purposes
			if (pathMatrix[x][y].at(0) == 0) {
				pathMatrix[x][y].at(0) = minValue(x, y);
				
			}
		}
		//going left in the matrix
		for (int y = targety-1; y >= 0; y--) {
			if (pathMatrix[x][y].at(0) == 0) {
				pathMatrix[x][y].at(0) = minValue(x, y);
			}
		}
	}

	//going up in the matrix
	for (int x = targetx-1; x >= 0; x--) {
		//going right in the matrix
		for (int y = targety; y < matrixWidth; y++) {
			if (pathMatrix[x][y].at(0) == 0) {
				pathMatrix[x][y].at(0) = minValue(x, y);
			}
		}
		//going left in the matrix
		for (int y = targety-1; y >= 0; y--) {
			if (pathMatrix[x][y].at(0) == 0) {
				pathMatrix[x][y].at(0) = minValue(x, y);
			}
		}
	}

/*
	for (int i = 0; i < matrixHeight; i++) {
		for (int j = 0; j < matrixWidth; j++) {
			if (pathMatrix[i][j].at(0) != 1){
				pathMatrix[i][j].at(0) = minValue(i, j);
			}
		}
	}
*/
	calculateWavefrontPath();
	




}

void wheretoGo::calculateWavefrontPath() {
	
	
	
	double i = pointx;
	double j = pointy;
	

	while ( i != targetx  && j !=  targety) {
		
		vector<double> xy = findMinPath(i,j);
		i = xy.at(0);
		j = xy.at(1);

			targetsX.push_back(i);
			targetsY.push_back(j);
		
		
	}
	
	
	for (int i = 0; i < targetsY.size(); i++) {
			myfile4 << targetsX.at(i) << " " << targetsY.at(i) << "\n";
	}
	myfile4.close();
	targetCounter = targetsX.size();
	
}
//For find the minimum effor path
vector<double> wheretoGo::findMinPath(int x, int y) {
	
	int globalMin = 99;
	vector<double> newxy(2);
	int i;
	int j;
	for (int k = 0; k < neighborhood.size(); k++) {


		 i = x + neighborhood.at(k).first;

		 j = y + neighborhood.at(k).second;

		if ((i >= 0 && i < matrixHeight) && (j >= 0 && j < matrixWidth)) {

			if (pathMatrix[i][j].at(0) != 0 && pathMatrix[i][j].at(0) != 1) {
				int min = pathMatrix[i][j].at(0);
				if (min < globalMin) {
					globalMin = min;
					newxy.at(0) = i;
					newxy.at(1) = j;
				}
			}

		}
	}
		
	//Always choose the corners, if there is a minima

		 i = x + neighborhood.at(0).first;
		 j = y + neighborhood.at(0).second;
		if (pathMatrix[i][j].at(0) != 0 && pathMatrix[i][j].at(0) != 1) {
			if (globalMin == pathMatrix[i][j].at(0)) {
				newxy.at(0) = i;
				newxy.at(1) = j;
			}
		}

		 i = x + neighborhood.at(2).first;
		 j = y + neighborhood.at(2).second;
		if (pathMatrix[i][j].at(0) != 0 && pathMatrix[i][j].at(0) != 1) {
			if (globalMin == pathMatrix[i][j].at(0)) {
				newxy.at(0) = i;
				newxy.at(1) = j;
			}
		}

		 i = x + neighborhood.at(5).first;
		 j = y + neighborhood.at(5).second;
		if (pathMatrix[i][j].at(0) != 0 && pathMatrix[i][j].at(0) != 1) {
			if (globalMin == pathMatrix[i][j].at(0)) {
				newxy.at(0) = i;
				newxy.at(1) = j;
			}
		}

		 i = x + neighborhood.at(7).first;
		 j = y + neighborhood.at(7).second;
		if (pathMatrix[i][j].at(0) != 0 && pathMatrix[i][j].at(0) != 1) {
			if (globalMin == pathMatrix[i][j].at(0)) {
				newxy.at(0) = i;
				newxy.at(1) = j;
			}
		}

	


	
	return newxy;
}


void wheretoGo::printMatrix(){
    


	for ( int i = 0 ; i < matrixHeight ; i++){
        for(  int j = 0; j < matrixWidth  ; j++)
        {
		//	cout <<  " " << pathMatrix[i][j].at(0) <<" ";
			myfile1 <<  " "  <<  int(pathMatrix[i][j].at(0));
        }
		myfile1 << "\n";
		//cout << "\n";
    }
	

}
//For fill matrix
int wheretoGo::minValue(double x, double y) {

	int globalMin = 99;
	
	for (int k = 0; k < neighborhood.size(); k++) {
		
		//cout << neighborhood.at(k).first << " " << neighborhood.at(k).second << "\n";

		int i = x + neighborhood.at(k).first;
		
		int j = y + neighborhood.at(k).second;
		
		if ( (i >= 0 && i < matrixHeight) && (j >= 0 && j < matrixWidth) ) {

			if (pathMatrix[i][j].at(0) != 0 && pathMatrix[i][j].at(0) != 1) {
				int min = pathMatrix[i][j].at(0);
				if (min < globalMin) {
					globalMin = min;
				}
			}

			
		}
			
		
	}
	
	return ++globalMin;
}


