/**********************************************************************
Onur Berk Töre
Yeditepe University, RIS Lab

Potantial Field Method for point, 2D Environment

**********************************************************************/

#include "wheretoGo.h"
#include <stdlib.h>
#include <fstream>
#include <stdio.h>
#include <math.h>
/*
-100 wall,repel point;
-200 point;
-300 target;
*/

wheretoGo::wheretoGo(){
	 myfile1.open("matrix.txt");
	 myfile2.open("coordinates.txt");
	 myfile3.open("forces.txt");

	attractiveConstant = 1.0f;
	repulsiveConstant = 8000.0f;
	distanceOfInfluence = 30000.0f;
	
	matrixHeight = 100;
	matrixWidth = 100;

	targetRadius = 10;
	 
	firstTime = 1;

	pointMass = 25;

	currentTarget = 0;
	targetCounter = 0;

	initializeMatrix();

	//add4Wall();
	
	pointvelx = 0 ;
	pointvely = 0 ;

	addStartPoint(5,5);
	
	//addTargetPoint(25,35);
	//addTargetPoint(70, 50);
	//addTargetPoint(30,40);
	addTargetPoint(90,90);


	//add4Wall();
	
	addBarrier(25,25);
	//addBarrier(60, 52);
	//addBarrier(23,30);
	//addBarrier(40,20);

	//printMatrix();

	//For saving our point from get stuck somewhere
	minX = 0;
	maxX = 0;
	minY = 0;
	maxY = 0;
	runs = 0 ;
//	repetitiveStuck = false;
	// Get bigger if repetitive Stuck happens 

}

wheretoGo::~wheretoGo(){
	myfile2.close();
	myfile3.close();
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
        pathMatrix[0][i].at(0) =  -100; //wall
    }
    //left wall
    for(int i = 0 ; i < matrixHeight; i++)
    {
        pathMatrix[i][0].at(0) = -100;  
    }
    //right wall
    for(int i = 0; i < matrixHeight; i++)
    {
        pathMatrix[i][matrixWidth-1].at(0) = -100;
    }
    //bottom wall 
    for(int i = 0; i < matrixWidth; i++)
    {
        pathMatrix[matrixHeight-1][i].at(0) = -100;
    }
}


void wheretoGo::addStartPoint(int x, int y){

	pointx = x;
	pointy = y;

	pathMatrix[x][y].at(0) = -200;

}

void wheretoGo::addTargetPoint(int x, int y){


	pathMatrix[x][y].at(0) = -300;
	
	targetsX.push_back(x); 
	targetsY.push_back(y);
	
	targetCounter++;

}


void wheretoGo::addBarrier(int x, int y){
	
	barriersX.push_back(x);
	barriersY.push_back(y);
	pathMatrix[x][y].at(0) = -100;

}

/**/
void wheretoGo::calculatePath(){


	if(firstTime){
		targetx = targetsX.at(0);
		targety = targetsY.at(0);

		calculateDistance();
	}

	while(1){
		cout << "hello\n";
		if ( 10 == isInTarget() ){
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
			return 10;
		}
		
		else{
		targetx = targetsX.at(currentTarget);
		targety = targetsY.at(currentTarget);
		calculateDistance();
		}



	}
	else
		return 1;
}

void wheretoGo::calculateDistance(){

	for(int i = 0 ; i < matrixWidth ; i++){
		for(int j = 0 ; j < matrixHeight  ; j++ ){
		
			pathMatrix[i][j].at(1) = abs( targetx - i ) ;
			pathMatrix[i][j].at(2) = abs(targety - j ) ;
		}
	}

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
 	myfile3 << "fattx is: " << fattx << "\t" << "fatty is: " << fatty << "\n";
	myfile3 << "frepelx is: " << frepelx << "\t" << "frepely is: " << frepely << "\n";

	fnetx =  (fattx + frepelx ) ;
	fnety =  (fatty + frepely ) ;

	myfile3 << "fnetx is: " << fnetx << "\t" << "fnety is: " << fnety << "\n\n" ;
	myfile3 << "\n";
	
	toGo();

	if (isStuck() == true) {
		cout << "I'm stuck\n";
		saveMe();
	}

	


}
/*
Not very complex maybe better implementation
*/
bool wheretoGo::isStuck(){

	if (fnetx < 0.009 && fnety < 0.009) {
		if (runs > 10) {
			return true;
		}
		else {
			runs++;
			return false;
		}
	}

	if ((pointx > minX && pointx < maxX) && (pointy > minY && pointy < maxY) ) {
		if (runs > 10) {
			return true;
		}
		else {
			runs++;
			return false;
		}
	}
	return false;


}
/*

void wheretoGo::saveMe(int forceConstant){
	
	minX = 0;
	maxX = 0;
	minY = 0;
	maxY = 0;
	runs = 0;
}
*/
/*
Not a  good implementation use different implementation
/planning.cs.uiuc.edu/node225.html
*/
void wheretoGo::saveMe() {
	
	minX = 0;
	maxX = 0;
	minY = 0;
	maxY = 0;
	runs = 0;


	//Apply random force to point//
	
	if ( (pointx == maxX && pointy == maxY)  || ( ( pointx > minX  && pointx < maxX )  || ( pointy > minY && pointy < maxY) ) ) {
		pointx += fnetx;
		
	}
	else {
		pointx += -fnetx;
		pointy += -fnety;
	}
		


}


void wheretoGo::fAtt() {

	fattx = -1 * attractiveConstant * (pointx - targetx);
	fatty = -1 * attractiveConstant * (pointy - targety);
}



void wheretoGo::fRepel() {

	for (unsigned int i = 0; i < barriersX.size(); i++) {
		double euclidean_distance = sqrt(pow(pointx - barriersX.at(i), 2) + pow(pointy - barriersY.at(i), 2));

		if (euclidean_distance > 0 && euclidean_distance < distanceOfInfluence) {
			frepelx += repulsiveConstant *  ( (1 / euclidean_distance) - (1 / distanceOfInfluence) ) *  (1 / pow(euclidean_distance, 2) ) * ( (pointx - barriersX.at(i) ) / euclidean_distance );
			frepely += repulsiveConstant *  ( (1 / euclidean_distance) - (1 / distanceOfInfluence) ) *  (1 / pow(euclidean_distance, 2) ) * ( (pointy - barriersY.at(i) ) / euclidean_distance );
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

	pointx += fnetx * 0.01;
	pointy += fnety * 0.01;


	if (pointx > maxX) { maxX = pointx; }
	if (pointx < minX) { minX = pointx; }
	if (pointy > maxY) { maxY = pointy; }
	if (pointy < minY) { minY = pointy; }
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
	myfile1.close();

}