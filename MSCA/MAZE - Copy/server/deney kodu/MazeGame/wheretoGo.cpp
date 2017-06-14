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


wheretoGo::wheretoGo(){
	
	attractiveConstant	=	0.5f;
	repulsiveConstant	=	2.5f;
	distanceOfInfluence =	15.0f;  
	targetRadius		=	5;
	firstTime			=	1;
	pointMass			=	25;
	currentTarget		=	0;
	targetCounter		=	0;
	matrixHeight		=	82;
	matrixWidth			=	250;
	pointvelx			=	0;
	pointvely			=	0;
	forceCounter		=	1;
	runs				=	0;
	runsss				=	1;
	barrier				=	9999;
	target				=	2;
	freeCell			=	0;
	
	
	constructScenerio();
	
	
}



void wheretoGo::constructScenerio() {
	
	//Neighbors cells in matrix
	neighbors.push_back(make_pair(-1, -1));
	neighbors.push_back(make_pair(-1, 0));
	neighbors.push_back(make_pair(-1, 1));
	neighbors.push_back(make_pair(0, -1));
	neighbors.push_back(make_pair(0, 1));
	neighbors.push_back(make_pair(1, -1));
	neighbors.push_back(make_pair(1, 0));
	neighbors.push_back(make_pair(1, 1));



	initializeMatrices();
	addWall();
	addStartPoint(40, 38);

	//Left-Down
	for (int i = matrixHeight - 27; i < matrixHeight; i++) {
		for (int j = 0; j < 68; j++) {
			addBarrier(i, j);
		}

	}
	//Left-Up
	for (int i = 0; i < 27; i++) {
		for (int j = 0; j < 68; j++) {
			addBarrier(i, j);
		}
	}

	//Right-Center
	for (int i = 27; i < 54; i++) {
		for (int j = 105; j < matrixWidth; j++) {
			addBarrier(i, j);
		}
	}

	addTargetPoint(16, 178);

	
}

void wheretoGo::sumMatrices()
{
	for (int i = 0; i < matrixHeight; i++) {
		for (int j = 0; j < matrixWidth; j++) {
			if (waveFrontMatrix[i][j] != barrier) {
				potantielFieldMatrix[i][j] += waveFrontMatrix[i][j];
			}
		}
	}

	//Min value in the target
	potantielFieldMatrix[targetx][targety] = 0;
	
	
	writepotantielFieldMatrixToFile("0sumMatrices.txt");
}

void wheretoGo::initializeMatrices(){

	for ( int k = 0 ; k < matrixHeight; k++){
		potantielFieldMatrix.push_back( vector<double> () );
		for(int i = 0; i < matrixWidth; i++){
			potantielFieldMatrix[k].push_back(freeCell);
		}
	}


	for (int k = 0; k < matrixHeight; k++) {
		waveFrontMatrix.push_back(vector<double>());
		for (int i = 0; i < matrixWidth; i++) {
			waveFrontMatrix[k].push_back(freeCell);
		}
	}

	
}

void wheretoGo::addWall(){
	 // top wall
    for(int i = 0 ; i < matrixWidth; i++)
    {
    	addBarrier(0,i);
    }
    //left wall
    for(int i = 0 ; i < matrixHeight; i++)
    {
    	addBarrier(i,0);
    }
    //right wall
    for(int i = 0; i < matrixHeight; i++)
    {
    	addBarrier(i,matrixWidth-1);
	}
    //bottom wall 
    for(int i = 0; i < matrixWidth; i++)
    {
    	addBarrier(matrixHeight-1,i);
    }
}


void wheretoGo::addStartPoint(int x, int y){

	pointx = x;
	pointy = y;

}

void wheretoGo::addTargetPoint(int x, int y){

	potantielFieldMatrix[x][y] = target;
	waveFrontMatrix[x][y] = target;

	targetx = x;
	targety = y;
}


void wheretoGo::addBarrier(int x, int y) {

	barriersX.push_back(x);
	barriersY.push_back(y);


	potantielFieldMatrix[x][y] = barrier;
	waveFrontMatrix[x][y] = barrier;
	
	
	/*
	Barrier'i noktaya komþusu olan 8 noktaya ekler  
	for (int k = 0; k < neighbors.size(); k++) {

		//cout << neighbors.at(k).first << " " << neighbors.at(k).second << "\n";

		int i = x + neighbors.at(k).first;

		int j = y + neighbors.at(k).second;

		if ((i >= 0 && i < matrixHeight) && (j >= 0 && j < matrixWidth)) {

			potantielFieldMatrix[i][j].at(0) = barrier;
			potantielFieldMatrix[i][j].at(1) = barrier;
			potantielFieldMatrix[i][j].at(2) = barrier;
			barriersX.push_back(i);
			barriersY.push_back(j);

		}



	}

	*/

}

void wheretoGo::AddPotentialField() {

	for (int i = 0; i < matrixHeight; i++) {
		for (int j = 0; j < matrixWidth; j++) {


			if ((potantielFieldMatrix[i][j] != barrier) && (potantielFieldMatrix[i][j] != target)) {

				potantielFieldMatrix[i][j] = repelForce(i, j);

			}
		}
	}
	writepotantielFieldMatrixToFile("0PotantielField.txt");
}

float wheretoGo::repelForce(double x, double y) {

	float repelForce = 0 ;

	for (unsigned int i = 0; i < barriersX.size(); i++) {
		double euclidean_distance = sqrt(pow(x - barriersX.at(i), 2) + pow(y - barriersY.at(i), 2));

		if (euclidean_distance > 0 && euclidean_distance < distanceOfInfluence) {


			repelForce += repulsiveConstant * ((1 / euclidean_distance) - (1 / distanceOfInfluence));

			/*
			aradaki mesafe cok fazla
			frepelx = exp((-euclidean_distance / repulsiveConstant) * (x - barriersX.at(i)));
			frepely = exp((-euclidean_distance / repulsiveConstant) * (y - barriersY.at(i)));
			aradaki mesafe çok az
			frepelx += (1 / 2)   *  repulsiveConstant * (pow(((1 / euclidean_distance) - (1 / distanceOfInfluence)), 2));
			frepely += (1 / 2)   *  repulsiveConstant * (pow(((1 / euclidean_distance) - (1 / distanceOfInfluence)), 2));
			Aradaki distance çok az
			frepelx +=  repulsiveConstant *  ( (1.0f / euclidean_distance) - (1.0f / distanceOfInfluence) ) *  (1.0f / pow(euclidean_distance, 2) ) * ( (x - barriersX.at(i) ) / euclidean_distance );
			frepely +=  repulsiveConstant *  ( (1.0f / euclidean_distance) - (1.0f / distanceOfInfluence) ) *  (1.0f / pow(euclidean_distance, 2) ) * ( (y - barriersY.at(i) ) / euclidean_distance );
			*/
		}

		else {
			repelForce += 0;
		}




	}
	return repelForce;


}

void wheretoGo::writepotantielFieldMatrixToFile(string filename)
{
	ofstream out;
	out.open(filename);

	for (int i = 0; i < matrixHeight; i++) {
		for (int j = 0; j < matrixWidth; j++) {
			out << potantielFieldMatrix[i][j] << " ";
		}
		out << "\n";
	}
	out.close();
}

void wheretoGo::writewaveFrontMatrixToFile(string filename)
{
	ofstream out;
	out.open(filename);

	for (int i = 0; i < matrixHeight; i++) {
		for (int j = 0; j < matrixWidth; j++) {
			out << waveFrontMatrix[i][j] << " ";
		}
		out << "\n";
	}
	out.close();
}


void wheretoGo::calculateWavefront() {

		//going down in the matrix
		for (int x = targetx; x < matrixHeight; x++) {
			//going right in the matrix
			for (int y = targety; y < matrixWidth; y++) {
				if (waveFrontMatrix[x][y] == freeCell ) {
					waveFrontMatrix[x][y] = minValue(x, y);
					
				}
			}
		
			//going left in the matrix
			for (int y = targety - 1; y >= 0; y--) {
				if (waveFrontMatrix[x][y] == freeCell) {
					waveFrontMatrix[x][y] = minValue(x, y);
					
				}
			}
		}
		
		//going up in the matrix
		for (int x = targetx - 1; x >= 0; x--) {
			//going right in the matrix
			for (int y = targety; y < matrixWidth; y++) {
				if (waveFrontMatrix[x][y] == freeCell) {
					waveFrontMatrix[x][y] = minValue(x, y);
				
					
				}
			}
			//going left in the matrix
			for (int y = targety - 1; y >= 0; y--) {
				if (waveFrontMatrix[x][y] == freeCell) {
					waveFrontMatrix[x][y] = minValue(x, y);
					
				}
			}
		}

		
		//En son üzerinden bir daha geçiyor //En temizi ??
		for (int i = 0; i < matrixHeight; i++) {
			for (int j = 0; j < matrixWidth; j++) {
				if (waveFrontMatrix[i][j] != barrier) {
					waveFrontMatrix[i][j] = minValue(i, j);
				}
			}
		}
	
		waveFrontMatrix[targetx][targety] = 2;
		writewaveFrontMatrixToFile("0WaveFront.txt");
}


int wheretoGo::minValue(double x, double y) {

	double globalMin = barrier;

	for (int k = 0; k < neighbors.size(); k++) {

	
		double i = x + neighbors.at(k).first;

		double j = y + neighbors.at(k).second;

		if ((i >= 0 && i < matrixHeight) && (j >= 0 && j < matrixWidth)) {

			if (waveFrontMatrix[i][j] != barrier && waveFrontMatrix[i][j] != freeCell) {
				double min = waveFrontMatrix[i][j];
				if (min < globalMin) {
					globalMin = min;
				}
			}
		}
	}
	return ++globalMin;
	
	
}

void wheretoGo::selectPath(int i)
{
	if (i == 0) {
		dtargetsX = atargetsX;
		dtargetsY = atargetsY;
	}
	if (i == 1) {
		dtargetsX = btargetsX;
		dtargetsY = btargetsY;
	}
	if (i == 2) {
		dtargetsX = ctargetsX;
		dtargetsY = ctargetsY;
	}
	for (int i = 0; i < dtargetsX.size(); i++) {
		isVisited.push_back(false);
	}

}



void wheretoGo::calculatePath() {


	double i = pointx;
	double j = pointy;

	while( !isInTarget(i,j) ){
			vector<double> xy = findMinPath(i,j);
			i = xy.at(0);
			j = xy.at(1);
			cout << i << "\t" << j << "\n";
			atargetsX.push_back(i);
			atargetsY.push_back(j);
	}


	writePathToFile("0Path.txt");
}


int wheretoGo::isInTarget(int i, int j) {

	if(  (i == targetx) && (j == targety)	)
		return 1;
	else
		return 0;


}



void wheretoGo::writePathToFile(string filename) {

	ofstream out;
	out.open(filename);

	for (int i = 0; i < atargetsX.size(); i++) {
		out << atargetsX.at(i) << "\t" << atargetsY.at(i) << "\n";
	}
	out.close();
}

void wheretoGo::writeImprovedPathToFile(string filename) {

	ofstream out;
	out.open(filename);

	for (int i = 0; i < ctargetsX.size(); i++) {
		out << ctargetsX.at(i) << "\t" << ctargetsY.at(i) << "\n";
	}
	out.close();
}


vector<double> wheretoGo::findMinPath(int x, int y) {

	double  globalMin = barrier;
	vector<double> newxy(2);

	double i,j;

	
	for (int k = 0; k < neighbors.size(); k++) {
		
		i = x + neighbors.at(k).first;
		j = y + neighbors.at(k).second;

		if ((i >= 0 && i < matrixHeight) && (j >= 0 && j < matrixWidth)) {

			if (potantielFieldMatrix[i][j] != barrier) {
				
				double min = potantielFieldMatrix[i][j];
				if (min < globalMin) {
					globalMin = min;
					newxy.at(0) = i;
					newxy.at(1) = j;
				}
			}

		}
			
	}
	/*
	//Always choose the corners, if there is a minima

	i = x + neighbors.at(0).first;
	j = y + neighbors.at(0).second;	
	if ((i >= 0 && i < matrixHeight) && (j >= 0 && j < matrixWidth)) {

		if (potantielFieldMatrix[i][j] != barrier) {
			if (globalMin == potantielFieldMatrix[i][j]) {
				newxy.at(0) = i;
				newxy.at(1) = j;
			}
		}
	}
	//---------------------------------------------------------------------
	i = x + neighbors.at(2).first;
	j = y + neighbors.at(2).second;
	if ((i >= 0 && i < matrixHeight) && (j >= 0 && j < matrixWidth)) {
		if (potantielFieldMatrix[i][j] != barrier) {
			if (globalMin == potantielFieldMatrix[i][j]) {
				newxy.at(0) = i;
				newxy.at(1) = j;
			}
		}
	}
	//---------------------------------------------------------------------
	i = x + neighbors.at(5).first;
	j = y + neighbors.at(5).second;
	if ((i >= 0 && i < matrixHeight) && (j >= 0 && j < matrixWidth)) {

		if (potantielFieldMatrix[i][j] != barrier) {
				if (globalMin == potantielFieldMatrix[i][j]) {
					newxy.at(0) = i;
					newxy.at(1) = j;
				}
		}
	}

	//---------------------------------------------------------------------
	i = x + neighbors.at(7).first;
	j = y + neighbors.at(7).second;
	if ((i >= 0 && i < matrixHeight) && (j >= 0 && j < matrixWidth)) {

		if (potantielFieldMatrix[i][j] != barrier) {
			if (globalMin == potantielFieldMatrix[i][j]) {
				newxy.at(0) = i;
				newxy.at(1) = j;
			}
		}
	}
	*/



	return newxy;
}


vector<double> wheretoGo::calculateForce(double res_x, double res_y, double res_x_vel, double res_y_vel){
	
	double ksc;
	double look_ahead_time = 0.001;
	double look_ahead_pos_x;
	double look_ahead_pos_y;
	double euclidean_distance = 999999999;
	double min_euclidean_distance = 999999999;
	int pos;
	vector<double> force(2);

	look_ahead_pos_x = res_x + look_ahead_time * res_x_vel;
	look_ahead_pos_y = res_y + look_ahead_time * res_y_vel;


	for (int i = 0; i < dtargetsX.size(); i++) {
		euclidean_distance = sqrt((pow(look_ahead_pos_x - dtargetsX.at(i), 2)) + pow(look_ahead_pos_y - dtargetsY.at(i), 2));
	
		if (min_euclidean_distance > euclidean_distance) {
			min_euclidean_distance = euclidean_distance;
			pos = i;
		}

		force.at(0) = ksc * (look_ahead_pos_x - dtargetsX.at(i));
		force.at(1) = ksc * (look_ahead_pos_y - dtargetsY.at(i));
	

	
	}




}



void wheretoGo::pathImprove() {


	findSlope();


	ctargetsX.push_back(btargetsX.at(0));
	ctargetsY.push_back(btargetsY.at(0));



	int i = 0;
	int j = 1;
	int k = 2;

	while (k < btargetsX.size()) {
		vector<int> triangle1;
		vector<int> triangle2;
		vector<int> triangle3;

		triangle1.push_back(btargetsX.at(i));
		triangle1.push_back(btargetsY.at(i));

		triangle2.push_back(btargetsX.at(j));
		triangle2.push_back(btargetsY.at(j));

		triangle3.push_back(btargetsX.at(k));
		triangle3.push_back(btargetsY.at(k));


		bool noBar = true;
		bool b1, b2, b3;
		vector<int> bar;
		for (int a = 0; a < barriersX.size(); a++) {
			if (noBar == true) {
				bar.push_back(barriersX.at(a));
				bar.push_back(barriersY.at(a));

				b1 = sign(bar, triangle1, triangle2) < 0.0f;
				b2 = sign(bar, triangle2, triangle3) < 0.0f;
				b3 = sign(bar, triangle3, triangle1) < 0.0f;


				noBar = !(((b1 == b2) && (b2 == b3)));

				bar.pop_back();
				bar.pop_back();
			}
			else
				continue;
		}
		if (noBar == true) {

			ctargetsX.push_back(btargetsX.at(k));
			ctargetsY.push_back(btargetsY.at(k));


			i = i + 2;
			j = j + 2;
			k = k + 2;


		}
		else {
			ctargetsX.push_back(btargetsX.at(j));
			ctargetsY.push_back(btargetsY.at(j));

			i++;
			j++;
			k++;

		}


		triangle1.pop_back();
		triangle1.pop_back();

		triangle2.pop_back();
		triangle2.pop_back();

		triangle3.pop_back();
		triangle3.pop_back();
	}

	//New target Counter
	targetCounter = ctargetsX.size();
	
	//---------
	ctargetsX.push_back(atargetsX.at(atargetsX.size()-1));
	ctargetsY.push_back(atargetsY.at(atargetsY.size()-1));

	writeImprovedPathToFile("0ImprovedPath.txt");


}

float wheretoGo::sign(vector<int> p1, vector<int> p2, vector<int> p3) {

	return (p1.at(0) - p3.at(0)) * (p2.at(1) - p3.at(1)) - (p2.at(0) - p3.at(0)) * (p1.at(1) - p3.at(1));
}


void wheretoGo::findSlope() {

	btargetsX.push_back(atargetsX.at(0));
	btargetsY.push_back(atargetsY.at(0));


	int slopeX = atargetsX.at(1) - atargetsX.at(0);
	int slopeY = atargetsY.at(1) - atargetsY.at(0);

	for (int i = 2; i < atargetsX.size(); i++) {
		int newSlopeX = atargetsX.at(i) - atargetsX.at(i - 1);
		int newSlopeY = atargetsY.at(i) - atargetsY.at(i - 1);

		if (slopeX == newSlopeX && slopeY == newSlopeY) {
			continue;
		}
		else {
			slopeX = newSlopeX;
			slopeY = newSlopeY;
			btargetsX.push_back(atargetsX.at(i - 1));
			btargetsY.push_back(atargetsY.at(i - 1));
		}
	}
	btargetsX.push_back(atargetsX.at(atargetsX.size() - 1));
	btargetsY.push_back(atargetsY.at(atargetsY.size() - 1));


}



























/*

std::vector<double> wheretoGo::calculatePath(double x, double z){

	pointx = x;
	pointy = z;

	//Empty force vector 
	vector<double> noForce(2);
	noForce.at(0)  = 0 ;
	noForce.at(1)  = 0 ;


	/*if(firstTime){
		targetx = ctargetsX.at(currentTarget);
		targety = ctargetsY.at(currentTarget);


		//Add potantiel field algorithm to the matrix
		AddPotantielField();
		cout << "Potantiel Field Added\n\n\n";

		firstTime = false;

	}

	//cout << "TargetCounter is" << targetCounter <<  "\n";
		
	//cout << "targetx is: " << targetx << " targety is: " << targety << "\n";
	*/

	//isInTarget();
		
	/*
	
	/*Karistirma simdi bunu
	if (borderOut() == -1) {
		cout << "Out of the border";
		return noForce;
	}
	*/
	/*if (runsss  >= 5000  ){
	//	return calculateForce(x, z);

	}
	else{
		//Stop algorithm for first start;
		this->runsss++;
		return noForce;
	}

	myfile2 << pointx << " " << pointy << "\n";

}*/


int wheretoGo::borderOut() {

	if (  (abs(pointx) > matrixHeight) || (abs(pointy) > matrixWidth) )
	{

		return -1;

	}
	return 1;

}


/*
int wheretoGo::isInTarget(){

	

	if ( (pointx > targetx - targetRadius  && pointx < targetx+ targetRadius ) && (pointy > targety - targetRadius  && pointy < targety + targetRadius ) ){
		cout << "I am in the target\n\n\n";
		currentTarget++;
		cout << "CURRENT TARGET NUMBER: " << currentTarget << "\n\n\n";
		//Also can be change with 
		//if(currentTarget >= targetCounter)
		if(currentTarget >= ctargetsX.size() ){
			//cout << "currentTarget is : " << currentTarget << "\n";

			return 100;
		}
		
		else{
		targetx = ctargetsX.at(currentTarget);
		targety = ctargetsY.at(currentTarget);
		//Reconstruct potantielField matrix new targets
		AddPotantielField();
		cout << "Potantiel Field Added\n\n\n";
		}



	}
	else
		return 1;
}
*/

/**
std::vector<double> wheretoGo::calculateForce(double  x, double  y) {
	
	std::vector<double> returnValue(2);
	int newx = x;
	int newy = y;
	int counter = 0;
	int i;
	int j;
	for (int k = 0; k < neighbors.size(); k++) {


		i = newx + neighbors.at(k).first;

		j = newy + neighbors.at(k).second;

		if ((i >= 0 && i < matrixHeight) && (j >= 0 && j < matrixWidth)) {

			if (potantielFieldMatrix[i][j].at(0) != 0 && potantielFieldMatrix[i][j].at(0) != 1) {
				counter++;
				returnValue.at(0) += potantielFieldMatrix[i][j].at(0);
				returnValue.at(1) += potantielFieldMatrix[i][j].at(1); 
				}
			}
		}
	
	if (counter != 0) {
		returnValue.at(0) = returnValue.at(0) / counter;
		returnValue.at(1) = returnValue.at(1) / counter;
	}
	else {
		returnValue.at(0) = 0;
		returnValue.at(1) = 0;
	}

	return returnValue;

	/*

	This is from the old game No neccessarry to keep it in here.

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
	std::vector<float> returnValue(2);

	returnValue.at(0) = fnetx;
	returnValue.at(1) = fnety;
	return returnValue; 
	*//*
}*/
/*
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
/*
//	pointx += fnetx * 0.01f;
//	pointy += fnety * 0.01f;


}









void wheretoGo::clearForces() {

	fattx = 0;
	fatty = 0;

	frepelx;
	frepely;

	fnetx = 0;
	fnety = 0;
}



/*
void wheretoGo::AddPotantielField(){

	
	for ( int i = 0 ; i < matrixHeight; i++){
		for (int j = 0; j < matrixWidth; j++ ) {
			
			fattx = 0;
			fatty = 0;

			frepelx = 0;
			frepely = 0;

			fnetx = 0;
			fnety = 0;

			//Barrier ya da target noktasý ise force koyma oraya 
			if (!	(potantielFieldMatrix[i][j].at(0) == 1 && potantielFieldMatrix[i][j].at(1) == 1) || (potantielFieldMatrix[i][j].at(0) == 2)) { 
				//fAtt(i, j);
				fRepel(i, j);
			}

			myfile9  << fattx << " ";
			myfile10 << fatty << " ";
			myfile11 << frepelx << " ";
			myfile12 << frepely << " ";
	
			fnetx =  (fattx + frepelx ) ;
			fnety =  (fatty + frepely ) ;

			
			potantielFieldMatrix[i][j].at(0) = fnetx;
			potantielFieldMatrix[i][j].at(1) = fnety;

		}

		myfile9  << "\n";
		myfile10 << "\n";
		myfile11 << "\n";
		myfile12 << "\n";

		

	}
	
	printMatrix(); // print potantielFieldMatrix[i][j].at(0) which is net force in x axis // 0matrix0
	YAxisprintMatrix(); // print potantielFieldMatrix.at(1)  which is net force in y axis // 0matrix1
	printNetForce(); // print  sqrt(x2 + y2)  // 0matrix2
	return;

	

}*/

/*void wheretoGo::fAtt(double x, double y) {

	fattx = -1.0f * attractiveConstant * (x - targetx);
	fatty = -1.0f * attractiveConstant * (y - targety);
}
*/







/*

void wheretoGo::printMatrix(){
    


	for ( int i = 0 ; i < matrixHeight ; i++){
        for(  int j = 0; j < matrixWidth  ; j++)
        {
		//	cout <<  " " << potantielFieldMatrix[i][j].at(0) <<" ";
			myfile1 <<   potantielFieldMatrix[i][j].at(0) << " ";
        }
		myfile1 << "\n";
		//cout << "\n";
    }
	myfile1.close(); 

}


void wheretoGo::YAxisprintMatrix(){
    


	for ( int i = 0 ; i < matrixHeight ; i++){
        for(  int j = 0; j < matrixWidth  ; j++)
        {
		//	cout <<  " " << potantielFieldMatrix[i][j].at(0) <<" ";
			myfile7 <<  potantielFieldMatrix[i][j].at(1) << " ";
        }
		myfile7 << "\n";
		//cout << "\n";
    }
	
	myfile7.close();
}

void wheretoGo::printNetForce()
{
	double fnetForce;

	for (int i = 0; i < matrixHeight; i++) {
		for (int j = 0; j < matrixWidth; j++) {
			fnetForce = sqrt(pow(potantielFieldMatrix[i][j].at(0), 2) + pow(potantielFieldMatrix[i][j].at(1), 2));
			potantielFieldMatrix[i][j].at(2) = fnetForce;
			myfile8 << fnetForce << " ";
		}
		myfile8 << "\n";
	}

}*/