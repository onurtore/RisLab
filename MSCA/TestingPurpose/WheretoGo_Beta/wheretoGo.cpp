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
#include <iostream>
#include <string>

using namespace std;


wheretoGo::wheretoGo(){

    repulsiveConstant	=	2.5f;
    distanceOfInfluence =	15.0f;
    targetCounter		=	0;
    matrixHeight		=	82;
    matrixWidth			=	200;
    barrier				=	9999;
    target				=	2;
    freeCell			=	0;





}


int wheretoGo::foundPaths(){

    string file_string1 = "Onur76_28.txt";
    string file_string2 = "Onur93_0.txt";
    
    ofstream myfile1;
    ofstream myfile2;

    
    myfile1.open(file_string1);
    myfile2.open(file_string2);
    
    if((!myfile.is_open()) && (!myfile2.is_open()) && (!myfile3.is_open())) {
    
        return 1;
    
    }
    
    
    else{
        cout << "Paths have been found - Lets Take Them.\n";
    }
    string line;
    bool read_y = false;
    string x_coordinate,y_coordinate;
    
    while(getline(myfile1,line)){
          x_coordinate.clear();
          y_coordinate.clear();
          read_y = false;
          for(int i = 0 ; i < line.size(); i++ ){
          
                if(line.at(i) == ' '){
                    read_y = true;
                    continue;
                }
                
                switch(read_y){
                    case false :
                        x_coordinate.push_back(line.at(i));
                        break;
                    case true : 
                        y_coordinate.push_back(line.at(i));
                        break;
                }
          }
          target1X.push_back(atoi(x_coordinate.c_str()));
          target1Z.push_back(atoi(y_coordinate.c_str()));
    }
        while(getline(myfile2,line)){
          x_coordinate.clear();
          y_coordinate.clear();
          read_y = false;
          for(int i = 0 ; i < line.size(); i++ ){
          
                if(line.at(i) == ' '){
                    read_y = true;
                    continue;
                }
                
                switch(read_y){
                    case false :
                        x_coordinate.push_back(line.at(i));
                        break;
                    case true : 
                        y_coordinate.push_back(line.at(i));
                        break;
                }
          }
          target2X.push_back(atoi(x_coordinate.c_str()));
          target2Z.push_back(atoi(y_coordinate.c_str()));

    }
        while(getline(myfile3,line)){
          x_coordinate.clear();
          y_coordinate.clear();
          read_y = false;
          for(int i = 0 ; i < line.size(); i++ ){
          
                if(line.at(i) == ' '){
                    read_y = true;
                    continue;
                }
                
                switch(read_y){
                    case false :
                        x_coordinate.push_back(line.at(i));
                        break;
                    case true : 
                        y_coordinate.push_back(line.at(i));
                        break;
                }
          }
          target3X.push_back(atoi(x_coordinate.c_str()));
          target3Z.push_back(atoi(y_coordinate.c_str()));
    }
}

void wheretoGo::generatePaths(){

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


    
    for(int i = 0 ; i < 2 ; i++ ){
        switch(i){
            case 1:
                addStartPoint(40, 38);
                addTargetPoint(13, 176);
                break;
            case 2:
                addStartPoint(13, 176);
                addTargetPoint(41,7);
                break;
        }

        AddPotentialField(i); // Fill the matrix with the potantiel field
        calculateWavefront(i);
        sumMatrices(i);
        calculatePath(i);
        pathImprove(i);
        selectPath(i,0);
        clearMatrix();
    }
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
	
	addWall();
	
	 //Add barriers
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


void wheretoGo::clearMatrix(){

    for ( int k = 0 ; k < matrixHeight; k++){
		for(int i = 0; i < matrixWidth; i++){
			potantielFieldMatrix[k][i] = freeCell;
		}
	}


	for (int k = 0; k < matrixHeight; k++) {
	    for (int i = 0; i < matrixWidth; i++) {
			waveFrontMatrix[k][i] = freeCell;
		}
	}



}



void wheretoGo::sumMatrices(int i)
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
	
	
	string file_string = i + "sumMatrices.txt";

	writepotantielFieldMatrixToFile(file_string);
}






void wheretoGo::addStartPoint(int x, int y){

	pointx = x;
	pointy = y;

}

void wheretoGo::addTargetPoint(int x, int y){

	potantielFieldMatrix[x][y] = target;
	waveFrontMatrix[x][y] = target;

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

void wheretoGo::AddPotentialField(int i ) {

	for (int i = 0; i < matrixHeight; i++) {
		for (int j = 0; j < matrixWidth; j++) {


			if ((potantielFieldMatrix[i][j] != barrier) && (potantielFieldMatrix[i][j] != target)) {

				potantielFieldMatrix[i][j] = repelForce(i, j);

			}
		}
	}
	
	string file_string = i + "PotantielField.txt";
	
	writepotantielFieldMatrixToFile(file_string);
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


void wheretoGo::calculateWavefront(int i ) {

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
	
		waveFrontMatrix[targetx][targety] = target;
		
		string file_string; 
		file_string = i + "WaveFront.txt";
		
		writewaveFrontMatrixToFile(file_string);
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



void wheretoGo::calculatePath(int i) {


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

    string file_string = i + "FirstPath.txt";
	writePathToFile(file_string);
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

	double i = 0 ,j = 0;

	
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
	return newxy;
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


int wheretoGo::borderOut() {

	if (  (abs(pointx) > matrixHeight) || (abs(pointy) > matrixWidth) )
	{

		return -1;

	}
	return 1;

}

