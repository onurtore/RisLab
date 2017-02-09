/**********************************************************************
Onur Berk Töre
Yeditepe University, RIS Lab


Potantial Field Method for point, in 2D Environment, with wavefront algorithm

**********************************************************************/


#include <iostream>
#include "wheretoGo.h"
#include <conio.h>
using namespace std;



int main(){

	wheretoGo myClass;
	myClass.calculateWavefront();
	myClass.pathImprove();

	
	myClass.calculatePath();
	myClass.printMatrix();
	
	cout << "The End\n";
	
	getch();
	

}
