/**********************************************************************
Onur Berk Töre
Yeditepe University, RIS Lab


Potantial Field Method for point, 2D Environment

**********************************************************************/


#include <iostream>
#include "wheretoGo.h"
#include <conio.h>
using namespace std;



int main(){

	wheretoGo myClass;


	myClass.calculatePath();
	myClass.printMatrix();
	getch();
	

}