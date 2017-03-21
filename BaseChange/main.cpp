/**********************************************************************
Onur Berk Töre
Yeditepe University, RIS Lab

2D Coordinates System Base Change

**********************************************************************/

//hello2sssdddd

#include <iostream>
#include <vector>
#include <conio.h>
#include "baseChange.h"

int main(int argc, char *argv[]) {

	baseChange PickOfDestiny;
	
	float x = 0;
	float y = 50;

	std::vector<float> newPos = PickOfDestiny.translateNewPos(x, y);
	
	std::cout << "newPos.size is : " << newPos.size() << "\n";
	
	for ( unsigned int i = 0; i < newPos.size(); i++) {
		std::cout << newPos.at(i) << " ";
	}

	_getch();

}