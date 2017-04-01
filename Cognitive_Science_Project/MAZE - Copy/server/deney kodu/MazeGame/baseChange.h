/**********************************************************************
Onur Berk Töre
Yeditepe University, RIS Lab

2D Coordinates System Base Change 

**********************************************************************/

#include <iostream>
#include <vector>

class baseChange {

public : 

	baseChange();
	
	float Xsrc_min;
	float Xsrc_max;
	float Xres_max;
	float Xres_min;
	
	float Zsrc_min;
	float Zsrc_max;
	float Zres_max;
	float Zres_min;


	std::vector<float> translateNewPos(float x, float z);


};