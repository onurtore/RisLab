/**********************************************************************
Onur Berk Töre
Yeditepe University, RIS Lab

2D Coordinates System Base Change 

**********************************************************************/

#include "baseChange.h"

baseChange::baseChange(){

	Xsrc_min = -100;
	Xsrc_max = +100;
	Xres_max = +100;
	Xres_min = 0;
	
	Zsrc_min = -41;
	Zsrc_max = +41;
	Zres_max = +100;
	Zres_min = 0;



}

std::vector<float> baseChange::translateNewPos(float x, float z)
{
	float res_x = (x - Xsrc_min) / (Xsrc_max - Xsrc_min) * (Xres_max - Xres_min) + Xres_min;
	float res_y = (z - Zsrc_min) / (Zsrc_max - Zsrc_min) * (Zres_max - Zres_min) + Zres_min;


	/*C++11 
	return std::vector<float>{ res_x,res_y };
	*/	
	std::vector<float> newPos(2);
	newPos.at(0) = res_x;
	newPos.at(1) = res_y;
	
	return newPos;
}
