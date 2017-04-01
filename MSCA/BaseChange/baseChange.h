/**********************************************************************
Onur Berk Töre
Yeditepe University, RIS Lab

2D Coordinates System Base Change

**********************************************************************/


#include <iostream>
#include <vector>

class baseChange {

public : 
	const float Xsrc_min = -100;
	const float Xsrc_max = +100;
	const float Xres_max = +100;
	const float Xres_min = 0;

	const float Zsrc_min = -40;
	const float Zsrc_max = +40;
	const float Zres_max = +100;
	const float Zres_min = 0;



	std::vector<float> translateNewPos(float x, float z);


};