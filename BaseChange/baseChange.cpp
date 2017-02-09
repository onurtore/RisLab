/**********************************************************************
Onur Berk Töre
Yeditepe University, RIS Lab

2D Coordinates System Base Change 

**********************************************************************/
#include "baseChange.h"

std::vector<float> baseChange::translateNewPos(float x, float z)
{
	float res_x = (x - Xsrc_min) / (Xsrc_max - Xsrc_min) * (Xres_max - Xres_min) + Xres_min;
	float res_y = (z - Zsrc_min) / (Zsrc_max - Zsrc_min) * (Zres_max - Zres_min) + Zres_min;

	return std::vector<float>{ res_x,res_y };
}
