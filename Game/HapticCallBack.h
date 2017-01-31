/**********************************************************************
Onur Berk Töre
Yeditepe University, RIS Lab
**********************************************************************/
#pragma once

#include "Public.h"


// HapticCallBack: For updating haptic loop and generating forces
class  HapticCallBack
{
public:
	HHD hHD1;

	HapticCallBack();
	virtual int initialize_geomagic(bool myboolean);
	virtual HDCallbackCode HDCALLBACK MyHapticLoop(void *pUserData);
	

};