#pragma once
#include "Public.h"
#include "HapticInterfacePoint.h"
#include "GraphicsController.h"
#include "TargetGraphics.h"

class GameController
{
public:

	GraphicsController *grCtrl;

	//HapticInterfacePoint *userIP;
	HapticInterfacePoint *contIP;
	//HapticInterfacePoint *negtIP;

	GameController();

	void addGraphicsController(GraphicsController *gC);
	void moveCtrller();

	// AYSE: implement levels within game
	int getLevel(); 
	void setLevel(int level);

private:
	void calculatePF();
};
