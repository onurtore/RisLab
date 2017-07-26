#pragma once
#include "Public.h"

public class ForceAngleRec
{
public:
	ForceAngleRec();
	void writeFile();

	//int	expDay;
	//int perm;   // which permutation we are in, determines the labels of the game
	//int	userID;	// which user currently playing	
	//int	cond; 
	//int session;

	// like a timestamp, initially started at zero, 
	// precisely record the haptic thread running time
	vector<int> *runsCnt;	
	vector<float> *forceFB1X, *forceFB1Z,*forceFB2X, *forceFB2Z;
	// how many times the current player has played the game.
	// reset at each theme change
	//vector<int> *trialID;				
	//// AYSE: time score reported to the user
	//vector<int> *timeScore;
	//// AYSE: servo ticks spent in pits
	//vector<int> *runsInPits;
	//// AYSE: time ticks spent in pits
	//vector<int> *ticksInPits;
	//// AYSE: servo ticks spent in pits
	//vector<int> *runsOnTargets;
	//// AYSE: time ticks spent in pits
	//vector<int> *ticksOnTargets;
	//vector<int> *numPitfalls;

	//	
	//// forces calculated
	//vector<Vector> *fOnUser, *fOnBbyIner, *fOnBbySpr, *fOnCIP;
	////////////////////vector<float> *forceFB1X, *forceFB1Z,*forceFB2X, *forceFB2Z;
	//vector<Vector> *fOnNbyC, *fOnNbyBSpr, *fOnNbyH, *fOnNbyBIner;
	//vector<float> *fOnUMag;
	//vector<float> *smoothFOnUser, *smoothFxOnUser, *smoothFzOnUser;
	//// position, velocity, and accelaration of the ball
	//vector<Vector> *ballP, *ballV, *ballA;
	//vector<float> *ballM, *ballR, *gameGravity;
	//// positions of the control mechanisms and the target
	//vector<Vector> *cipP, *chipV, *hipP, *hipV, *nipP, *nhipV, *targetP;
	//// spring and damper constants
	//vector<float> *kpBN, *kdBN, *kpHN, *kpCN, *kpCT, *kdCT;
	//// angles of the board
	//vector<float> *thetaX, *thetaZ;
	////////////////////////////////////vector<float> *thetaY;
	//// target id
	//vector<int> *targetID;		// we have the position but in case we just need an ID
	//// for determining CIP's position, how many steps we are going to calculate
	//vector<int> *chipIterCnt;	
	//// theme: different background, target, etc colors.
	//// we probably have 3 themes for 3 different scenarios
	//// sce1: Both axes user controlled, no comp. contr.;
	//// sce2: 1 axis user controlled, 1 axis computer controlled;
	//// sce3: both the user and the computer control the 2 axes.
	//vector<int> *boardTheme;	
	//vector<int> *ctrlM;	
	//vector<long> *tick;
};