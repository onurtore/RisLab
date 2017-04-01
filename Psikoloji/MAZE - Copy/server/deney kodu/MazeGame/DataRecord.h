#pragma once
#include "Public.h"
typedef SbVec3f Vector;

// define scenarios
#define STRAIGHT		1	// straight path
#define ROTATIONAL 		2	// rotational path
#define MIXED			3	// complex path

// define conditions
#define PRACTICE				10   // practice game where we show the springs
#define HARMONY					1	// harmonious operation
#define SINGLE_BLIND_A1			2	// single blind condition agent 1 is given goals
#define SINGLE_BLIND_A2			3	// single blind condition agent 2 is given goals
#define FULL_CONFLICT			4	// fully conflicting
#define PARTIAL_CONFLICT		5	// partially conflicting

#define COND_COUNT				6   // number of conditions

#define PERM_0                 0
#define PERM_1                 1
#define PERM_2                 2
#define PERM_3                 3
#define PERM_4                 4






public class DataRecord
{
public:
	DataRecord(void);
	void writeMeToFile();

	int	expDay;
	int	userID;		// which user currently playing	
	int perm;
	int	cond;		// which conflict condition
	int scenario;	// which scenario
	int session;
	int NUM_TRIALS_PER_COND;
	int practice_mode_trials;

	// like a timestamp, initially started at zero, 
	// precisely record the haptic thread running time
	vector<int> *runsCnt;	
	vector<int> *runsPerTrialCnt;
	vector<int> *conflictCnt;
	// how many times the current player has played the game.
	// reset at each theme change
	vector<int> *trialID;	

	vector<int>		*timeScore;

	vector<int> *target1_ACHIEVED;
	vector<int> *target2_ACHIEVED;


	vector<Vector>	*fOnCIP;
	vector<Vector>	*fOnHIP;

	
	vector<Vector>	*fOnCIP_SCALED;
	vector<Vector>	*fOnHIP_SCALED;
	
	vector<Vector>	*forceResistance;
	vector<Vector>  *forceFriction;

	vector<float>	*angularPosition;

	vector<float>	*angularVelocity;
	vector<float>	*angularAcc;

	vector<Vector>	*cipP;
	vector<Vector>	*hipP;
	vector<Vector>	*ballP;

	vector<Vector>	*cipV;
	vector<Vector>	*hipV;
	vector<Vector>	*ballV;
	vector<Vector>	*ballA;
	vector<int>		*errorT;
	vector<Vector>	*targetAgent1;
	vector<Vector>	*targetAgent2;
	vector<float>	*targetAngleAgent1;
	vector<float>	*targetAngleAgent2;
	vector<long>	*tick;

	
	
	/*
	
				
	// AYSE: time score reported to the user
	vector<int> *timeScore;
	// AYSE: servo ticks spent in pits
	vector<int> *runsInPits;
	// AYSE: time ticks spent in pits
	vector<int> *ticksInPits;
	// AYSE: servo ticks spent in pits
	vector<int> *runsOnTargets;
	// AYSE: time ticks spent in pits
	vector<int> *ticksOnTargets;
	vector<int> *numPitfalls;


	// AYSE: user defined moments for debugging purposes
	vector<bool> *flags;	
	vector<bool> *waitOnTargetFlags;
		
	// forces calculated
	vector<Vector> *fOnUser, *fOnBbyIner, *fOnBbySpr, *fOnCHIP;
	vector<Vector> *fOnNbyC, *fOnNbyBSpr, *fOnNbyH, *fOnNbyBIner;
	vector<float> *fOnUMag;
	vector<float> *smoothFOnUser, *smoothFxOnUser, *smoothFzOnUser;
	// position, velocity, and accelaration of the ball
	vector<Vector> *ballP, *ballV, *ballA;
	vector<float> *ballM, *ballR, *gameGravity;
	// positions of the control mechanisms and the target
	vector<Vector> *chipP, *chipV, *hipP, *hipV, *nhipP, *nhipV, *targetP;
	// spring and damper constants
	vector<float> *kpBN, *kdBN, *kpHN, *kpCN, *kpCT, *kdCT;
	// angles of the board
	vector<float> *thetaX, *thetaZ;
	// target id
	vector<int> *targetID;		// we have the position but in case we just need an ID
	// for determining CHIP's position, how many steps we are going to calculate
	vector<int> *chipIterCnt;	
	// theme: different background, target, etc colors.
	// we probably have 3 themes for 3 different scenarios
	// sce1: Both axes user controlled, no comp. contr.;
	// sce2: 1 axis user controlled, 1 axis computer controlled;
	// sce3: both the user and the computer control the 2 axes.
	vector<int> *boardTheme;	
	vector<int> *ctrlM;	
	vector<long> *tick;
	*/
};