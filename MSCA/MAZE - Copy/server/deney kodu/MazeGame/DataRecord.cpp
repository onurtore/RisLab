#include "DataRecord.h"

DataRecord::DataRecord(void)
{
	expDay		= 1;
	perm		= 0;     // first permutation
	userID		= 1;	 // which user currently playing	
	scenario	= MIXED;
	//cond		= FULL_CONFLICT ;
		

	NUM_TRIALS_PER_COND = NUM_TRIALS_PER_COND_S;
	practice_mode_trials = PRACTICE_COUNT;
	
	// AYSE: what was that?
	session			  = 1;//NUM_TRIALS_PER_COND;
	
 
	runsCnt			  = new vector<int>;
	runsPerTrialCnt	  = new vector<int>;
	conflictCnt		  = new vector<int>;

	trialID			  = new vector<int>;

	//CIGIL 
	timeScore		  = new vector<int>;
	
	target1_ACHIEVED  = new vector<int>;
	target2_ACHIEVED  = new vector<int>;

	fOnCIP		      = new vector<Vector>;
	fOnHIP			  = new vector<Vector>;

	fOnCIP_SCALED	  = new vector<Vector>;
	fOnHIP_SCALED	  = new vector<Vector>;

	forceResistance   = new vector<Vector>;
	
	forceFriction   = new vector<Vector>;
	
	angularPosition	  = new vector<float>;
	angularVelocity	  = new vector<float>;
	angularAcc		  = new vector<float>;
	
	cipP			  = new vector<Vector>;
	hipP			  = new vector<Vector>;
	ballP			  = new vector<Vector>;

	cipV			  = new vector<Vector>;
	hipV			  = new vector<Vector>;
	ballV		      = new vector<Vector>;
	ballA			  = new vector<Vector>;
	errorT            = new vector<int>;
	targetAgent1	  = new vector<Vector>;
	targetAgent2	  = new vector<Vector>;
	targetAngleAgent1 = new vector<float>;
	targetAngleAgent2 = new vector<float>;
	tick			  = new vector<long>;
}

// AYSE: write to a single file.
void DataRecord::writeMeToFile()
{
	int prevTrialID = 1;
	
	char fOutName[24];
	ofstream fileOutPos;

	cout << "Writing file to disk." << endl;
	
	// AYSE: write to a single file 
	/*sprintf(fOutName, "e%02d_c%02d_u%02d_t%02d.txt", expDay, cond, userID, (session-1)*NUM_TRIALS_PER_COND + 1);*/
	sprintf(fOutName, "e%02d_s%02d_p%02d_u%02d.txt", expDay, scenario, perm, userID);
	
	cout << fOutName << endl;
	//ofstream fileOutPos;
	fileOutPos.open(fOutName);

	for(unsigned int i = 0; i < runsCnt->size(); i++){

		fileOutPos	
			<< tick->at(i)				<< "\t" 	
			<< errorT->at(i)			<< "\t"
			<< runsCnt->at(i)			<< "\t" << runsPerTrialCnt->at(i)   << "\t"	
			<< conflictCnt->at(i)		<< "\t" 
			<< trialID->at(i)		 	<< "\t"	    
			<< target1_ACHIEVED->at(i)	<< "\t"<< target2_ACHIEVED->at(i)	<< "\t"		
			<< targetAgent1->at(i)[0]	<< "\t" << targetAgent1->at(i)[2]	<< "\t"
			<< targetAgent2->at(i)[0]	<< "\t" << targetAgent2->at(i)[2]	<< "\t"
			<< targetAngleAgent1->at(i)	<< "\t" << targetAngleAgent2->at(i)	<< "\t"
			<< angularPosition->at(i)   << "\t" << angularVelocity->at(i)   << "\t"   
			<< angularAcc->at(i)        << "\t" 
			<< fOnCIP->at(i)[0]		    << "\t"	<< fOnCIP->at(i)[2]		    << "\t" 
			<< fOnHIP->at(i)[0]		    << "\t"	<< fOnHIP->at(i)[2]		    << "\t"		
			
			<< fOnCIP_SCALED->at(i)[0]  << "\t"	<< fOnCIP_SCALED->at(i)[2]	<< "\t" 
			<< fOnHIP_SCALED->at(i)[0]  << "\t"	<< fOnHIP_SCALED->at(i)[2]  << "\t"	

			<< forceResistance->at(i)[0]<< "\t"	<< forceResistance->at(i)[2]<< "\t"	
			<< forceFriction->at(i)[0]<< "\t"	<< forceFriction->at(i)[2]<< "\t"	
			<< ballP->at(i)[0]			<< "\t" <<	ballP->at(i)[2]			<< "\t"		
			<< ballV->at(i)[0]			<< "\t" <<	ballV->at(i)[2]			<< "\t"		
			<< ballA->at(i)[0]			<< "\t" <<	ballA->at(i)[2]			<< "\t"		
			<< cipP->at(i)[0]			<< "\t" <<	cipP->at(i)[2]			<< "\t"		
			<< cipV->at(i)[0]			<< "\t" <<	cipV->at(i)[2]			<< "\t"		
			<< hipP->at(i)[0]			<< "\t" <<	hipP->at(i)[2]			<< "\t"		
			<< hipV->at(i)[0]			<< "\t" <<	hipV->at(i)[2]			<< "\t"		
			<< timeScore->at(i)	<<endl;


			//      << tick->at(i)				<< "\t"	<<	runsCnt->at(i)			<< "\t"		// 1
		    //		<< targetID->at(i)			<< "\t" <<	trialID->at(i)			<< "\t"		// 2
			//		<< thetaX->at(i)			<< "\t" <<	thetaZ->at(i)			<< "\t"		// 3
			//		<< fOnUser->at(i)[0]		<< "\t" <<	fOnUser->at(i)[2]		<< "\t"		// 4
			//		/*<< tmpFBI[0]				<< "\t" <<	tmpFBI[2]				<< "\t"
			//		<< tmpFBS[0]				<< "\t" <<	tmpFBS[2]				<< "\t"*/
			//		<< fOnCHIP->at(i)[0]		<< "\t" <<	fOnCHIP->at(i)[2]		<< "\t"		// 5
			//		<< fOnNbyC->at(i)[0]		<< "\t" <<	fOnNbyC->at(i)[2]		<< "\t"		// 6
			//		<< fOnNbyH->at(i)[0]		<< "\t" <<	fOnNbyH->at(i)[2]		<< "\t"		// 7
			//		<< fOnNbyBSpr->at(i)[0]		<< "\t" <<	fOnNbyBSpr->at(i)[2]	<< "\t"		// 8
			//		<< fOnNbyBIner->at(i)[0]	<< "\t" <<	fOnNbyBIner->at(i)[2]	<< "\t"		// 9
			//		<< ballP->at(i)[0]			<< "\t" <<	ballP->at(i)[2]			<< "\t"		// 10
			//		<< ballV->at(i)[0]			<< "\t" <<	ballV->at(i)[2]			<< "\t"		// 11
			//		<< ballA->at(i)[0]			<< "\t" <<	ballA->at(i)[2]			<< "\t"		// 12
			//		<< chipP->at(i)[0]			<< "\t" <<	chipP->at(i)[2]			<< "\t"		// 13
			//		<< chipV->at(i)[0]			<< "\t" <<	chipV->at(i)[2]			<< "\t"		// 14
			//		<< hipP->at(i)[0]			<< "\t" <<	hipP->at(i)[2]			<< "\t"		// 15
			//		<< nhipP->at(i)[0]			<< "\t" <<	nhipP->at(i)[2]			<< "\t";     // 16

		
		//fileOutPos	<< timeScore->at(i)	<<endl;//<< flags->at(i)				<< "\t" <<	waitOnTargetFlags->at(i)<< "\t"		// 18
					//<< timeScore->at(i)	<<endl;		//<< "\t"	<<  runsInPits->at(i)		<< "\t"		// 19
					//<< ticksInPits->at(i)		<< "\t"	<<  runsOnTargets->at(i)	<< "\t"		// 20
					//<< ticksOnTargets->at(i)	<< "\t"	<<	numPitfalls->at(i)		<< "\t"		// 21
					//<< endl;

		//<< tmpFUMag		<< "\t"	<< tmpSmoF		<< endl;
					//<< tmpSmoFx		<< "\t"	<< tmpSmoFz		<< endl;	
	}
	fileOutPos.close();
	cout << "File I \\ O done..." << endl;
}