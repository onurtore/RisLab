#include "ForceAngleRec.h"


ForceAngleRec::ForceAngleRec(void)
{
	//expDay	= 9;
	//perm	= 6;
	//userID	= 32;			// which user currently playing	
	//cond	= SC;
	//#ifdef PLAYONCE
	//	session	= 6;
	//#else
	//	session = 1;
	//#endif
	//
	runsCnt			= new vector<int>;
	//trialID			= new vector<int>;

	//timeScore		= new vector<int>;
	//runsInPits		= new vector<int>;
	//ticksInPits		= new vector<int>;
	//runsOnTargets	= new vector<int>;
	//ticksOnTargets	= new vector<int>;
	//numPitfalls		= new vector<int>;

	//fOnUser			= new vector<Vector>;
	//fOnUMag			= new vector<float>;
	////smoothFOnUser	= new vector<float>;
	//smoothFxOnUser	= new vector<float>;
	//smoothFzOnUser	= new vector<float>;
	//fOnBbyIner		= new vector<Vector>;
	//fOnBbySpr		= new vector<Vector>;
	//fOnCIP			= new vector<Vector>;
	//fOnNbyC			= new vector<Vector>;
	//fOnNbyBSpr		= new vector<Vector>;
	//fOnNbyH			= new vector<Vector>;
	//fOnNbyBIner		= new vector<Vector>;
	//////////forceFB1X=new vector<float>;
	//////////forceFB1Z=new vector<float>;
	//////////forceFB2X=new vector<float>; 
	//////////forceFB2Z=new vector<float>;
	//////////thetaY=new vector<float>;
	//ballP			= new vector<Vector>;
	//ballV			= new vector<Vector>;
	//ballA			= new vector<Vector>;

	//ballM			= new vector<float>;
	//ballR			= new vector<float>;
	//gameGravity		= new vector<float>;

	//cipP			= new vector<Vector>;
	//hipP			= new vector<Vector>;
	//nipP			= new vector<Vector>;
	//chipV			= new vector<Vector>;
	//hipV			= new vector<Vector>;
	//nhipV			= new vector<Vector>;
	//targetP			= new vector<Vector>;

	//kpBN			= new vector<float>;
	//kdBN			= new vector<float>;
	//kpHN			= new vector<float>;
	//kpCN			= new vector<float>;
	//kpCT			= new vector<float>;
	//kdCT			= new vector<float>;
	//thetaX			= new vector<float>;
	//thetaZ			= new vector<float>;
	//targetID		= new vector<int>;
	//chipIterCnt		= new vector<int>;
	//boardTheme		= new vector<int>;
	//ctrlM			= new vector<int>;
	//tick			= new vector<long>;
}

void ForceAngleRec::writeFile()
{
	//int prevTrialID = 1;
	//
	//cout << "I have read " << runsCnt->size()	<< " values..\n";
	//char fOutName[24];
	ofstream fileOutF2;
    char fOutName[30];
	//cout << "Writing file " << 1 << " to disk." << endl;
	//sprintf(fOutName, "e%02d_c%02d_u%02d_t%02d.txt", expDay, cond, userID, (session-1)*NUM_TRIALS_PER_COND + 1);
	sprintf(fOutName, "force_angle.txt");
	//cout << fOutName << endl;
	////ofstream fileOutPos;
	//fileOutPos.open(fOutName);
	//ofstream fileOutF2;
	fileOutF2.open(fOutName);

	for(unsigned int i = 0; i < runsCnt->size(); i++)
	{
     fileOutF2	 << runsCnt->at(i)		<< "\t"	<<  0/*thetay->at(i)*/	<< "\t"	
	     		 << forceFB1X	        << "\t" <<	forceFB1Z		<< "\t"		
                 << forceFB2X	        << "\t" <<	forceFB2Z		<< endl		;
	 // 3
	//	if (trialID->at(i) != prevTrialID)
	//	{
	//		fileOutPos.close();
	//		cout << "Writing file " << trialID->at(i) << " to disk." << endl;
	//		sprintf(fOutName, "e%02d_c%02d_u%02d_t%02d.txt", expDay, cond, userID, (session-1)*NUM_TRIALS_PER_COND + trialID->at(i));
	//		cout << fOutName << endl;
	//		//ofstream fileOutPos;
	//		fileOutPos.open(fOutName);
	//	}

	//	prevTrialID = trialID->at(i);

	//	fileOutPos	<< tick->at(i)				<< "\t"	<<	runsCnt->at(i)			<< "\t"		// 1
	//				<< targetID->at(i)			<< "\t" <<	trialID->at(i)			<< "\t"		// 2
	//				<< thetaX->at(i)			<< "\t" <<	thetaZ->at(i)			<< "\t"		// 3
	//				<< fOnUser->at(i)[0]		<< "\t" <<	fOnUser->at(i)[2]		<< "\t"		// 4
	//				/*<< tmpFBI[0]				<< "\t" <<	tmpFBI[2]				<< "\t"
	//				<< tmpFBS[0]				<< "\t" <<	tmpFBS[2]				<< "\t"*/
	//				<< fOnCIP->at(i)[0]		<< "\t" <<	fOnCIP->at(i)[2]		<< "\t"		// 5
	//				<< fOnNbyC->at(i)[0]		<< "\t" <<	fOnNbyC->at(i)[2]		<< "\t"		// 6
	//				<< fOnNbyH->at(i)[0]		<< "\t" <<	fOnNbyH->at(i)[2]		<< "\t"		// 7
	//				<< fOnNbyBSpr->at(i)[0]		<< "\t" <<	fOnNbyBSpr->at(i)[2]	<< "\t"		// 8
	//				<< fOnNbyBIner->at(i)[0]	<< "\t" <<	fOnNbyBIner->at(i)[2]	<< "\t"		// 9
	//				<< ballP->at(i)[0]			<< "\t" <<	ballP->at(i)[2]			<< "\t"		// 10
	//				<< ballV->at(i)[0]			<< "\t" <<	ballV->at(i)[2]			<< "\t"		// 11
	//				<< ballA->at(i)[0]			<< "\t" <<	ballA->at(i)[2]			<< "\t"		// 12
	//				<< cipP->at(i)[0]			<< "\t" <<	cipP->at(i)[2]			<< "\t"		// 13
	//				<< chipV->at(i)[0]			<< "\t" <<	chipV->at(i)[2]			<< "\t"		// 14
	//				<< hipP->at(i)[0]			<< "\t" <<	hipP->at(i)[2]			<< "\t"		// 15
	//				<< nipP->at(i)[0]			<< "\t" <<	nipP->at(i)[2]			<< "\t";     // 16
	//	if ( this->cond == RE || this->cond == REVHC )
	//	{
	//		fileOutPos	<< kpCN->at(i)			<< "\t" <<	ctrlM->at(i)			<< "\t";	// 17
	//	}
	//	fileOutPos	<< flags->at(i)				<< "\t" <<	waitOnTargetFlags->at(i)<< "\t"		// 18
	//				<< timeScore->at(i)			<< "\t"	<<  runsInPits->at(i)		<< "\t"		// 19
	//				<< ticksInPits->at(i)		<< "\t"	<<  runsOnTargets->at(i)	<< "\t"		// 20
	//				<< ticksOnTargets->at(i)	<< "\t"	<<	numPitfalls->at(i)		<< "\t"		// 21
	//				<< endl;
	//	//<< tmpFUMag		<< "\t"	<< tmpSmoF		<< endl;
	//				//<< tmpSmoFx		<< "\t"	<< tmpSmoFz		<< endl;	
	//}
	//fileOutPos.close();
	}
    fileOutF2.close();
	//cout << "File I \\ O done..." << endl;

}