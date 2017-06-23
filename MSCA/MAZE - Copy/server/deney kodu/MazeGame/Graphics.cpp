/////////////////////////////////////////////////////////////////
//
// A sample program for haptic interaction with two phantoms
// by Ayse Kucukyilmaz 
// Nov, 2011
//
/////////////////////////////////////////////////////////////////
#include <fstream>
#include "Public.h"
#include "HapticCallBack.h"
#include "MathCB.h"
#include "HapticInterfacePoint.h"
#include "GraphicsController.h"
#include "SoundPlayerThread.h"
//Onur Path Finder function
#include "wheretoGo.h"
#include "baseChange.h"
// FUNCTION PROTOTYPES
void myKeyPressCB(void *, SoEventCallback *);
void MyHandleClose(void *,class SoWinComponent *);
void graphicsTimerCallback(void *data, SoSensor *);
void InitialSetup();
void displayUsage();
void GenerateDefaultGraphics(SoSeparator *);
void outputToScreen();
void initScreenText();
void setMessageText(char* msg);
void setScoreText(char* score);
void setWarningText(char* warning);
void setTrialText(int trialNum);

extern void setTargetText1(char* target);
extern void setTargetText2(char* target);

void writeFile();
// GLOBALS
SoTransform *stylus_transform;
Vector stylusOffset;
SbMatrix stylusTransMatrix1;
//SbMat stylusTransMatrix1;
SoTransform *stylus_transform2;
Vector stylusOffset2;
SbMatrix stylusTransMatrix2;

//SbMat stylusTransMatrix2;
float stylusR;
float stylusR2;
int runds=1;



SoTransform *boardRotateX, *boardRotateZ;
SbMatrix boardRotateMatrixX, boardRotateMatrixZ;

string messageString;
string scoreString;
string warningString;
string trialString;
string startString;
string targetString1;
string targetString2;
string gameString;


//EXTERNED VARIABLES
extern float kpBN, kdBN, kpCT, kdCT;

//extern Ball *ball;
extern HapticInterfacePoint *CIP, *NIP, *HIP;
//AYSE: externed from Haptic.cpp for role exchange visualization

extern int ctrlMode;

extern SbVec3f* computedForces;
extern float TARGET_POS_A1[NUM_TRIALS_PER_COND_M][3]; // target Positions for player 1
extern float TARGET_POS_A2[NUM_TRIALS_PER_COND_M][3]; // target Positions for player 2 

extern float PRACTICE_TARGET_POS_A1_S [PRACTICE_COUNTS][3];
extern float PRACTICE_TARGET_POS_A2_S [PRACTICE_COUNTS][3];

extern float PRACTICE_TARGET_POS_A1_M [PRACTICE_COUNTS][3];
extern float PRACTICE_TARGET_POS_A2_M [PRACTICE_COUNTS][3];


extern float PRACTICE_TARGET_ANGLE_A1[PRACTICE_COUNTS];
extern float PRACTICE_TARGET_ANGLE_A2[PRACTICE_COUNTS];

extern float TARGET_ANGLE_A1[NUM_TRIALS_PER_COND_M];
extern float TARGET_ANGLE_A2[NUM_TRIALS_PER_COND_M];

float PRACTICE_TARGET_POSITION_A1_S  [NUM_TRIALS_PER_COND_M][3];
float PRACTICE_TARGET_POSITION_A2_S  [NUM_TRIALS_PER_COND_M][3];




float TARGET_POSITION_A1_S[7][3]; // target Positions for player 1
float TARGET_POSITION_A2_S[7][3];


float TARGET_POSITION_A1_M[6][3]; // target Positions for player 1
float TARGET_POSITION_A2_M[6][3];


// AYSE: VISUALIZATION VARIABLES
// path visualization
SoCoordinate3	*pathBCoor, *cipPathCoor, *nipPathCoor;
SoLineSet		*pathBLines, *cipPathLines, *nipPathLpines;
SoFont			*pathBFont;

//for printing a meesage on screen
SoAsciiText *messageText;


//for printing the trial number on screen
SoAsciiText *trialText;

//AYSE: for printing the score on screen
SoAsciiText *scoreText;

//AYSE: for printing the warnings on screen to motivate user
SoAsciiText *warningText;

SoAsciiText *startText;

SoAsciiText *targetText1;
SoAsciiText *targetText2;


SoAsciiText *gameText;

// GLOBALS for synchronization
// variables need to be setup by InitialSetup()
bool myStylusFlag;

HapticCallBack *effect;
vector <HDSchedulerHandle> callbackHandlers;;
extern int initialize_phantom(bool myboolean);
extern void stopHaptics();




extern HDCallbackCode HDCALLBACK MyHapticLoop(void *pUserData);
extern HDCallbackCode HDCALLBACK DutyCycleCallback(void *pUserData);
extern HDCallbackCode HDCALLBACK QueryMotorTemp(void *pUserData);
extern HDCallbackCode HDCALLBACK ComputeForceCallback(void *pUserData);
extern HDCallbackCode HDCALLBACK BeginFrameCallback(void *);
extern HDCallbackCode HDCALLBACK EndFrameCallback(void *);
extern char *RecordCallback(void *pUserData);
extern HDdouble *aMotorTemp = 0;

//These are must be global Onur? 
//YES THESE ARE MUST BE GLOBAL
wheretoGo path;

vector<double> hapticForce;
bool firstTime = true;
//These are must be global ?? 
//	
// file output
DataRecord *dataRec;
//ForceAngleRec *faRec;
GraphicsController *graphCtrller;
//GameController *gameCtrller;
HHOOK hMessageBoxHook_;
static int messageBoxCounter = 0;

//Onur
vector<  vector< float  > > lineEquations;


// callback for hooking unwanted coin error message boxes
LRESULT CALLBACK CbtHookProc(int nCode, WPARAM wParam, LPARAM lParam)
{
	messageBoxCounter++; 

	if (messageBoxCounter == 1)
		return ::CallNextHookEx(    hMessageBoxHook_, 
		nCode, 
		wParam, 
		lParam); 

	if(nCode < 0)
	{
		return ::CallNextHookEx(    hMessageBoxHook_, 
			nCode, 
			wParam, 
			lParam); 
	}

	switch(nCode)
	{
	case HCBT_CREATEWND: // a window is about to be created
		return -1;

	}

	return ::CallNextHookEx(    hMessageBoxHook_, 
		nCode, 
		wParam, 
		lParam); 
}

/*
* Usage: MazeGame <user_id> <scenario> <cond>
where:
scenario = 1 -> straight path
scenario = 2 -> rotational path
scenario = 3 -> visual cues

cond = 0 -> practice game
cond = 1 -> fully conflicting
cond = 2 -> partially conflicting
cond = 3 -> single blind condition agent 1 is given goals
cond = 4 -> single blind condition agent 2 is given goals
cond = 5 -> harmonious operation
*/


void createLine(int x1, int y1,int x2, int y2) {

	vector<float> lineEq;

	float r;
	if( x2 - x1 == 0 ){

		r = 0.001;
	}

	else{

		r = x2 - x1;
	}

	float slope = (y2 - y1) / (r);

	//y2 = slope * x2 + b
	
	//y2 - (slope * x2) = b

	float b = y2 - (slope * x2);

	lineEq.push_back(y2);
	lineEq.push_back(slope);
	lineEq.push_back(b);


	lineEquations.push_back(lineEq);



	return;
}

int main(int argc, char **argv)
{

//Onur
	path.AddPotentialField(); // Fill the matrix with the potantiel field
	cout << "First Step Done - Potential Field Added" << "\n\n"; 
	path.calculateWavefront();
	cout << "Second Step Done - Wavefront Added" << "\n\n";
	path.sumMatrices();
	cout << "Third Step Done - Wavefront Added to Potantiel Field Matrix" << "\n\n";
	path.calculatePath();
	cout << "Fourth Step Done - Path Calculated" << "\n\n";
	path.pathImprove();
	cout << "Fifth Step Done - Path Improved" << "\n\n";
	path.selectPath(1);
	cout << "Sixth Step Done - Path Selected" << "\n\n";
	for (int i = 0; i <  path.dtargetsX.size(); i++) {
		path.isVisited.push_back(false);
	}



	for (int i = 0; i+1 < path.dtargetsX.size(); i++) {
		createLine(path.dtargetsX.at(i),path.dtargetsY.at(i), path.dtargetsX.at(i+1), path.dtargetsY.at(i+1));
	}

//Onur


	
	
//Onur
	int screenWidth = 1080;
	int screenHeight = 900;

	dataRec = new DataRecord();


	if (argc >= 4)
	{
		dataRec->userID		= atoi(argv[1]);
		dataRec->scenario	= atoi(argv[2]);
		dataRec->cond		= atoi(argv[3]);
	}

	if (argc == 5)
	{
		dataRec->perm = atoi(argv[4]);
	}

	// if practice condition we'll see the translation scenario
	if (dataRec->cond == PRACTICE)
	{
		//dataRec->scenario = STRAIGHT;
		//NUM_TRIALS_PER_COND = 2;
	}

	HWND myWindow = SoWin::init("Player1");

	if(myWindow == NULL)
	{   //fclose (pFile);
		return(-1);
	}

	hMessageBoxHook_ = SetWindowsHookEx(WH_CBT, 
		&CbtHookProc, 
		::GetModuleHandle(NULL), 
		GetCurrentThreadId());


	HWND myWindow2 = SoWin::init("Player2");

	::UnhookWindowsHookEx(hMessageBoxHook_);
	hMessageBoxHook_ = 0;

	if(myWindow2 == NULL)
	{   //fclose (pFile);
		return(-1);
	}

	SetWindowPos(myWindow,HWND_TOP,0,0,screenWidth,screenHeight,SWP_SHOWWINDOW);
	SetWindowPos(myWindow2,HWND_TOPMOST,screenWidth,0,screenWidth,screenHeight,SWP_SHOWWINDOW);
#ifdef FULL_SCREEN
	//SetWindowLong(myWindow,GWL_STYLE,GetWindowLong(myWindow,GWL_STYLE) & !WS_BORDER & !WS_SIZEBOX & !WS_DLGFRAME);
	//SetWindowPos(myWindow,HWND_TOP,0,0,100,100,SWP_SHOWWINDOW);
#endif
	// *****************************************************************
	// create the scene graph
	//gameCtrller = new GameController();
	graphCtrller = new GraphicsController(dataRec->scenario, dataRec->cond);

	// Define target positions
	Vector boundary = graphCtrller->boardGr->boundary;
	float boundaryThickness = graphCtrller->boardGr->boundaryThickness;


	// VERTICAL ALIGNMENT OF TARGETS -- z direction
	// AYSE: all zeros
	for (int i = 0; i < dataRec->NUM_TRIALS_PER_COND; i++)
	{
		TARGET_POS_A1[i][0] = SHIFT_X;
		TARGET_POS_A2[i][0] = SHIFT_X;

		TARGET_POS_A1[i][2] = SHIFT_Z;
		TARGET_POS_A2[i][2] = SHIFT_Z;
		



		TARGET_ANGLE_A1[i] = 0;
		TARGET_ANGLE_A2[i] = 0;
	}
	

	//Target positions and angles for practice trials both in straight and mixed scenes
	for (int i = 0; i < PRACTICE_COUNT; i++)
	{
	  PRACTICE_TARGET_POS_A1_S [i][0] = SHIFT_X;
	  PRACTICE_TARGET_POS_A2_S [i][0] = SHIFT_X;

	  PRACTICE_TARGET_POS_A1_S [i][0] = SHIFT_Z;
	  PRACTICE_TARGET_POS_A2_S [i][0] = SHIFT_Z;

	  PRACTICE_TARGET_POS_A1_M [i][0] = SHIFT_X;
	  PRACTICE_TARGET_POS_A2_M [i][0] = SHIFT_X;

	  PRACTICE_TARGET_POS_A1_M [i][0] = SHIFT_Z;
	  PRACTICE_TARGET_POS_A2_M [i][0] = SHIFT_Z;

	  PRACTICE_TARGET_ANGLE_A1[i] = 0;
	  PRACTICE_TARGET_ANGLE_A2[i] = 0;
	}

    //Practice target positions for straight scene
	PRACTICE_TARGET_POSITION_A1_S [1][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 2 * TARGET_WIDTH /3.0); 
	PRACTICE_TARGET_POSITION_A1_S [2][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 2 * TARGET_WIDTH /3.0); 
	PRACTICE_TARGET_POSITION_A1_S [0][0] =  SHIFT_X;


	//Target positions for both players in straight scene
	TARGET_POSITION_A1_S [1][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 2 * TARGET_WIDTH /3.0); // target at righ , harmony
	TARGET_POSITION_A1_S [2][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 8 + TARGET_WIDTH /3.0);//  targets at left, single blind1
	TARGET_POSITION_A1_S [3][0] = -10000; // don't show targe (single blind2)
	TARGET_POSITION_A1_S [4][0] = (SHIFT_X); // target at middle, harmony
	TARGET_POSITION_A1_S [5][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 2 * TARGET_WIDTH /3.0); //  target at right, fully
	TARGET_POSITION_A1_S [6][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 2 * TARGET_WIDTH /3.0); // target at right, harmony 
	TARGET_POSITION_A1_S [0][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 8 + INITIAL_TARGET ); //targets at left, partial
	

	TARGET_POSITION_A2_S [1][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 2 * TARGET_WIDTH /3.0); // target at right, harmony
	TARGET_POSITION_A2_S [2][0] = -10000; // don't show target (single blind1)
	TARGET_POSITION_A2_S [3][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 8 - TARGET_WIDTH /3.0); //target at right (single blind2)
	TARGET_POSITION_A2_S [4][0] = (SHIFT_X); //target at middle, harmony
	TARGET_POSITION_A2_S [5][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 2 * TARGET_WIDTH /3.0); //target at left, fully
	TARGET_POSITION_A2_S [6][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 2 * TARGET_WIDTH /3.0); // target at right, harmony
	TARGET_POSITION_A2_S [0][0] = (SHIFT_X - (boundary[0]-boundaryThickness)  + 8 + TARGET_WIDTH /3.0); // targets at left, partial
	

	
	//Target positions for both players in mixed scene
	TARGET_POSITION_A1_M [1][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 2 * TARGET_WIDTH /3.0); //harmony, target at right
	TARGET_POSITION_A1_M [1][2] = (SHIFT_Z - wallDepth*0.5 -0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );
    TARGET_POSITION_A1_M [0][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 2 * TARGET_WIDTH /3.0); // conditional harmony, target at left, conditional harmony
	TARGET_POSITION_A1_M [0][2] = (SHIFT_Z); 
	TARGET_POSITION_A1_M [2][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 8 - TARGET_WIDTH /3.0); // targets at right,single blind 1
	TARGET_POSITION_A1_M [2][2] = (SHIFT_Z + wallDepth*0.5 +0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );
    TARGET_POSITION_A1_M [3][0] = -10000;// targets at right,single blind 2
	TARGET_POSITION_A1_M [3][2] = (SHIFT_Z - wallDepth*0.5 -0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );
	TARGET_POSITION_A1_M [4][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 8 - TARGET_WIDTH /3.0); //targets at right, fully 
	TARGET_POSITION_A1_M [4][2] = (SHIFT_Z + wallDepth*0.5 +0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );
    TARGET_POSITION_A1_M [5][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 38 - TARGET_WIDTH /3.0); // targets at right, partially
	TARGET_POSITION_A1_M [5][2] = (SHIFT_Z - wallDepth*0.5 -0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );


	TARGET_POSITION_A2_M [1][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 2 * TARGET_WIDTH /3.0);  //harmony, target at right
	TARGET_POSITION_A2_M [1][2] = (SHIFT_Z - wallDepth*0.5 -0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );
    TARGET_POSITION_A2_M [0][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 2 * TARGET_WIDTH /3.0); // conditional harmony, target at left
	TARGET_POSITION_A2_M [0][2] = (SHIFT_Z); 
	TARGET_POSITION_A2_M [2][0] = -10000; // single blind 1, targets at right
	TARGET_POSITION_A2_M [2][2] = (SHIFT_Z + wallDepth*0.5 +0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );
    TARGET_POSITION_A2_M [3][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 2 * TARGET_WIDTH /3.0); //target at right, single blind 2
	TARGET_POSITION_A2_M [3][2] = (SHIFT_Z - wallDepth*0.5 -0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );
	TARGET_POSITION_A2_M [4][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 8 - TARGET_WIDTH /3.0); //target at right, fully conflict
	TARGET_POSITION_A2_M [4][2] = (SHIFT_Z - wallDepth*0.5 - 0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );
    TARGET_POSITION_A2_M [5][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 28 - INITIAL_TARGET ); // targets at right, partial conflict
	TARGET_POSITION_A2_M [5][2] = (SHIFT_Z - wallDepth*0.5 -0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );


    
    





	if (dataRec->scenario == STRAIGHT)
	{		
		if(dataRec->perm == 7)
		{
			PRACTICE_TARGET_POS_A1_S [1][0] =  PRACTICE_TARGET_POSITION_A1_S [1][0];
			PRACTICE_TARGET_POS_A1_S [0][0] =  PRACTICE_TARGET_POSITION_A1_S [2][0];

			PRACTICE_TARGET_POS_A2_S [1][0] =  PRACTICE_TARGET_POSITION_A1_S [1][0];
			PRACTICE_TARGET_POS_A2_S [0][0] =  PRACTICE_TARGET_POSITION_A1_S [2][0];
		}
		else
		{
			PRACTICE_TARGET_POS_A1_S [1][0] =  PRACTICE_TARGET_POSITION_A1_S [practice_PERMS[dataRec->perm][0]][0];
			PRACTICE_TARGET_POS_A1_S [0][0] =  PRACTICE_TARGET_POSITION_A1_S [practice_PERMS[dataRec->perm][1]][0];

			PRACTICE_TARGET_POS_A2_S [1][0] =  PRACTICE_TARGET_POSITION_A1_S [practice_PERMS[dataRec->perm][0]][0];
			PRACTICE_TARGET_POS_A2_S [0][0] =  PRACTICE_TARGET_POSITION_A1_S [practice_PERMS[dataRec->perm][1]][0];

			TARGET_POS_A1 [1][0] = TARGET_POSITION_A1_S[ PERMS[ dataRec->perm ][1]][0];
			TARGET_POS_A1 [2][0] = TARGET_POSITION_A1_S[ PERMS[ dataRec->perm ][2]][0];
			TARGET_POS_A1 [3][0] = TARGET_POSITION_A1_S[ PERMS[ dataRec->perm ][3]][0];
			TARGET_POS_A1 [4][0] = TARGET_POSITION_A1_S[ PERMS[ dataRec->perm ][4]][0];
			TARGET_POS_A1 [5][0] = TARGET_POSITION_A1_S[ PERMS[ dataRec->perm ][5]][0];
			TARGET_POS_A1 [6][0] = TARGET_POSITION_A1_S[ PERMS[ dataRec->perm ][6]][0];
			TARGET_POS_A1 [0][0] = TARGET_POSITION_A1_S[ PERMS[ dataRec->perm ][0]][0];
		


			TARGET_POS_A2 [1][0] = TARGET_POSITION_A2_S[ PERMS[ dataRec->perm ][1]][0];
			TARGET_POS_A2 [2][0] = TARGET_POSITION_A2_S[ PERMS[ dataRec->perm ][2]][0];
			TARGET_POS_A2 [3][0] = TARGET_POSITION_A2_S[ PERMS[ dataRec->perm ][3]][0];
			TARGET_POS_A2 [4][0] = TARGET_POSITION_A2_S[ PERMS[ dataRec->perm ][4]][0];
			TARGET_POS_A2 [5][0] = TARGET_POSITION_A2_S[ PERMS[ dataRec->perm ][5]][0];
			TARGET_POS_A2 [6][0] = TARGET_POSITION_A2_S[ PERMS[ dataRec->perm ][6]][0];
			TARGET_POS_A2 [0][0] = TARGET_POSITION_A2_S[ PERMS[ dataRec->perm ][0]][0];

		}
	}
	else if (dataRec->scenario == MIXED)
	{

		if(dataRec->perm == 7)
		{
			 PRACTICE_TARGET_POS_A1_M [1][0] =  (SHIFT_X + (boundary[0]-boundaryThickness) - 27 + TARGET_WIDTH /3.0); //targets at right
			PRACTICE_TARGET_POS_A1_M [0][0] =  (SHIFT_X - (boundary[0]-boundaryThickness) + 2 * TARGET_WIDTH /3.0); //targets at left

			PRACTICE_TARGET_POS_A2_M [1][0] =  (SHIFT_X + (boundary[0]-boundaryThickness) - 27 + TARGET_WIDTH /3.0); //targets at right
			PRACTICE_TARGET_POS_A2_M [0][0] =  (SHIFT_X - (boundary[0]-boundaryThickness) + 2 * TARGET_WIDTH /3.0);//targets at left

			PRACTICE_TARGET_POS_A1_M [1][2] =  (SHIFT_Z - wallDepth*0.5 -0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );//target at the bottom
			PRACTICE_TARGET_POS_A1_M [0][2] =  (SHIFT_Z);//target at the middle

			PRACTICE_TARGET_POS_A2_M [1][2] =  (SHIFT_Z - wallDepth*0.5 -0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );//target at the bottom
			PRACTICE_TARGET_POS_A2_M [0][2] =  (SHIFT_Z);//target at the middle 

		}
		else
		{
		    PRACTICE_TARGET_POS_A1_M [1][0] =  (SHIFT_X + (boundary[0]-boundaryThickness) - 27 + TARGET_WIDTH /3.0); //targets at right
			PRACTICE_TARGET_POS_A1_M [0][0] =  (SHIFT_X - (boundary[0]-boundaryThickness) + 2 * TARGET_WIDTH /3.0); //targets at left

			PRACTICE_TARGET_POS_A2_M [1][0] =  (SHIFT_X + (boundary[0]-boundaryThickness) - 27 + TARGET_WIDTH /3.0); //targets at right
			PRACTICE_TARGET_POS_A2_M [0][0] =  (SHIFT_X - (boundary[0]-boundaryThickness) + 2 * TARGET_WIDTH /3.0);//targets at left

			PRACTICE_TARGET_POS_A1_M [1][2] =  (SHIFT_Z - wallDepth*0.5 -0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );//target at the bottom
			PRACTICE_TARGET_POS_A1_M [0][2] =  (SHIFT_Z);//target at the middle

			PRACTICE_TARGET_POS_A2_M [1][2] =  (SHIFT_Z - wallDepth*0.5 -0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );//target at the bottom
			PRACTICE_TARGET_POS_A2_M [0][2] =  (SHIFT_Z);//target at the middle 


			TARGET_POS_A1 [1][0] = TARGET_POSITION_A1_M[ PERMS_MIXED[ dataRec->perm ][0]][0];
			TARGET_POS_A1 [2][0] = TARGET_POSITION_A1_M[ PERMS_MIXED[ dataRec->perm ][1]][0];
			TARGET_POS_A1 [3][0] = TARGET_POSITION_A1_M[ PERMS_MIXED[ dataRec->perm ][2]][0];
			TARGET_POS_A1 [4][0] = TARGET_POSITION_A1_M[ PERMS_MIXED[ dataRec->perm ][3]][0];
			TARGET_POS_A1 [5][0] = TARGET_POSITION_A1_M[ PERMS_MIXED[ dataRec->perm ][4]][0];
			TARGET_POS_A1 [6][0] = TARGET_POSITION_A1_M[ PERMS_MIXED[ dataRec->perm ][5]][0];
			TARGET_POS_A1 [7][0] = TARGET_POSITION_A1_M[ PERMS_MIXED[ dataRec->perm ][6]][0];
			TARGET_POS_A1 [8][0] = TARGET_POSITION_A1_M[ PERMS_MIXED[ dataRec->perm ][7]][0];
			TARGET_POS_A1 [9][0] = TARGET_POSITION_A1_M[ PERMS_MIXED[ dataRec->perm ][8]][0];
			TARGET_POS_A1 [0][0] = TARGET_POSITION_A1_M[ PERMS_MIXED[ dataRec->perm ][9]][0];
		
	        TARGET_POS_A2 [1][0] = TARGET_POSITION_A2_M[ PERMS_MIXED[ dataRec->perm ][0]][0];
			TARGET_POS_A2 [2][0] = TARGET_POSITION_A2_M[ PERMS_MIXED[ dataRec->perm ][1]][0];
			TARGET_POS_A2 [3][0] = TARGET_POSITION_A2_M[ PERMS_MIXED[ dataRec->perm ][2]][0];
			TARGET_POS_A2 [4][0] = TARGET_POSITION_A2_M[ PERMS_MIXED[ dataRec->perm ][3]][0];
			TARGET_POS_A2 [5][0] = TARGET_POSITION_A2_M[ PERMS_MIXED[ dataRec->perm ][4]][0];
			TARGET_POS_A2 [6][0] = TARGET_POSITION_A2_M[ PERMS_MIXED[ dataRec->perm ][5]][0];
			TARGET_POS_A2 [7][0] = TARGET_POSITION_A2_M[ PERMS_MIXED[ dataRec->perm ][6]][0];
			TARGET_POS_A2 [8][0] = TARGET_POSITION_A2_M[ PERMS_MIXED[ dataRec->perm ][7]][0];
			TARGET_POS_A2 [9][0] = TARGET_POSITION_A2_M[ PERMS_MIXED[ dataRec->perm ][8]][0];
			TARGET_POS_A2 [0][0] = TARGET_POSITION_A2_M[ PERMS_MIXED[ dataRec->perm ][9]][0];

			TARGET_POS_A1 [1][2] = TARGET_POSITION_A1_M[ PERMS_MIXED[ dataRec->perm ][0]][2];
			TARGET_POS_A1 [2][2] = TARGET_POSITION_A1_M[ PERMS_MIXED[ dataRec->perm ][1]][2];
			TARGET_POS_A1 [3][2] = TARGET_POSITION_A1_M[ PERMS_MIXED[ dataRec->perm ][2]][2];
			TARGET_POS_A1 [4][2] = TARGET_POSITION_A1_M[ PERMS_MIXED[ dataRec->perm ][3]][2];
			TARGET_POS_A1 [5][2] = TARGET_POSITION_A1_M[ PERMS_MIXED[ dataRec->perm ][4]][2];
			TARGET_POS_A1 [6][2] = TARGET_POSITION_A1_M[ PERMS_MIXED[ dataRec->perm ][5]][2];
			TARGET_POS_A1 [7][2] = TARGET_POSITION_A1_M[ PERMS_MIXED[ dataRec->perm ][6]][2];
			TARGET_POS_A1 [8][2] = TARGET_POSITION_A1_M[ PERMS_MIXED[ dataRec->perm ][7]][2];
			TARGET_POS_A1 [9][2] = TARGET_POSITION_A1_M[ PERMS_MIXED[ dataRec->perm ][8]][2];
			TARGET_POS_A1 [0][2] = TARGET_POSITION_A1_M[ PERMS_MIXED[ dataRec->perm ][9]][2];
		
	        TARGET_POS_A2 [1][2] = TARGET_POSITION_A2_M[ PERMS_MIXED[ dataRec->perm ][0]][2];
			TARGET_POS_A2 [2][2] = TARGET_POSITION_A2_M[ PERMS_MIXED[ dataRec->perm ][1]][2];
			TARGET_POS_A2 [3][2] = TARGET_POSITION_A2_M[ PERMS_MIXED[ dataRec->perm ][2]][2];
			TARGET_POS_A2 [4][2] = TARGET_POSITION_A2_M[ PERMS_MIXED[ dataRec->perm ][3]][2];
			TARGET_POS_A2 [5][2] = TARGET_POSITION_A2_M[ PERMS_MIXED[ dataRec->perm ][4]][2];
			TARGET_POS_A2 [6][2] = TARGET_POSITION_A2_M[ PERMS_MIXED[ dataRec->perm ][5]][2];
			TARGET_POS_A2 [7][2] = TARGET_POSITION_A2_M[ PERMS_MIXED[ dataRec->perm ][6]][2];
			TARGET_POS_A2 [8][2] = TARGET_POSITION_A2_M[ PERMS_MIXED[ dataRec->perm ][7]][2];
			TARGET_POS_A2 [9][2] = TARGET_POSITION_A2_M[ PERMS_MIXED[ dataRec->perm ][8]][2];
			TARGET_POS_A2 [0][2] = TARGET_POSITION_A2_M[ PERMS_MIXED[ dataRec->perm ][9]][2];
		}
	}


	//if (dataRec->scenario == STRAIGHT)
	//{
	//	// VERTICAL ALIGNMENT OF TARGETS -- z direction
	//	// AYSE: only fill as many elements as needed
	//	if (dataRec->cond == PARTIAL_CONFLICT) // alternating targets both at either right or left
	//	{
	//		// different target positions for odd and even trials
	//		TARGET_POS_A1 [0][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 8 + INITIAL_TARGET ); // even trial, targets at left
	//		TARGET_POS_A1 [1][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 27 + TARGET_WIDTH /3.0); // odd trial, targets at right
	//		TARGET_POS_A1 [2][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 28 + INITIAL_TARGET + TARGET_WIDTH /3.0); // even trial, targets at left
	//		TARGET_POS_A1 [3][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 8 - TARGET_WIDTH /3.0); // odd trial, targets at right
	//		TARGET_POS_A1 [4][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 22 + TARGET_WIDTH /3.0);
	//		TARGET_POS_A1 [5][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 38 - TARGET_WIDTH /3.0); // odd trial, targets at right


	//		TARGET_POS_A2 [0][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 4 + TARGET_WIDTH /3.0); // even trial, targets at left
	//		TARGET_POS_A2 [1][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 18 - INITIAL_TARGET + TARGET_WIDTH /3.0); // odd trial, targets at right
	//		TARGET_POS_A2 [2][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 22 + TARGET_WIDTH /3.0); // even trial, targets at left
	//		TARGET_POS_A2 [3][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 12 - INITIAL_TARGET ); // odd trial, targets at right
	//		TARGET_POS_A2 [4][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 28 + INITIAL_TARGET + TARGET_WIDTH /3.0); // even trial, targets at left
	//		TARGET_POS_A2 [5][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 28 - INITIAL_TARGET ); // odd trial, targets at right
	//	}
	//	else if (dataRec->cond == FULL_CONFLICT) // alternating targets at opposite ends, followed by a target at middle
	//	{
	//		// target positions at opposite corners for odd trials
	//		// at the middle for even trials

	//		TARGET_POS_A1 [0][0] = (SHIFT_X); // even trial, target at middle
	//		TARGET_POS_A1 [1][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 2 * TARGET_WIDTH /3.0); // even trial, target at left
	//		TARGET_POS_A1 [2][0] = (SHIFT_X); // even trial, target at middle
	//		TARGET_POS_A1 [3][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 2 * TARGET_WIDTH /3.0); // even trial, target at right

	//		TARGET_POS_A2 [0][0] = (SHIFT_X); // odd trial, target at middle
	//		TARGET_POS_A2 [1][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 2 * TARGET_WIDTH /3.0); // even trial, target at left
	//		TARGET_POS_A2 [2][0] = (SHIFT_X); // odd trial, target at middle
	//		TARGET_POS_A2 [3][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 2 * TARGET_WIDTH /3.0); // even trial, target at right
	//	}
	//	else if (dataRec->cond == SINGLE_BLIND_A1) // only one target for agent 1
	//	{
	//		// different target positions for odd and even trials
	//		TARGET_POS_A1 [0][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 8 + INITIAL_TARGET ); // even trial, targets at left
	//		TARGET_POS_A1 [1][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 27 + TARGET_WIDTH /3.0); // odd trial, targets at right
	//		TARGET_POS_A1 [2][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 28 + INITIAL_TARGET + TARGET_WIDTH /3.0); // even trial, targets at left
	//		TARGET_POS_A1 [3][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 8 - TARGET_WIDTH /3.0); // odd trial, targets at right
	//		TARGET_POS_A1 [4][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 22 + TARGET_WIDTH /3.0);
	//		TARGET_POS_A1 [5][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 38 - TARGET_WIDTH /3.0); 

	//		TARGET_POS_A2 [0][0] = -10000; // don't show target
	//		TARGET_POS_A2 [1][0] = -10000; // don't show target
	//		TARGET_POS_A2 [2][0] = -10000; // don't show target
	//		TARGET_POS_A2 [3][0] = -10000; // don't show target
	//		TARGET_POS_A2 [4][0] = -10000; // don't show target
	//		TARGET_POS_A2 [5][0] = -10000; // don't show target
	//	}
	//	else if (dataRec->cond == SINGLE_BLIND_A2) // only one target for agent 2
	//	{
	//		TARGET_POS_A1 [0][0] = -10000; // don't show target
	//		TARGET_POS_A1 [1][0] = -10000; // don't show target
	//		TARGET_POS_A1 [2][0] = -10000; // don't show target
	//		TARGET_POS_A1 [3][0] = -10000; // don't show target
	//		TARGET_POS_A1 [4][0] = -10000; // don't show target
	//		TARGET_POS_A1 [5][0] = -10000; // don't show target


	//		TARGET_POS_A2 [0][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 8 + INITIAL_TARGET ); // even trial, targets at left
	//		TARGET_POS_A2 [1][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 27 + TARGET_WIDTH /3.0); // odd trial, targets at right
	//		TARGET_POS_A2 [2][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 28 + INITIAL_TARGET + TARGET_WIDTH /3.0); // even trial, targets at left
	//		TARGET_POS_A2 [3][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 8 - TARGET_WIDTH /3.0); // odd trial, targets at right
	//		TARGET_POS_A2 [4][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 22 + TARGET_WIDTH /3.0);
	//		TARGET_POS_A2 [5][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 38 - TARGET_WIDTH /3.0); 


	//	}
	//	else if (dataRec->cond == PRACTICE || dataRec->cond == HARMONY) // show the same targets to both agents
	//	{
	//		TARGET_POS_A1 [0][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 2 * TARGET_WIDTH /3.0); // target at left
	//		TARGET_POS_A1 [1][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 2 * TARGET_WIDTH /3.0); // target at righ
	//		TARGET_POS_A1 [2][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 2 * TARGET_WIDTH /3.0); // target at left
	//		TARGET_POS_A1 [3][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 2 * TARGET_WIDTH /3.0); // target at righ

	//		TARGET_POS_A2 [0][0] = TARGET_POS_A1 [0][0]; 
	//		TARGET_POS_A2 [1][0] = TARGET_POS_A1 [1][0]; 
	//		TARGET_POS_A2 [2][0] = TARGET_POS_A1 [2][0]; 
	//		TARGET_POS_A2 [3][0] = TARGET_POS_A1 [3][0]; 
	//	}
	//}
	//else if (dataRec->scenario == ROTATIONAL)
	//{
	//	if (dataRec->cond == PARTIAL_CONFLICT)
	//	{
	//		TARGET_ANGLE_A1[0] = (22 * PI) / 180;
	//		TARGET_ANGLE_A1[1] = (-45 * PI) / 180;

	//		TARGET_ANGLE_A2[0] = ( 45 * PI) / 180;
	//		TARGET_ANGLE_A2[1] = (-22 * PI) / 180;
	//	}
	//	else if (dataRec->cond == FULL_CONFLICT) 
	//	{
	//		TARGET_ANGLE_A1[0] = 0; // even trial, target at middle
	//		TARGET_ANGLE_A1[1] = (-43 * PI) / 180; // even trial, target at left
	//		TARGET_ANGLE_A1[2] = 0; // even trial, target at middle
	//		TARGET_ANGLE_A1[3] = (43 * PI) / 180; // even trial, target at right

	//		TARGET_ANGLE_A2[0] = 0; // even trial, target at middle
	//		TARGET_ANGLE_A2[1] = (43 * PI) / 180; // even trial, target at left
	//		TARGET_ANGLE_A2[2] = 0; // even trial, target at middle
	//		TARGET_ANGLE_A2[3] = (-43 * PI) / 180; // even trial, target at right

	//	}
	//	else if (dataRec->cond == SINGLE_BLIND_A1) // only one target for agent 1
	//	{
	//		TARGET_ANGLE_A1[0] = (40 * PI) / 180;
	//		TARGET_ANGLE_A1[1] = (-40 * PI) / 180;

	//		TARGET_POS_A2 [0][0] = -10000; // don't show target
	//		TARGET_POS_A2 [1][0] = -10000; // don't show target
	//	}
	//	else if (dataRec->cond == SINGLE_BLIND_A2) // only one target for agent 2
	//	{
	//		TARGET_POS_A1 [0][0] = -10000; // don't show target
	//		TARGET_POS_A1 [1][0] = -10000; // don't show target

	//		TARGET_ANGLE_A2[0] = (40 * PI) / 180;
	//		TARGET_ANGLE_A2[1] = (-40 * PI) / 180;

	//	}
	//	else if (dataRec->cond == PRACTICE || dataRec->cond == HARMONY) // show the same targets to both agents
	//	{
	//		TARGET_ANGLE_A1 [0] = (40 * PI) / 180;
	//		TARGET_ANGLE_A1 [1] = (-40 * PI) / 180;

	//		TARGET_ANGLE_A2[0] = TARGET_ANGLE_A1[0]; 
	//		TARGET_ANGLE_A2[1] = TARGET_ANGLE_A1[1]; 
	//	}
	//}
	//else if (dataRec->scenario == MIXED)
	//{
	//	// VERTICAL ALIGNMENT OF TARGETS -- z direction
	//	// AYSE: only fill as many elements as needed
	//	if (dataRec->cond == PARTIAL_CONFLICT) // alternating targets both at either right or left
	//	{
	//		// different target positions for odd and even trials
	//		TARGET_POS_A1 [0][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 2 * TARGET_WIDTH /3.0); // even trial, targets at left
	//		TARGET_POS_A1 [1][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 27 + TARGET_WIDTH /3.0); // odd trial, targets at right
	//		TARGET_POS_A1 [2][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 2 * TARGET_WIDTH /3.0);
	//		TARGET_POS_A1 [3][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 8 - TARGET_WIDTH /3.0); // odd trial, targets at right
	//		TARGET_POS_A1 [4][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 2 * TARGET_WIDTH /3.0);
	//		TARGET_POS_A1 [5][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 38 - TARGET_WIDTH /3.0); // odd trial, targets at right

	//		TARGET_POS_A1 [0][2] = (SHIFT_Z );
	//		TARGET_POS_A1 [1][2] =  (SHIFT_Z - wallDepth*0.5 -0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );//- 0.25*(BOARD_WIDTH - wallDepth) );
	//		TARGET_POS_A1 [2][2] = (SHIFT_Z);
	//		TARGET_POS_A1 [3][2] = (SHIFT_Z + wallDepth*0.5 +0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );//- 0.25*(BOARD_WIDTH - wallDepth) );
	//		TARGET_POS_A1 [4][2] = SHIFT_Z;// odd trial, targets at right
	//		TARGET_POS_A1 [5][2] =  (SHIFT_Z - wallDepth*0.5 -0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );//- 0.25*(BOARD_WIDTH - wallDepth) );


	//		TARGET_POS_A2 [0][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 2 * TARGET_WIDTH /3.0);
	//		TARGET_POS_A2 [1][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 18 - INITIAL_TARGET + TARGET_WIDTH /3.0); // odd trial, targets at right
	//		TARGET_POS_A2 [2][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 2 * TARGET_WIDTH /3.0);
	//		TARGET_POS_A2 [3][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 12 - INITIAL_TARGET ); // odd trial, targets at right
	//		TARGET_POS_A2 [4][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 2 * TARGET_WIDTH /3.0);
	//		TARGET_POS_A2 [5][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 28 - INITIAL_TARGET ); // odd trial, targets at right

	//		TARGET_POS_A2 [0][2] = (SHIFT_Z );
	//		TARGET_POS_A2 [1][2] =  (SHIFT_Z - wallDepth*0.5 -0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );//- 0.25*(BOARD_WIDTH - wallDepth) );
	//		TARGET_POS_A2 [2][2] = (SHIFT_Z);
	//		TARGET_POS_A2 [3][2] = (SHIFT_Z + wallDepth*0.5 +0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );//- 0.25*(BOARD_WIDTH - wallDepth) );
	//		TARGET_POS_A2 [4][2] = SHIFT_Z;// odd trial, targets at right
	//		TARGET_POS_A2 [5][2] =  (SHIFT_Z - wallDepth*0.5 -0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );//- 0.25*(BOARD_WIDTH - wallDepth) );
	//	}
	//	else if (dataRec->cond == FULL_CONFLICT) // alternating targets at opposite ends, followed by a target at middle
	//	{
	//		// target positions at opposite corners for odd trials
	//		// at the middle for even trials

	//		TARGET_POS_A1 [0][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 2 * TARGET_WIDTH /3.0);
	//		TARGET_POS_A1 [1][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 2 * TARGET_WIDTH /3.0); // even trial, target at left
	//		TARGET_POS_A1 [2][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 2 * TARGET_WIDTH /3.0);
	//		TARGET_POS_A1 [3][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 2 * TARGET_WIDTH /3.0); // even trial, target at right

	//		TARGET_POS_A1 [0][2] = (SHIFT_Z);//+ 0.5*(BOARD_WIDTH*0.5 - wallWidth*0.5);//
	//		TARGET_POS_A1 [1][2] =(SHIFT_Z - wallDepth*0.5 -0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );//- 0.25*(BOARD_WIDTH - wallDepth) );
	//		TARGET_POS_A1 [2][2] = SHIFT_Z;//+ 0.5*(BOARD_WIDTH*0.5 - wallWidth*0.5);//
	//		TARGET_POS_A1 [3][2] = (SHIFT_Z + wallDepth*0.5 +0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );//- 0.25*(BOARD_WIDTH - wallDepth) );



	//		TARGET_POS_A2 [0][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 2 * TARGET_WIDTH /3.0);
	//		TARGET_POS_A2 [1][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 2 * TARGET_WIDTH /3.0); // even trial, target at left
	//		TARGET_POS_A2 [2][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 2 * TARGET_WIDTH /3.0);
	//		TARGET_POS_A2 [3][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 2 * TARGET_WIDTH /3.0);

	//		TARGET_POS_A2 [0][2] = (SHIFT_Z);//+ 0.5*(BOARD_WIDTH*0.5 - wallWidth*0.5);//
	//		TARGET_POS_A2 [1][2] =(SHIFT_Z + wallDepth*0.5 + 0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );//- 0.25*(BOARD_WIDTH - wallDepth) );
	//		TARGET_POS_A2 [2][2] = SHIFT_Z;//+ 0.5*(BOARD_WIDTH*0.5 - wallWidth*0.5);//
	//		TARGET_POS_A2 [3][2] = (SHIFT_Z - wallDepth*0.5 - 0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );//- 0.25*(BOARD_WIDTH - wallDepth) );

	//	}
	//	else if (dataRec->cond == SINGLE_BLIND_A1) // only one target for agent 1
	//	{
	//		// different target positions for odd and even trials
	//		TARGET_POS_A1 [0][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + TARGET_WIDTH /1.5);
	//		TARGET_POS_A1 [1][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 27 + TARGET_WIDTH /3.0); // odd trial, targets at right
	//		TARGET_POS_A1 [2][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + TARGET_WIDTH /1.5); // even trial, targets at left
	//		TARGET_POS_A1 [3][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 8 - TARGET_WIDTH /3.0); // odd trial, targets at right
	//		TARGET_POS_A1 [4][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + TARGET_WIDTH /1.5);
	//		TARGET_POS_A1 [5][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 38 - TARGET_WIDTH /3.0); 

	//		TARGET_POS_A1 [0][2] = SHIFT_Z;// - wallDepth*0.5 -0.5*(new_depth_mixed*0.5 - wallDepth*0.5) );//- 0.25*(BOARD_WIDTH - wallDepth) );
	//		TARGET_POS_A1 [1][2] = (SHIFT_Z - wallDepth*0.5 -0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );//- 0.25*(BOARD_WIDTH - wallDepth) );
	//		TARGET_POS_A1 [2][2] = (SHIFT_Z );
	//		TARGET_POS_A1 [3][2] = (SHIFT_Z + wallDepth*0.5 +0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );//- 0.25*(BOARD_WIDTH - wallDepth) );
	//		TARGET_POS_A1 [4][2] = (SHIFT_Z);
	//		TARGET_POS_A1 [5][2] = (SHIFT_Z - wallDepth*0.5 -0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );//- 0.25*(BOARD_WIDTH - wallDepth) );


	//		TARGET_POS_A2 [0][0] = -10000; // don't show target
	//		TARGET_POS_A2 [1][0] = -10000; // don't show target
	//		TARGET_POS_A2 [2][0] = -10000; // don't show target
	//		TARGET_POS_A2 [3][0] = -10000; // don't show target
	//		TARGET_POS_A2 [4][0] = -10000; // don't show target
	//		TARGET_POS_A2 [5][0] = -10000; // don't show target


	//	}
	//	else if (dataRec->cond == SINGLE_BLIND_A2) // only one target for agent 2
	//	{
	//		TARGET_POS_A1 [0][0] = -10000; // don't show target
	//		TARGET_POS_A1 [1][0] = -10000; // don't show target
	//		TARGET_POS_A1 [2][0] = -10000; // don't show target
	//		TARGET_POS_A1 [3][0] = -10000; // don't show target
	//		TARGET_POS_A1 [4][0] = -10000; // don't show target
	//		TARGET_POS_A1 [5][0] = -10000; // don't show target


	//		TARGET_POS_A2 [0][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + TARGET_WIDTH /1.5);
	//		TARGET_POS_A2 [1][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 27 + TARGET_WIDTH /3.0); // odd trial, targets at right
	//		TARGET_POS_A2 [2][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + TARGET_WIDTH /1.5); // even trial, targets at left
	//		TARGET_POS_A2 [3][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 8 - TARGET_WIDTH /3.0); // odd trial, targets at right
	//		TARGET_POS_A2 [4][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + TARGET_WIDTH /1.5);
	//		TARGET_POS_A2 [5][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 38 - TARGET_WIDTH /3.0); 

	//		TARGET_POS_A2 [0][2] = SHIFT_Z;// - wallDepth*0.5 -0.5*(new_depth_mixed*0.5 - wallDepth*0.5) );//- 0.25*(BOARD_WIDTH - wallDepth) );
	//		TARGET_POS_A2 [1][2] = (SHIFT_Z - wallDepth*0.5 -0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );//- 0.25*(BOARD_WIDTH - wallDepth) );
	//		TARGET_POS_A2 [2][2] = (SHIFT_Z );
	//		TARGET_POS_A2 [3][2] = (SHIFT_Z + wallDepth*0.5 +0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );//- 0.25*(BOARD_WIDTH - wallDepth) );
	//		TARGET_POS_A2 [4][2] = (SHIFT_Z);
	//		TARGET_POS_A2 [5][2] = (SHIFT_Z - wallDepth*0.5 -0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );//- 0.25*(BOARD_WIDTH - wallDepth) );


	//	}
	//	else if (dataRec->cond == PRACTICE || dataRec->cond == HARMONY) // show the same targets to both agents
	//	{
	//		TARGET_POS_A1 [0][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 2 * TARGET_WIDTH /3.0); // target at left
	//		TARGET_POS_A1 [1][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 2 * TARGET_WIDTH /3.0); // target at righ
	//		TARGET_POS_A1 [2][0] = (SHIFT_X - (boundary[0]-boundaryThickness) + 2 * TARGET_WIDTH /3.0); // target at left
	//		TARGET_POS_A1 [3][0] = (SHIFT_X + (boundary[0]-boundaryThickness) - 2 * TARGET_WIDTH /3.0); // target at righ

	//		TARGET_POS_A1 [0][2] = (SHIFT_Z);// - wallDepth*0.5 -0.5*(new_depth_mixed*0.5 - wallDepth*0.5) );//- 0.25*(BOARD_WIDTH - wallDepth) );
	//		TARGET_POS_A1 [1][2] = (SHIFT_Z - wallDepth*0.5 -0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );//- 0.25*(BOARD_WIDTH - wallDepth) );
	//		TARGET_POS_A1 [2][2] = (SHIFT_Z);// - wallDepth*0.5 -0.5*(new_depth_mixed*0.5 - wallDepth*0.5) );//- 0.25*(BOARD_WIDTH - wallDepth) );
	//		TARGET_POS_A1 [3][2] = (SHIFT_Z + wallDepth*0.5 +0.5*(BOARD_DEPTH_MIXED*0.5 - wallDepth*0.5) );//- 0.25*(BOARD_WIDTH - wallDepth) );



	//		TARGET_POS_A2 [0][0] = TARGET_POS_A1 [0][0]; 
	//		TARGET_POS_A2 [1][0] = TARGET_POS_A1 [1][0]; 
	//		TARGET_POS_A2 [2][0] = TARGET_POS_A1 [2][0]; 
	//		TARGET_POS_A2 [3][0] = TARGET_POS_A1 [3][0]; 

	//		TARGET_POS_A2 [0][2] = TARGET_POS_A1 [0][2]; 
	//		TARGET_POS_A2 [1][2] = TARGET_POS_A1 [1][2];
	//		TARGET_POS_A2 [2][2] = TARGET_POS_A1 [2][2]; 
	//		TARGET_POS_A2 [3][2] = TARGET_POS_A1 [3][2]; 


	//	}
	//}




	// Create the effect 
	effect = new HapticCallBack;
	if (dataRec->cond == PRACTICE)
	{
		effect->hipsOn = true;
	}
	SoundPlayerThread::hapticCb = effect;
	SoSeparator* root = graphCtrller->getRoot1();
	GenerateDefaultGraphics(root);

	InitialSetup();

	SoWinExaminerViewer *myViewer = new SoWinExaminerViewer(myWindow);
	myViewer->setAnimationEnabled(true);
	myViewer->setSceneGraph(graphCtrller->getRoot1());

	// reorient the camera so that we see the whole board from above
	SoCamera *myCamera = myViewer->getCamera();
	myCamera->position.setValue(SHIFT_X, 160, 1 + SHIFT_Z/*BOARD_WIDTH+20/* 100, 200*/);//100
	myCamera->pointAt(SbVec3f(SHIFT_X, -125, -1 + SHIFT_Z/*-20/* -100, -200*/));//-38
	myViewer->saveHomePosition();
	myViewer->setTitle("Yeditepe University - Maze Game - Player 1");
	myViewer->setSize(SbVec2s(screenWidth/*2560*/, screenHeight));
	myViewer->setDecoration(FALSE);
	myViewer->setFullScreen(FALSE);

	myViewer->setBackgroundColor(SbColor(0,0,0));

	myViewer->setWindowCloseCallback(MyHandleClose, graphCtrller->getRoot1());
	myViewer->show();
	SoWinExaminerViewer *myViewer2 = new SoWinExaminerViewer(myWindow2);
	myViewer2->setAnimationEnabled(true);	
	myViewer2->setSceneGraph(graphCtrller->getRoot2());

	// reorient the camera so that we see the whole board from above
	SoCamera *myCamera2 = myViewer2->getCamera();
	myCamera2->position.setValue(SHIFT_X, 160, 1+SHIFT_Z/*BOARD_WIDTH+20/* 100, 200*/);
	myCamera2->pointAt(SbVec3f(SHIFT_X, -125, -1+SHIFT_Z/*-20/* -100, -200*/));
	myViewer2->saveHomePosition();
	myViewer2->setTitle("Yeditepe University - Maze Game - Player 2");
	myViewer2->setSize(SbVec2s(screenWidth/*2560*/, screenHeight));
	myViewer2->setDecoration(FALSE);
	myViewer2->setFullScreen(FALSE);//TRUE);

	myViewer2->setBackgroundColor(SbColor(0,0,0));
	myViewer2->setWindowCloseCallback(MyHandleClose, root);
	myViewer2->show();

	SoWin::show(myWindow);
	SoWin::show(myWindow2);
	SoWin::mainLoop();
}

void GenerateDefaultGraphics(SoSeparator * root)
{
	// initialize the servo loop (key-press event)
	// An event callback node so we can receive key press events
	SoEventCallback *myEventCB = new SoEventCallback;
	myEventCB->addEventCallback(SoKeyboardEvent::getClassTypeId(), myKeyPressCB, root);
	root->addChild(myEventCB);

	initScreenText();

	boardRotateX = new SoTransform;
	boardRotateZ = new SoTransform;

	//root->addChild(boardRotateX);
	//root->addChild(boardRotateZ);

	graphCtrller->addInterfacePoints(); 


	// add the ball
	//Onur
	graphCtrller->addBall();
	graphCtrller->addBoard();

	
	//Onur 
	/*
	if (firstTime) {

		double Xsrc_max = 82;	//Onur Code x axis max up-down 
		double Xsrc_min = 0;	//Onur Code x axis min up-down
		double Ysrc_max = 200;	//Onur Code y axis max left-right
		double Ysrc_min = 0;	//Onur Code y axis min left-right



		double Xres_min = -100; //MazeGame x axis min left-right 
		double Xres_max = +100;	//MazeGame x axis max left-right
		double Zres_min = -41;	//MazeGame z axis min up-down
		double Zres_max = +41;	//MazeGame z axis max up-down

		for (int i = 0; i < path.dtargetsX.size(); i++) {
			double res_z = ((path.dtargetsX.at(i) - Xsrc_min) / (Xsrc_max - Xsrc_min) * (Zres_max - Zres_min) + Zres_min);
			double res_x = ((path.dtargetsY.at(i) - Ysrc_min) / (Ysrc_max - Ysrc_min) * (Xres_max - Xres_min) + Xres_min);
			
			cout << res_x << "\t" << res_z << "\n";
			graphCtrller->addBall(res_x,res_z);
		}
	}
	
	*/

	//Onur
		

	if(dataRec->scenario == MIXED)
		graphCtrller->addWall();

	if (effect->hipsOn)
		graphCtrller->addSpring();

	// Set up the timer callback to update the stylus
	SoTimerSensor *graphicUpdate = new SoTimerSensor(graphicsTimerCallback, root);
	graphicUpdate->setInterval(1.0/40.0);
	graphicUpdate->schedule();
}

// handle key-press event here
void myKeyPressCB(void *userData, SoEventCallback *eventCB)
{

	const SoEvent *event = eventCB->getEvent();

	if (SO_KEY_PRESS_EVENT(event, UP_ARROW)) 
	{
		////if (server->clientConnected())
		////{

		//// initialize phantom servo loop 
		if(effect->keepHapticLoop == false) 
		{
			HDErrorInfo error;
			effect->keepHapticLoop = true;

			effect->initialize_phantom(effect->keepHapticLoop);
			printf("Game started!\n");

			// begin frame at the beginning of each servo loop
			callbackHandlers.push_back( hdScheduleAsynchronous(BeginFrameCallback, (void*)0, HD_MAX_SCHEDULER_PRIORITY) );
			// end frame at the end of each servo loop
			callbackHandlers.push_back( hdScheduleAsynchronous(EndFrameCallback, (void*)0, HD_MIN_SCHEDULER_PRIORITY) );

			//callbackHandlers.push_back( hdScheduleAsynchronous(ComputeForceCallback, computedForces, HD_DEFAULT_SCHEDULER_PRIORITY) );
			callbackHandlers.push_back( hdScheduleAsynchronous(MyHapticLoop, (void*)0, HD_DEFAULT_SCHEDULER_PRIORITY) );
			//callbackHandlers.push_back( hdScheduleAsynchronous(DutyCycleCallback, (void*)0, HD_MIN_SCHEDULER_PRIORITY) );

			hdSetSchedulerRate(SCHEDULER_RATE);
			hdStartScheduler();


			SoundPlayerThread::run();

			if (HD_DEVICE_ERROR(error = hdGetError()))
			{
				//hduPrintError(stderr, &error, "Failed to start the scheduler");
				printf("Failed to start the scheduler");
				exit(0);
			}
		}
		//	}

	}
	else if (SO_KEY_PRESS_EVENT(event, DOWN_ARROW)) 
	{
		// stop phantom servo loop 
		if( effect->keepHapticLoop == true )	
		{
			effect->keepHapticLoop = false;
			printf("haptic is stopped\n");

			//Sleep(100);
			effect->stopHaptics();
			exit(0);
		}
	}
	else if (SO_KEY_PRESS_EVENT(event, F2))
	{
		effect->soundOn = !(effect->soundOn);
		//cout << "soundOn " << effect->soundOn << endl;
	}
	else if (SO_KEY_PRESS_EVENT(event, F5))
	{
		effect->tiltOn = !(effect->tiltOn);
		//cout << "tiltOn " << effect->tiltOn << endl;
	}
	else if (SO_KEY_PRESS_EVENT(event, F6))
	{
		effect->hipsOn = !(effect->hipsOn);
		//cout << "hipsOn " << effect->hipsOn << endl;
	}
	else if (SO_KEY_PRESS_EVENT(event, F8))
	{
		effect->scoreTextOn = !(effect->scoreTextOn);
		//cout << "scoreTextOn " << scoreTextOn << endl;
	}

	else if (SO_KEY_PRESS_EVENT(event, W))
	{
		effect->warningOn = !(effect->warningOn);
		//cout << "warningOn " << warningOn << endl;
	}
	else if (SO_KEY_PRESS_EVENT(event, V))
	{
		effect->vectorOn = !(effect->vectorOn);
		//cout << "hipsOn " << effect->hipsOn << endl;
	}
	else if (SO_KEY_PRESS_EVENT(event, X))
	{
		effect->RvectorOn = !(effect->RvectorOn);
		//cout << "hipsOn " << effect->hipsOn << endl;
	}


	eventCB->setHandled();
}

// handle the window close message
void MyHandleClose(void *,class SoWinComponent *)
{
	//turn off phantom if it is on
	if(effect->keepHapticLoop ==true)
	{
		effect->keepHapticLoop = false;
		printf("handle closed\n");

		//Sleep(100);
		effect->stopHaptics();
		exit(0);
	}
	//exit(0);
}

void graphicsTimerCallback(void *data, SoSensor *)
{
	// WARNING : all data should be synchronized !!
	// this is where all painting should be done...
	// you will see screen stalls if you dont do so...
	// get the stylus from haptic loop

	

	if(myStylusFlag == true)
	{
		// AYSE: warning text in the middle of screen
		messageText->string.setValue(messageString.c_str());


		float bpX, bpY, bpZ, cpX, cpY, cpZ, npX, npY, npZ, hpX, hpY, hpZ,alpha;
		//alpha=0;

		float bpX2, bpY2, bpZ2,alpha2;
		float hhpX, hhpY, hhpZ,hcpX, hcpY, hcpZ;

		alpha=graphCtrller->ballGr->getAngleBallGr();
		graphCtrller->ballGr->getPosition().getValue(bpX, bpY, bpZ);
		graphCtrller->cip.getPosition().getValue(cpX, cpY, cpZ);
		graphCtrller->hip.getPosition().getValue(hpX, hpY, hpZ);

		graphCtrller->nip.getPosition().getValue(npX, npY, npZ);
		graphCtrller->handleH.getPosition().getValue(hhpX, hhpY, hhpZ);
		graphCtrller->handleC.getPosition().getValue(hcpX, hcpY, hcpZ);

		//alpha=graphCtrller->ballGr->ggetAngleBallGr();
		graphCtrller->ballGr->setTranslate(Vector(bpX, bpY, bpZ));
		graphCtrller->ballGr2->setTranslate(Vector(bpX, bpY, bpZ));

		//graphCtrller->boardGr->setColor(colorBoundary);

		if((graphCtrller->boardGr->getInit()))
		{
			if (effect->hipsOn)
			{
				graphCtrller->cip.setRadius(IP_RADIUS);
				graphCtrller->hip.setRadius(IP_RADIUS);

				graphCtrller->cip2.setRadius(IP_RADIUS);
				graphCtrller->hip2.setRadius(IP_RADIUS);
			}

			graphCtrller->nip.setRadius(NIP_RADIUS);
			graphCtrller->handleH.setRadius(IP_RADIUS);//NIP_RADIUS);
			graphCtrller->handleC.setRadius(IP_RADIUS);//NIP_RADIUS);
			graphCtrller->handleH2.setRadius(IP_RADIUS);//NIP_RADIUS);
			graphCtrller->handleC2.setRadius(IP_RADIUS);//NIP_RADIUS);

			graphCtrller->ballGr->shape->depth.setValue(graphCtrller->ballGr->getDepth());
			graphCtrller->ballGr->shape->height.setValue(graphCtrller->ballGr->getHeight());
			graphCtrller->ballGr->shape->width.setValue(graphCtrller->ballGr->getWidth());
			graphCtrller->ballGr->setHandleRadius(IP_RADIUS);
			graphCtrller->ballGr2->setDepth(graphCtrller->ballGr2->getDepth());
			graphCtrller->ballGr2->setWidth(graphCtrller->ballGr2->getWidth());
			graphCtrller->ballGr2->setHeight(graphCtrller->ballGr2->getHeight());
			graphCtrller->ballGr2->setHandleRadius(IP_RADIUS);
			graphCtrller->boardGr->boardMat->diffuseColor.setValue(COLOR_BOARD);
			graphCtrller->boardGr->targetMat->diffuseColor.setValue(COLOR_GREEN);
			//graphCtrller->boardGr->targetMat2->diffuseColor.setValue(COLOR_GREEN);
			graphCtrller->boardGr->boardBoundaryMat->diffuseColor.setValue(COLOR_BOARD_BOUNDARY);

			graphCtrller->boardGr2->boardMat->diffuseColor.setValue(COLOR_BOARD);
			graphCtrller->boardGr2->targetMat->diffuseColor.setValue(COLOR_GREEN);
			//graphCtrller->boardGr2->targetMat2->diffuseColor.setValue(COLOR_GREEN);
			graphCtrller->boardGr2->boardBoundaryMat->diffuseColor.setValue(COLOR_BOARD_BOUNDARY);
			graphCtrller->boardGr->ObstacleMat->diffuseColor.setValue(COLOR_BOARD_BOUNDARY);
			graphCtrller->boardGr2->ObstacleMat->diffuseColor.setValue(COLOR_BOARD_BOUNDARY);
			graphCtrller->wall1->wallMat->diffuseColor.setValue(COLOR_BOARD_BOUNDARY);
			graphCtrller->wall2->wallMat->diffuseColor.setValue(COLOR_BOARD_BOUNDARY);
			graphCtrller->wall3->wallMat->diffuseColor.setValue(COLOR_BOARD_BOUNDARY);
			graphCtrller->wall1->wallMat->transparency.setValue(0.0f);
			graphCtrller->wall2->wallMat->transparency.setValue(0.0f);
			graphCtrller->wall3->wallMat->transparency.setValue(0.0f);

		}
		else 
		{
			// don't draw the sphere for hip
			graphCtrller->boardGr->boardBoundaryMat->diffuseColor.setValue(Vector(0.0, 0.0, 0.0));

			graphCtrller->cip.setRadius(0);
			graphCtrller->hip.setRadius(0);
			graphCtrller->cip2.setRadius(0);
			graphCtrller->hip2.setRadius(0);
			graphCtrller->nip.setRadius(0);
			graphCtrller->handleH.setRadius(0);
			graphCtrller->handleC.setRadius(0);
			//graphCtrller->ballGr->setDepth(0.0);
			//graphCtrller->ballGr->setWidth(0.0);
			//graphCtrller->ballGr->setHeight(0.0);
			graphCtrller->ballGr->shape->width.setValue(0.0);
			graphCtrller->ballGr->shape->depth.setValue(0.0);
			graphCtrller->ballGr->shape->height.setValue(0.0);

			graphCtrller->ballGr->setHandleRadius(0.0f);
			graphCtrller->ballGr2->setDepth(0.0);
			graphCtrller->ballGr2->setWidth(0.0);
			graphCtrller->ballGr2->setHeight(0.0);
			graphCtrller->ballGr2->setHandleRadius(0.0f);
			graphCtrller->boardGr->boardMat->diffuseColor.setValue(Vector(0.0, 0.0, 0.0));
			graphCtrller->boardGr->targetMat->diffuseColor.setValue(Vector(0.0, 0.0, 0.0));
			//graphCtrller->boardGr->targetMat2->diffuseColor.setValue(Vector(0.0, 0.0, 0.0));

			graphCtrller->boardGr2->boardMat->diffuseColor.setValue(Vector(0.0, 0.0, 0.0));
			graphCtrller->boardGr2->targetMat->diffuseColor.setValue(Vector(0.0, 0.0, 0.0));
			//graphCtrller->boardGr2->targetMat2->diffuseColor.setValue(Vector(0.0, 0.0, 0.0));
			graphCtrller->boardGr2->boardBoundaryMat->diffuseColor.setValue(Vector(0.0, 0.0, 0.0));
			graphCtrller->boardGr2->ObstacleMat->diffuseColor.setValue(Vector(0.0, 0.0, 0.0));
			graphCtrller->boardGr->ObstacleMat->diffuseColor.setValue(Vector(0.0, 0.0, 0.0));
			graphCtrller->wall1->wallMat->diffuseColor.setValue(Vector(0.0, 0.0, 0.0));
			graphCtrller->wall2->wallMat->diffuseColor.setValue(Vector(0.0, 0.0, 0.0));
			graphCtrller->wall3->wallMat->diffuseColor.setValue(Vector(0.0, 0.0, 0.0));

		}
#ifdef ROTATION_ON


		//if ( !graphCtrller->ballGr->ball->isAngleCollide() )
		//{
		graphCtrller->ballGr->setRotation(alpha);
		graphCtrller->ballGr2->setRotation(alpha);
		//}


#endif
		bool isHitAnyWall = (graphCtrller->wall1->isHit() || graphCtrller->wall2->isHit() || graphCtrller->wall3->isHit());
		//cout<<isHitAnyWall<<"  "<<graphCtrller->wall1->isHit()<<"  "<<  graphCtrller->wall2->isHit()<<"   "<<graphCtrller->wall3->isHit()<< endl;
		if (graphCtrller->boardGr->getInit())
		{
			if(graphCtrller->boardGr->isHit())
			{
				graphCtrller->boardGr->boardBoundaryMat->diffuseColor.setValue(Vector(1.0, 0.0, 0.0));
				graphCtrller->boardGr2->boardBoundaryMat->diffuseColor.setValue(Vector(1.0, 0.0, 0.0));

				if(dataRec->scenario == ROTATIONAL)
				{
					graphCtrller->boardGr->ObstacleMat->diffuseColor.setValue(Vector(1.0, 0.0, 0.0));
					graphCtrller->boardGr2->ObstacleMat->diffuseColor.setValue(Vector(1.0, 0.0, 0.0));
				}
				else if(dataRec->scenario == MIXED)
				{
					graphCtrller->wall1->wallMat->diffuseColor.setValue(Vector(1.0, 0.0, 0.0));
					graphCtrller->wall2->wallMat->diffuseColor.setValue(Vector(1.0, 0.0, 0.0));
					graphCtrller->wall3->wallMat->diffuseColor.setValue(Vector(1.0, 0.0, 0.0));
				}
			}


			if (isHitAnyWall)
			{
				graphCtrller->wall1->wallMat->diffuseColor.setValue(Vector(1.0, 0.0, 0.0));
				graphCtrller->wall2->wallMat->diffuseColor.setValue(Vector(1.0, 0.0, 0.0));
				graphCtrller->wall3->wallMat->diffuseColor.setValue(Vector(1.0, 0.0, 0.0));
				graphCtrller->boardGr->boardBoundaryMat->diffuseColor.setValue(Vector(1.0, 0.0, 0.0));
				graphCtrller->boardGr2->boardBoundaryMat->diffuseColor.setValue(Vector(1.0, 0.0, 0.0));
			}
			else if((!isHitAnyWall)&&( !graphCtrller->boardGr->isHit()))
			{
				graphCtrller->wall1->wallMat->diffuseColor.setValue(COLOR_BOARD_BOUNDARY);
				graphCtrller->wall2->wallMat->diffuseColor.setValue(COLOR_BOARD_BOUNDARY);
				graphCtrller->wall3->wallMat->diffuseColor.setValue(COLOR_BOARD_BOUNDARY);
				graphCtrller->boardGr->boardBoundaryMat->diffuseColor.setValue(COLOR_BOARD_BOUNDARY);
				graphCtrller->boardGr2->boardBoundaryMat->diffuseColor.setValue(COLOR_BOARD_BOUNDARY);
			}


			if(graphCtrller->boardGr->isHitObstacle())
			{
				graphCtrller->boardGr->ObstacleMat->diffuseColor.setValue(Vector(1.0, 0.0, 0.0));
				graphCtrller->boardGr2->ObstacleMat->diffuseColor.setValue(Vector(1.0, 0.0, 0.0));
				graphCtrller->boardGr->boardBoundaryMat->diffuseColor.setValue(Vector(1.0, 0.0, 0.0));
				graphCtrller->boardGr2->boardBoundaryMat->diffuseColor.setValue(Vector(1.0, 0.0, 0.0));
			}
			else if(( !graphCtrller->boardGr->isHit())&&(!graphCtrller->boardGr->isHitObstacle()))
			{	
				graphCtrller->boardGr->ObstacleMat->diffuseColor.setValue(COLOR_BOARD_BOUNDARY);
				graphCtrller->boardGr2->ObstacleMat->diffuseColor.setValue(COLOR_BOARD_BOUNDARY);
				graphCtrller->boardGr->boardBoundaryMat->diffuseColor.setValue(COLOR_BOARD_BOUNDARY);
				graphCtrller->boardGr2->boardBoundaryMat->diffuseColor.setValue(COLOR_BOARD_BOUNDARY);
			}

		}


		if(graphCtrller->boardGr->getTrial() == dataRec->NUM_TRIALS_PER_COND + 1)
		{	
		   
			effect->keepHapticLoop = false;
			printf("haptics is stopped\n");

			effect->stopHaptics();
			exit(0);
		}

		float floorWidth=graphCtrller->boardGr->floorWidth;
		Vector boundary=graphCtrller->boardGr->boundary;
		float boundaryThickness=graphCtrller->boardGr->boundaryThickness;

		if (graphCtrller->boardGr->getInit())
		{
			graphCtrller->boardGr->targetMat->diffuseColor.setValue(COLOR_GREEN);
			graphCtrller->boardGr2->targetMat->diffuseColor.setValue(COLOR_GREEN);


			// locate targets
			float target1X, target2X, target1Z, target2Z, angle1, angle2;
			int tNo =graphCtrller->boardGr->getTrial();
			if(dataRec->scenario == STRAIGHT)
			{ 
				if(dataRec->perm == 7)
				{
					    target1X = PRACTICE_TARGET_POS_A1_S [tNo %2][0];
						target2X = PRACTICE_TARGET_POS_A2_S [tNo %2][0];

						target1Z = PRACTICE_TARGET_POS_A1_S [tNo %2][2];
						target2Z = PRACTICE_TARGET_POS_A2_S [tNo %2][2];

						angle1 = PRACTICE_TARGET_ANGLE_A1[tNo %2];
						angle2 = PRACTICE_TARGET_ANGLE_A1[tNo %2]; 
				}
				else
				{
					if (tNo <= PRACTICE_COUNT) // alternating targets at right and left
					{
						target1X = PRACTICE_TARGET_POS_A1_S [tNo %2][0];
						target2X = PRACTICE_TARGET_POS_A2_S [tNo %2][0];

						target1Z = PRACTICE_TARGET_POS_A1_S [tNo %2][2];
						target2Z = PRACTICE_TARGET_POS_A2_S [tNo %2][2];

						angle1 = PRACTICE_TARGET_ANGLE_A1[tNo %2];
						angle2 = PRACTICE_TARGET_ANGLE_A1[tNo %2]; 

					}
					else{
						target1X = TARGET_POS_A1[(tNo - PRACTICE_COUNT)%7][0];
						target2X = TARGET_POS_A2[(tNo - PRACTICE_COUNT)%7][0];

						target1Z = TARGET_POS_A1[(tNo - PRACTICE_COUNT)%7][2];
						target2Z = TARGET_POS_A2[(tNo - PRACTICE_COUNT)%7][2];

						angle1 = TARGET_ANGLE_A1[(tNo - PRACTICE_COUNT)%2];
						angle2 = TARGET_ANGLE_A2[(tNo - PRACTICE_COUNT)%2];
					}
				}
			}
			else if(dataRec->scenario==MIXED)
			{	
				if(dataRec->perm == 7)
				{
					target1X = PRACTICE_TARGET_POS_A1_M [tNo %2][0];
					target2X = PRACTICE_TARGET_POS_A2_M [tNo %2][0];

					target1Z = PRACTICE_TARGET_POS_A1_M [tNo %2][2];
					target2Z = PRACTICE_TARGET_POS_A2_M [tNo %2][2];

					angle1 = PRACTICE_TARGET_ANGLE_A1[tNo %2];
					angle2 = PRACTICE_TARGET_ANGLE_A1[tNo %2]; 
				}
				else
				{
					if (tNo <= PRACTICE_COUNT) // alternating targets at right and left
					{
						target1X = PRACTICE_TARGET_POS_A1_M [tNo %2][0];
						target2X = PRACTICE_TARGET_POS_A2_M [tNo %2][0];

						target1Z = PRACTICE_TARGET_POS_A1_M [tNo %2][2];
						target2Z = PRACTICE_TARGET_POS_A2_M [tNo %2][2];

						angle1 = PRACTICE_TARGET_ANGLE_A1[tNo %2];
						angle2 = PRACTICE_TARGET_ANGLE_A1[tNo %2]; 

					}
					else{
						target1X = TARGET_POS_A1[(tNo - PRACTICE_COUNT)%10][0];
						target2X = TARGET_POS_A2[(tNo - PRACTICE_COUNT)%10][0];

						target1Z = TARGET_POS_A1[(tNo - PRACTICE_COUNT)%10][2];
						target2Z = TARGET_POS_A2[(tNo - PRACTICE_COUNT)%10][2];

						angle1 = TARGET_ANGLE_A1[(tNo - PRACTICE_COUNT)%2];
						angle2 = TARGET_ANGLE_A2[(tNo - PRACTICE_COUNT)%2];
					}
				}
			}

			//if (dataRec->cond == PARTIAL_CONFLICT) // alternating targets at right and left
			//{
			//	target1X = TARGET_POS_A1[graphCtrller->boardGr->getTrial()%6][0];
			//	target2X = TARGET_POS_A2[graphCtrller->boardGr->getTrial()%6][0];

			//	target1Z = TARGET_POS_A1[graphCtrller->boardGr->getTrial()%6][2];
			//	target2Z = TARGET_POS_A2[graphCtrller->boardGr->getTrial()%6][2];

			//	angle1 = TARGET_ANGLE_A1[graphCtrller->boardGr->getTrial()%2];
			//	angle2 = TARGET_ANGLE_A2[graphCtrller->boardGr->getTrial()%2];
			//}
			//else if (dataRec->cond == FULL_CONFLICT) // alternating targets at opposite ends, followed by a target at middle 
			//{
			//	target1X = TARGET_POS_A1[graphCtrller->boardGr->getTrial()%4][0];
			//	target2X = TARGET_POS_A2[graphCtrller->boardGr->getTrial()%4][0];

			//	target1Z = TARGET_POS_A1[graphCtrller->boardGr->getTrial()%4][2];
			//	target2Z = TARGET_POS_A2[graphCtrller->boardGr->getTrial()%4][2];

			//	angle1 = TARGET_ANGLE_A1[graphCtrller->boardGr->getTrial()%4];
			//	angle2 = TARGET_ANGLE_A2[graphCtrller->boardGr->getTrial()%4];
			//}
			//else if (dataRec->cond == SINGLE_BLIND_A1 || dataRec->cond == SINGLE_BLIND_A2 ) // only one target 
			//{
			//	target1X = TARGET_POS_A1[graphCtrller->boardGr->getTrial()%6][0];
			//	target2X = TARGET_POS_A2[graphCtrller->boardGr->getTrial()%6][0];

			//	target1Z = TARGET_POS_A1[graphCtrller->boardGr->getTrial()%6][2];
			//	target2Z = TARGET_POS_A2[graphCtrller->boardGr->getTrial()%6][2];

			//	angle1 = TARGET_ANGLE_A1[graphCtrller->boardGr->getTrial()%2];
			//	angle2 = TARGET_ANGLE_A2[graphCtrller->boardGr->getTrial()%2];
			//}
			//else if (dataRec->cond == PRACTICE || dataRec->cond == HARMONY) // both see the same targets
			//{
			//	target1X = TARGET_POS_A1[graphCtrller->boardGr->getTrial()%4][0];
			//	target2X = TARGET_POS_A2[graphCtrller->boardGr->getTrial()%4][0];

			//	target1Z = TARGET_POS_A1[graphCtrller->boardGr->getTrial()%4][2];
			//	target2Z = TARGET_POS_A2[graphCtrller->boardGr->getTrial()%4][2];

			//	angle1 = TARGET_ANGLE_A1[graphCtrller->boardGr->getTrial()%2];
			//	angle2 = TARGET_ANGLE_A2[graphCtrller->boardGr->getTrial()%2];
			//}



			graphCtrller->boardGr->targetTrans->translation.setValue(
				target1X, 
				-TARGET_HEIGHT * 0.5 + 0.5,  
				target1Z + SHIFT_Z );//cigil


			graphCtrller->boardGr->targetTrans->rotation.setValue(SbVec3f(0,1,0),angle1); 

			graphCtrller->boardGr2->targetTrans->translation.setValue(
				target2X, 
				-TARGET_HEIGHT * 0.5 + 0.5,  
				target2Z + SHIFT_Z );//cigil
			graphCtrller->boardGr2->targetTrans->rotation.setValue(SbVec3f(0,1,0),angle2); 


			if(graphCtrller->boardGr->isArrived())
			{
				graphCtrller->boardGr->targetMat->diffuseColor.setValue(COLOR_BLUE);
			}
			else 
			{
				graphCtrller->boardGr->targetMat->diffuseColor.setValue(COLOR_GREEN);
			}


			if(graphCtrller->boardGr2->isArrived())
			{
				graphCtrller->boardGr2->targetMat->diffuseColor.setValue(COLOR_BLUE);
			}
			else  
			{
				graphCtrller->boardGr2->targetMat->diffuseColor.setValue(COLOR_GREEN);
			}

			SoSeparator *root=graphCtrller->getRoot1();

			SoSeparator *tmpSprSepC = (SoSeparator*)root->getByName("SPRC"); 
			SoSeparator *tmpSprSepC2 = (SoSeparator*)root->getByName("SPRC2"); 

			graphCtrller->sprGrC->drawSpring(tmpSprSepC,Point(hcpX,hcpY,hcpZ), hcpX-cpX,hcpZ-cpZ,_CIP);
			graphCtrller->sprGrC2->drawSpring(tmpSprSepC2,Point(hcpX,hcpY,hcpZ), hcpX-cpX,hcpZ-cpZ,_CIP2);

			SoSeparator *tmpSprSepH = (SoSeparator*)root->getByName("SPRH"); 
			SoSeparator *tmpSprSepH2 = (SoSeparator*)root->getByName("SPRH2"); 

			graphCtrller->sprGrH->drawSpring(tmpSprSepH,Point(hhpX,hhpY,hhpZ), hhpX-hpX,hhpZ-hpZ,_HIP);
			graphCtrller->sprGrH2->drawSpring(tmpSprSepH2,Point(hhpX,hhpY,hhpZ), hhpX-hpX,hhpZ-hpZ,_HIP2);

		}

		runds++;
		graphCtrller->cip.transfMat->translation.setValue(cpX, cpY, cpZ);
		graphCtrller->hip.transfMat->translation.setValue(hpX, hpY, hpZ);
		graphCtrller->cip2.transfMat->translation.setValue(cpX, cpY, cpZ);
		graphCtrller->hip2.transfMat->translation.setValue(hpX, hpY, hpZ);
		graphCtrller->nip.transfMat->translation.setValue(npX, npY, npZ);
		graphCtrller->handleH.transfMat->translation.setValue(hhpX, hhpY, hhpZ);
		graphCtrller->handleC.transfMat->translation.setValue(hcpX, hcpY, hcpZ);
		graphCtrller->handleH2.transfMat->translation.setValue(hhpX, hhpY, hhpZ);
		graphCtrller->handleC2.transfMat->translation.setValue(hcpX, hcpY, hcpZ);


		scoreText->string.setValue(scoreString.c_str());
		//startText->string.set1Value(startString.c_str());
		warningText->string.setValue(warningString.c_str());
		targetText1->string.setValue(targetString1.c_str());
		targetText2->string.setValue(targetString2.c_str());
		gameText->string.setValue(gameString.c_str());
		trialText->string.setValue(trialString.c_str());

		//	graphCtrller->fvector->setVectorWidth(graphCtrller->fvector->getVectorWidth());
		//  graphCtrller->fvector->setVectorDepth(graphCtrller->fvector->getVectorDepth());
		// rotate separator placed on scene graph before board
		boardRotateX->setMatrix(boardRotateMatrixX);
		boardRotateZ->setMatrix(boardRotateMatrixZ);

		myStylusFlag = false;
	}
}

void InitialSetup()
{
	effect->keepHapticLoop = false;
	// initial transformation set to identity.
	stylusTransMatrix1 = SbMatrix(1,0,0,0,
		0,1,0,0,
		0,0,1,0,
		0,0,0,1);
	myStylusFlag = false;

	/* Seed the random-number generator with current time */
	srand ( (unsigned)time(NULL) );
}

void setMessageText(char* msg)
{
	messageString = msg;
}

void setScoreText(int score)
{
	char str[20];
	if(score>0)
	{
		sprintf(str, "Time Score: %3d", (int)(score/1000));
		scoreString = str; 
	}
	else 
	{
		sprintf(str, "  ");
		scoreString = str; 
	}
}

void setTrialText(int trialNum)
{
	char str[20];
	if(trialNum >= 0)
	{
		sprintf(str, "Trial: %d", trialNum);
	}
	else 
	{
		sprintf(str, "  ");
	}
	trialString = str; 
}

void setWarningText(char* warning)
{
	warningString = warning;
}

void setTargetText1(char* target)
{
	targetString1 = target;
}
void setTargetText2(char* target)
{
	targetString2= target;
}
void setGameOverText(char* game)
{
	gameString = game;
}

void initScreenText()
{
	SoSeparator* root1 = graphCtrller->getRoot1();	
	SoSeparator* root2 = graphCtrller->getRoot2();

	if(effect->scoreTextOn)
	{
		SoSeparator *scoreTextSep = new SoSeparator;
		SoTransform *scoreTextTrans = new SoTransform;
		SoMaterial  *scoreTextMat = new SoMaterial;

		SoFont *scoreFont = new SoFont;
		scoreFont->name.setValue("Verdana");
		scoreFont->size.setValue(3.0f);

		scoreTextTrans->translation.setValue(SHIFT_X, 0, SHIFT_Z+BOARD_WIDTH*0.15);//(0, 0, BOARD_WIDTH*0.65);
		scoreTextTrans->rotation.setValue(SbVec3f(1,0,0), TO_RADIANS(-90));

		scoreTextMat = new SoMaterial;
		scoreTextMat->ambientColor.setValue(0.15f, 1.0f, 0.30f);
		//scoreTextMat->diffuseColor.setValue(0.0f, 0.0f, 0.0f);
		scoreText = new SoAsciiText;
		scoreText->justification = SoAsciiText::CENTER;
		scoreText->string = "Time Score:   0";

		scoreTextSep->addChild(scoreFont);
		scoreTextSep->addChild(scoreTextTrans);
		scoreTextSep->addChild(scoreTextMat);
		scoreTextSep->addChild(scoreText);

		//root1->addChild(scoreTextSep);	
		//root2->addChild(scoreTextSep);	
	}

	if(effect->warningOn)
	{
		SoSeparator *warningTextSep = new SoSeparator;
		SoTransform *warningTextTrans = new SoTransform;
		SoMaterial  *warningTextMat = new SoMaterial;
		warningText = new SoAsciiText;

		SoFont *warningFont = new SoFont;
		warningFont->name.setValue("Verdana");
		warningFont->size.setValue(3.0f);

	

		if(dataRec->scenario==STRAIGHT)
		{
			warningTextTrans->translation.setValue(SHIFT_X, 10, SHIFT_Z + BOARD_DEPTH_STR / 2 - 5);
		}
		else if(dataRec->scenario==MIXED)
		{
			warningTextTrans->translation.setValue(SHIFT_X, 10, SHIFT_Z + BOARD_DEPTH_MIXED / 2 - 10);
		}
		warningTextTrans->scaleFactor.setValue(2.0,2.0,2.0);
		warningTextTrans->rotation.setValue(SbVec3f(1,0,0), TO_RADIANS(-90));

		warningTextMat->diffuseColor.setValue(0.0f,0.0f,0.0f);

		warningText->justification = SoAsciiText::CENTER;
		warningText->string = " ";

		warningTextSep->addChild(warningFont);
		warningTextSep->addChild(warningTextTrans);
		warningTextSep->addChild(warningTextMat);
		warningTextSep->addChild(warningText);

		root1->addChild(warningTextSep);
		root2->addChild(warningTextSep);
	}


	SoSeparator *targetTextSep1 = new SoSeparator; // target text for player 1
	SoSeparator *targetTextSep2 = new SoSeparator; // target text for player 2
	SoTransform *targetTextTrans = new SoTransform;
	SoMaterial  *targetTextMat = new SoMaterial;
	SoFont *targetFont = new SoFont;
	targetText1 = new SoAsciiText;
	targetText2 = new SoAsciiText;

	targetFont->name.setValue("Verdana");
	targetFont->size.setValue(3.0f);

	targetTextTrans->translation.setValue(SHIFT_X, 10,  BOARD_HEIGHT/2+SHIFT_Z);//(0, 0, BOARD_WIDTH*0.65);
	targetTextTrans->scaleFactor.setValue(2.0,2.0,2.0);
	targetTextTrans->rotation.setValue(SbVec3f(1,0,0), TO_RADIANS(-90));
	targetTextMat->diffuseColor.setValue(0.1,0.1,0.1);
	targetText1->justification = SoAsciiText::CENTER;
	targetText1->string = " ";

	targetText2->justification = SoAsciiText::CENTER;
	targetText2->string = " ";

	targetTextSep1->addChild(targetFont);
	targetTextSep1->addChild(targetTextTrans);
	targetTextSep1->addChild(targetTextMat);
	targetTextSep1->addChild(targetText1);

	targetTextSep2->addChild(targetFont);
	targetTextSep2->addChild(targetTextTrans);
	targetTextSep2->addChild(targetTextMat);
	targetTextSep2->addChild(targetText2);

	root1->addChild(targetTextSep1);	
	root2->addChild(targetTextSep2);	



	
	SoSeparator *gameTextSep = new SoSeparator; // target text for player 2
	SoTransform *gameTextTrans = new SoTransform;
	SoMaterial  *gameTextMat = new SoMaterial;
	SoFont *gameFont = new SoFont;
	gameText = new SoAsciiText;

	gameFont->name.setValue("Verdana");
	gameFont->size.setValue(3.0f);

	if(dataRec->scenario==STRAIGHT)
	{
		gameTextTrans->translation.setValue(SHIFT_X, 10, SHIFT_Z + BOARD_DEPTH_STR / 2 + 10);
	}
	else if(dataRec->scenario==MIXED)
	{
		gameTextTrans->translation.setValue(SHIFT_X, 10, SHIFT_Z + BOARD_DEPTH_MIXED / 2 + 5);
	}
	gameTextTrans->scaleFactor.setValue(2.0,2.0,2.0);
	gameTextTrans->rotation.setValue(SbVec3f(1,0,0), TO_RADIANS(-90));
	//gameTextMat->diffuseColor.setValue(0.1,0.1,0.1);
	
	gameTextMat->diffuseColor.setValue(1.0f,1.0f,1.0f);//)(0.1f, 0.1f, 0.1f);
	gameTextMat->ambientColor.setValue(0.0f, 0.0f, 0.0f);

	gameText->justification = SoAsciiText::CENTER;
	gameText->string = " ";


	gameTextSep->addChild(gameFont);
	gameTextSep->addChild(gameTextTrans);
	gameTextSep->addChild(gameTextMat);
	gameTextSep->addChild(gameText);



	root1->addChild(gameTextSep);	
	root2->addChild(gameTextSep);	

#ifdef  START_TARGET
	SoSeparator *startTextSep;
	SoTransform *startTextTrans;
	SoMaterial  *startTextMat;
	SoFont *startFont = new SoFont;
	startFont->name.setValue("Verdana");
	startFont->size.setValue(3.0f);

	startTextSep = new SoSeparator;
	startTextTrans = new SoTransform;
	startTextTrans->translation.setValue(SHIFT_X-BOARD_WIDTH*0.5+12, 10,  SHIFT_Z-15);//(0, 0, BOARD_WIDTH*0.65);
	startTextTrans->rotation.setValue(SbVec3f(1,0,0), TO_RADIANS(-45));
	startTextMat = new SoMaterial;
	startTextMat->ambientColor.setValue(0.15,1,0.30);
	//scoreTextMat->diffuseColor.setValue(0.0,0.0,0.0);
	startText = new SoAsciiText;
	startText->justification = SoAsciiText::CENTER;
	startText->string = "START";

	startTextSep->addChild(startFont);
	startTextSep->addChild(startTextTrans);
	startTextSep->addChild(startTextMat);
	startTextSep->addChild(startText);

	root->addChild(startTextSep);	

	SoSeparator *targetTextSep;
	SoTransform *targetTextTrans;
	SoMaterial  *targetTextMat;
	SoFont *targetFont = new SoFont;
	targetFont->name.setValue("Verdana");
	targetFont->size.setValue(3.0f);

	targetTextSep = new SoSeparator;
	targetTextTrans = new SoTransform;
	targetTextTrans->translation.setValue(SHIFT_X+BOARD_WIDTH*0.5-10, 10,  SHIFT_Z-15);//(0, 0, BOARD_WIDTH*0.65);
	targetTextTrans->rotation.setValue(SbVec3f(1,0,0), TO_RADIANS(-45));
	targetTextMat = new SoMaterial;
	targetTextMat->ambientColor.setValue(0.15,1,0.30);
	//scoreTextMat->diffuseColor.setValue(0.0,0.0,0.0);
	targetText = new SoAsciiText;
	targetText->justification = SoAsciiText::CENTER;
	targetText->string = "GOAL";

	targetTextSep->addChild(targetFont);
	targetTextSep->addChild(targetTextTrans);
	targetTextSep->addChild(targetTextMat);
	targetTextSep->addChild(targetText);

	root1->addChild(targetTextSep);	

#endif

	SoSeparator *messageTextSep		= new SoSeparator;;
	SoSeparator *messageTextSep2	= new SoSeparator;;
	SoTransform *messageTextTrans	= new SoTransform;
	SoMaterial  *messageTextMat		= new SoMaterial;

	SoFont *messageFont = new SoFont;
	messageText = new SoAsciiText;

	messageFont->name.setValue("Verdana");
	messageFont->size.setValue(3.0f);


	if(dataRec->scenario==ROTATIONAL)
	{
		messageTextTrans->translation.setValue(SHIFT_X, 10, SHIFT_Z - BOARD_RADIUS - 5);
	}
	else if(dataRec->scenario==STRAIGHT)
	{
		messageTextTrans->translation.setValue(SHIFT_X, 10, SHIFT_Z - BOARD_DEPTH_STR / 2 - 5);
	}
	else if(dataRec->scenario==MIXED)
	{
		messageTextTrans->translation.setValue(SHIFT_X, 10, SHIFT_Z - BOARD_DEPTH_MIXED / 2 - 5);
	}

	messageTextTrans->rotation.setValue(SbVec3f(1,0,0), TO_RADIANS(-90.0f));

	messageTextMat->diffuseColor.setValue(1.0f,1.0f,1.0f);//)(0.1f, 0.1f, 0.1f);
	messageTextMat->ambientColor.setValue(0.0f, 0.0f, 0.0f);

	messageText->justification = SoAsciiText::CENTER;
	messageText->string = "Please hold the device lightly and get ready! ";

	messageTextSep->addChild(messageFont);
	messageTextSep->addChild(messageTextTrans);
	messageTextSep->addChild(messageTextMat);
	messageTextSep->addChild(messageText);

	root1->addChild(messageTextSep);
	root2->addChild(messageTextSep);


	SoSeparator *trialTextSep = new SoSeparator;
	SoTransform *trialTextTrans = new SoTransform;
	SoMaterial  *trialTextMat = new SoMaterial;
	trialText = new SoAsciiText;

	SoFont *trialFont = new SoFont;
	trialFont->name.setValue("Verdana");
	trialFont->size.setValue(5.0f);
	if(dataRec->scenario == STRAIGHT)
	{
		trialTextTrans->translation.setValue(SHIFT_X, 0, SHIFT_Z - BOARD_DEPTH_STR/2 - 5);//(-.3*SHIFT_X, 0, -1.5*SHIFT_X);//(0, 3, 12);
	}
	else if(dataRec->scenario == MIXED)
	{
		trialTextTrans->translation.setValue(SHIFT_X, 0, SHIFT_Z - BOARD_DEPTH_MIXED/2 - 5);
	}
	else if(dataRec->scenario == ROTATIONAL)
	{
		trialTextTrans->translation.setValue(SHIFT_X, 0, SHIFT_Z - BOARD_RADIUS - 5);
	}
	trialTextTrans->rotation.setValue(SbVec3f(1,0,0), TO_RADIANS(-90));
	trialTextMat->diffuseColor.setValue(1.0,1.0,1.0);
	trialText->justification = SoAsciiText::RIGHT;
	trialText->string = " ";

	trialTextSep->addChild(trialFont);
	trialTextSep->addChild(trialTextTrans);
	trialTextSep->addChild(trialTextMat);
	trialTextSep->addChild(trialText);

	root1->addChild(trialTextSep);
	root2->addChild(trialTextSep);

}