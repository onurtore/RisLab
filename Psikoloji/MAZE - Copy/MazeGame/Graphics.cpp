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

// FUNCTION PROTOTYPES
void myKeyPressCB(void *, SoEventCallback *);
void MyHandleClose(void *,class SoWinComponent *);
void graphicsTimerCallback(void *data, SoSensor *);
void InitialSetup();
void displayUsage();
void GenerateDefaultGraphics(SoSeparator *);
void outputToScreen();
void initScreenText(SoSeparator* root);
void setMessageText(char* msg);
void setScoreText(char* score);
void setWarningText(char* warning);
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
float ballRadius;


SoTransform *boardRotateX, *boardRotateZ;
SbMatrix boardRotateMatrixX, boardRotateMatrixZ;

string messageString;
string scoreString;
string warningString;
string startString;
string targetString;

//EXTERNED VARIABLES
extern Vector ctrlForce;
extern float angleX, angleZ;				// rotation angle of the board
extern float accX, accZ;
extern float positionX, positionZ;
extern float velocityX, velocityZ;
extern float o_massBall;		// mass of the ball - gr
extern float kpBN, kdBN, kpCT, kdCT;
//extern Ball *ball;
extern HapticInterfacePoint *CIP, *NIP, *HIP;
//AYSE: externed from Haptic.cpp for role exchange visualization

extern int ctrlMode;

extern SbVec3f* computedForces;

// AYSE: VISUALIZATION VARIABLES
// path visualization
SoCoordinate3	*pathBCoor, *cipPathCoor, *nipPathCoor;
SoLineSet		*pathBLines, *cipPathLines, *nipPathLpines;
SoFont			*pathBFont;

//for printing a meesage on screen
SoAsciiText *messageText;

//AYSE: for printing the score on screen
SoAsciiText *scoreText;

//AYSE: for printing the warnings on screen to motivate user
SoAsciiText *warningText;

SoAsciiText *startText;

SoAsciiText *targetText;

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

// file output
DataRecord *dataRec;
//ForceAngleRec *faRec;
GraphicsController *graphCtrller;
//GameController *gameCtrller;
	

// hook for unwanted coin error message boxes
HHOOK hMessageBoxHook_;
static int messageBoxCounter = 0;

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



int main(int argc, char **argv)
{


	int screenWidth = 1920;
	int screenHeight = 1440;

	dataRec = new DataRecord();

	if (argc == 4)
	{
		dataRec->userID = atoi(argv[1]);
		dataRec->cond	= atoi(argv[2]);
		dataRec->perm	= atoi(argv[3]);
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
	SetWindowLong(myWindow,GWL_STYLE,GetWindowLong(myWindow,GWL_STYLE) & !WS_BORDER & !WS_SIZEBOX & !WS_DLGFRAME);
	SetWindowLong(myWindow2,GWL_STYLE,GetWindowLong(myWindow,GWL_STYLE) & !WS_BORDER & !WS_SIZEBOX & !WS_DLGFRAME);
#endif

	// *****************************************************************
	// create the scene graph
	//gameCtrller = new GameController();
	graphCtrller = new GraphicsController();

	// Create the effect 
	effect = new HapticCallBack;
	SoundPlayerThread::hapticCb = effect;

	SoSeparator* root = graphCtrller->getRoot();
	GenerateDefaultGraphics(root);

	InitialSetup();

	SoWinExaminerViewer *myViewer = new SoWinExaminerViewer(myWindow);
	myViewer->setAnimationEnabled(false);
	myViewer->setSceneGraph(graphCtrller->getRoot());

	// reorient the camera so that we see the whole board from above
	SoCamera *myCamera = myViewer->getCamera();
	myCamera->position.setValue(145, 80, 60/*BOARD_WIDTH+20/* 100, 200*/);

	myCamera->pointAt(SbVec3f(140, -5, -30/*-20/* -100, -200*/));
	myViewer->saveHomePosition();
	myViewer->setTitle("Koc University RML - Maze Game - Player 1");
	myViewer->setSize(SbVec2s(screenWidth/*2560*/, screenHeight));
	myCamera->scaleHeight(1.4f);
	myViewer->setDecoration(FALSE);
	myViewer->setFullScreen(FALSE);

	myViewer->setBackgroundColor(SbColor(0,0,0));
	myViewer->show();


	SoWinExaminerViewer *myViewer2 = new SoWinExaminerViewer(myWindow2);
	myViewer2->setAnimationEnabled(false);
	myViewer2->setSceneGraph(graphCtrller->getRoot());

	// reorient the camera so that we see the whole board from above
	SoCamera *myCamera2 = myViewer2->getCamera();
	myCamera2->position.setValue(145, 80, 60/*BOARD_WIDTH+20/* 100, 200*/);

	myCamera2->pointAt(SbVec3f(140, -5, -30/*-20/* -100, -200*/));
	myViewer2->saveHomePosition();
	myViewer2->setTitle("Koc University RML - Maze Game - Player 2");
	myViewer2->setSize(SbVec2s(screenWidth/*2560*/, screenHeight));
	myCamera2->scaleHeight(1.4f);
	myViewer2->setDecoration(FALSE);
	myViewer2->setFullScreen(FALSE);

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

	initScreenText(graphCtrller->getRoot());

	ballRadius = BALL_RADIUS; 

	boardRotateX = new SoTransform;
	boardRotateZ = new SoTransform;

	//root->addChild(boardRotateX);
	//root->addChild(boardRotateZ);

	graphCtrller->addInterfacePoint(0); // cip
	graphCtrller->addInterfacePoint(1); // hip
	graphCtrller->addInterfacePoint(2); // hip
	graphCtrller->addInterfacePoint(3); // cip2
	graphCtrller->addInterfacePoint(4); // hip2
	graphCtrller->addInterfacePoint(20); // nip //cigil handlec
	graphCtrller->addInterfacePoint(21); // nip //cigil handleh

	// add the ball
	graphCtrller->addBall();

	graphCtrller->addBoard();
	//graphCtrller->addWall();
	graphCtrller->addSpring();


	//#ifdef FORCEVECTOR_ON
	graphCtrller->addVector();
	//#endif

	////////////////////////////////
	// Set up the timer callback to update the stylus
	SoTimerSensor *graphicUpdate = new SoTimerSensor(graphicsTimerCallback, root);
	graphicUpdate->setInterval(1.0/40.0);
	graphicUpdate->schedule();
}

// handle key-press event here
void myKeyPressCB(void *userData, SoEventCallback *eventCB)
{
	HDErrorInfo error;
	const SoEvent *event = eventCB->getEvent();

	if (SO_KEY_PRESS_EVENT(event, UP_ARROW)) 
	{
		// initialize phantom servo loop 
		if(effect->keepHapticLoop == false) 
		{
			effect->keepHapticLoop = true;

			effect->initialize_phantom(effect->keepHapticLoop);
			printf("Game started!\n");
			setMessageText(" ");

			// begin frame at the beginning of each servo loop
			callbackHandlers.push_back( hdScheduleAsynchronous(BeginFrameCallback, (void*)0, HD_MAX_SCHEDULER_PRIORITY) );
			// end frame at the end of each servo loop
			callbackHandlers.push_back( hdScheduleAsynchronous(EndFrameCallback, (void*)0, HD_MIN_SCHEDULER_PRIORITY) );

			//callbackHandlers.push_back( hdScheduleAsynchronous(ComputeForceCallback, computedForces, HD_DEFAULT_SCHEDULER_PRIORITY) );
			callbackHandlers.push_back( hdScheduleAsynchronous(MyHapticLoop, (void*)0, HD_DEFAULT_SCHEDULER_PRIORITY) );
			//callbackHandlers.push_back( hdScheduleAsynchronous(DutyCycleCallback, (void*)0, HD_MIN_SCHEDULER_PRIORITY) );

			// record at the end of each frame
			//callbackHandlers.push_back( hdScheduleAsynchronous(RecordCallback, (void*)0, HD_MIN_SCHEDULER_PRIORITY) );

			hdSetSchedulerRate(SCHEDULER_RATE);
			hdStartScheduler();


			SoundPlayerThread::run();

			if (HD_DEVICE_ERROR(error = hdGetError()))
			{
				//hduPrintError(stderr, &error, "Failed to start the scheduler");
				printf("Failed to start the scheduler");
				return;
			}

			////FILE *pFile = fopen("recordServoLoopData.txt","w");
			////hdStartRecord(pFile,RecordCallback,NULL,5000);
		}
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
		//cout << "scoreTextOn " << scoreTextOn << endl;
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
	// WARNING : every data should be synchronized !!
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

		alpha=graphCtrller->ballGr->ggetAngleBallGr();
		//alpha2=graphCtrller->ballGr2->ggetAngleBallGr();
		//cout<<alpha<<endl;
		graphCtrller->ballGr->getPosition().getValue(bpX, bpY, bpZ);
		//graphCtrller->ballGr2->getPosition().getValue(bpX2, bpY2, bpZ2);
		graphCtrller->cip.getPosition().getValue(cpX, cpY, cpZ);
		graphCtrller->hip.getPosition().getValue(hpX, hpY, hpZ);
		graphCtrller->nip.getPosition().getValue(npX, npY, npZ);
		graphCtrller->handleH.getPosition().getValue(hhpX, hhpY, hhpZ);
		graphCtrller->handleC.getPosition().getValue(hcpX, hcpY, hcpZ);

		if(effect->hipsOn)
		{
			graphCtrller->cip.setRadius(IP_RADIUS);
			graphCtrller->hip.setRadius(IP_RADIUS);
			graphCtrller->cip2.setRadius(IP_RADIUS);
			graphCtrller->hip2.setRadius(IP_RADIUS);
			graphCtrller->nip.setRadius(NIP_RADIUS);
			graphCtrller->handleH.setRadius(0.001);//NIP_RADIUS);
			graphCtrller->handleC.setRadius(0.001);//NIP_RADIUS);

		}
		else 
		{
			// don't draw the sphere for hip
			graphCtrller->cip.setRadius(0);
			graphCtrller->hip.setRadius(0);
			graphCtrller->cip2.setRadius(0);
			graphCtrller->hip2.setRadius(0);
			graphCtrller->nip.setRadius(0);
			graphCtrller->handleH.setRadius(0);
			graphCtrller->handleC.setRadius(0);
		}
		//alpha=graphCtrller->ballGr->ggetAngleBallGr();
		graphCtrller->ballGr->setTranslate(Vector(bpX, bpY, bpZ));
		graphCtrller->ballGr2->setTranslate(Vector(bpX+OFFSET, bpY, bpZ));

		//graphCtrller->boardGr->setColor(colorBoundary);
#ifdef ROTATION_ON
		float angle1;
		//angle1=graphCtrller->ballGr->angleBallGr;//ggetAngleBallGr();
		graphCtrller->ballGr->setRotation(alpha);
		graphCtrller->ballGr2->setRotation(alpha);
		//graphCtrller->ballGr2->setRotation(alpha);
		//cout<<alpha<<endl;


#endif




		if(graphCtrller->boardGr->isHit())
		{
			graphCtrller->boardGr->boardBoundaryMat->diffuseColor.setValue(Vector(1.0, 0.0, 0.0));
		}
		else
		{
			graphCtrller->boardGr->boardBoundaryMat->diffuseColor.setValue(COLOR_BOARD_BOUNDARY);
		}

		if(graphCtrller->wall1->isHit())
		{
			graphCtrller->wall1->wallMat->diffuseColor.setValue(Vector(1.0, 0.0, 0.0));
		}
		else
		{
			graphCtrller->wall1->wallMat->diffuseColor.setValue(COLOR_BOARD_BOUNDARY);
		}
		if(graphCtrller->wall2->isHit())
		{
			graphCtrller->wall2->wallMat->diffuseColor.setValue(Vector(1.0, 0.0, 0.0));
		}
		else
		{
			graphCtrller->wall2->wallMat->diffuseColor.setValue(COLOR_BOARD_BOUNDARY);
		}

		if(graphCtrller->wall3->isHit())
		{
			graphCtrller->wall3->wallMat->diffuseColor.setValue(Vector(1.0, 0.0, 0.0));
		}
		else
		{
			graphCtrller->wall3->wallMat->diffuseColor.setValue(COLOR_BOARD_BOUNDARY);
		}

		if(graphCtrller->boardGr->isArrived1())
		{
			graphCtrller->boardGr->targetMat1->diffuseColor.setValue(COLOR_BLUE);
		}
		else
		{
			graphCtrller->boardGr->targetMat1->diffuseColor.setValue(COLOR_GREEN);
		}
		if(graphCtrller->boardGr->isArrived2())
		{
			graphCtrller->boardGr->targetMat2->diffuseColor.setValue(COLOR_BLUE);
			effect->keepHapticLoop = false;
			printf("haptic is stopped\n");

			//Sleep(100);
			effect->stopHaptics();
		}
		else
		{
			graphCtrller->boardGr->targetMat2->diffuseColor.setValue(COLOR_GREEN);
		}


		if(effect->vectorOn)
		{
			//#ifdef FORCEVECTOR_ON
			Vector ffvector;
			ffvector=graphCtrller->fvector->getVectorT();
			graphCtrller->fvector->transfMat->translation.setValue(ffvector);
			graphCtrller->fvector->setCubeDepth(graphCtrller->fvector->getVectorDepth());
			graphCtrller->fvector->setCubeWidthHeight(1.0f,1.0f);
			graphCtrller->fvector->setRotation(graphCtrller->fvector->getVectorAngle());

			Vector ffvector1;
			ffvector1=graphCtrller->hvector->getVectorT();
			graphCtrller->hvector->transfMat->translation.setValue(ffvector1);
			graphCtrller->hvector->setCubeDepth(graphCtrller->hvector->getVectorDepth());
			graphCtrller->hvector->setCubeWidthHeight(1.0f,1.0f);
			graphCtrller->hvector->setRotation(graphCtrller->hvector->getVectorAngle());


			Vector ffvector2;
			ffvector2=graphCtrller->cvector->getVectorT();
			graphCtrller->cvector->transfMat->translation.setValue(ffvector2);
			graphCtrller->cvector->setCubeDepth(graphCtrller->cvector->getVectorDepth());
			graphCtrller->cvector->setCubeWidthHeight(1.0f,1.0f);
			graphCtrller->cvector->setRotation(graphCtrller->cvector->getVectorAngle());
			//#endif
		}	
		else
		{
			graphCtrller->fvector->setCubeDepth(0.0f);
			graphCtrller->fvector->setCubeWidthHeight(0.0f,0.0f);
			graphCtrller->hvector->setCubeDepth(0.0f);
			graphCtrller->hvector->setCubeWidthHeight(0.0f,0.0f);
			graphCtrller->cvector->setCubeDepth(0.0f);
			graphCtrller->cvector->setCubeWidthHeight(0.0f,0.0f);
		}


		if(effect->RvectorOn)
		{
			Vector reacVector2;
			reacVector2=graphCtrller->reacV->getVectorT();
			graphCtrller->reacV->transfMat->translation.setValue(reacVector2);
			graphCtrller->reacV->setCubeDepth(graphCtrller->reacV->getVectorDepth());
			graphCtrller->reacV->setCubeWidthHeight(1.0f,1.0f);
			graphCtrller->reacV->setRotation(graphCtrller->reacV->getVectorAngle());
		}

		else
		{
			graphCtrller->reacV->setCubeDepth(0.0f);
			graphCtrller->reacV->setCubeWidthHeight(0.0f,0.0f);

		}	

		SoSeparator *root=graphCtrller->getRoot();
		SoSeparator *tmpSprSepC = (SoSeparator*)root->getByName("SPRC"); 
		SoSeparator *tmpSprSepC2 = (SoSeparator*)root->getByName("SPRC2"); 
		graphCtrller->sprGrC->drawSpring(tmpSprSepC,Point(hcpX,hcpY,hcpZ), hcpX-cpX,hcpZ-cpZ,_CIP);
		graphCtrller->sprGrC2->drawSpring(tmpSprSepC2,Point(hcpX+OFFSET,hcpY,hcpZ), hcpX-cpX,hcpZ-cpZ,_CIP2);
		SoSeparator *tmpSprSepH = (SoSeparator*)root->getByName("SPRH"); 
		SoSeparator *tmpSprSepH2 = (SoSeparator*)root->getByName("SPRH2"); 
		graphCtrller->sprGrH->drawSpring(tmpSprSepH,Point(hhpX,hhpY,hhpZ), hhpX-hpX,hhpZ-hpZ,_HIP);
		graphCtrller->sprGrH2->drawSpring(tmpSprSepH2,Point(hhpX+OFFSET,hhpY,hhpZ), hhpX-hpX,hhpZ-hpZ,_HIP2);



		runds++;
		//cout<<runds<<"  angle "<<angle1<<endl;
		graphCtrller->cip.transfMat->translation.setValue(cpX, cpY, cpZ);
		graphCtrller->hip.transfMat->translation.setValue(hpX, hpY, hpZ);
		graphCtrller->cip2.transfMat->translation.setValue(cpX+OFFSET, cpY, cpZ);
		graphCtrller->hip2.transfMat->translation.setValue(hpX+OFFSET, hpY, hpZ);
		graphCtrller->nip.transfMat->translation.setValue(npX, npY, npZ);
		graphCtrller->handleH.transfMat->translation.setValue(hhpX, hhpY, hhpZ);
		graphCtrller->handleC.transfMat->translation.setValue(hcpX, hcpY, hcpZ);


		scoreText->string.setValue(scoreString.c_str());
		//startText->string.set1Value(startString.c_str());
		//targetText->target.set1Value(targetString.c_str());
		warningText->string.setValue(warningString.c_str());

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
	sprintf(str, "Time Score: %3d", (int)(score/1000));
	scoreString = str; 
}

void setWarningText(char* warning)
{
	warningString = warning;
}

void initScreenText(SoSeparator* root)
{
	if(effect->scoreTextOn)
	{
		SoSeparator *scoreTextSep;
		SoTransform *scoreTextTrans;
		SoSeparator *scoreTextSep2;
		SoTransform *scoreTextTrans2;
		SoMaterial  *scoreTextMat;
		SoFont *scoreFont = new SoFont;
		scoreFont->name.setValue("Verdana");
		scoreFont->size.setValue(6.0f);

		scoreTextSep = new SoSeparator;
		scoreTextTrans = new SoTransform;
		scoreTextTrans->translation.setValue(fwx, 0, -3.8*fwx);//(0, 0, BOARD_WIDTH*0.65);
		scoreTextTrans->rotation.setValue(SbVec3f(1,0,0), TO_RADIANS(-45));
		scoreTextSep2 = new SoSeparator;
		scoreTextTrans2 = new SoTransform;
		scoreTextTrans2->translation.setValue(fwx+OFFSET, 0, -3.8*fwx);//(0, 0, BOARD_WIDTH*0.65);
		scoreTextTrans2->rotation.setValue(SbVec3f(1,0,0), TO_RADIANS(-45));
		scoreTextMat = new SoMaterial;
		scoreTextMat->ambientColor.setValue(0.15,1,0.30);
		//scoreTextMat->diffuseColor.setValue(0.0,0.0,0.0);
		scoreText = new SoAsciiText;
		scoreText->justification = SoAsciiText::CENTER;
		scoreText->string = "Time Score:   0";

		scoreTextSep->addChild(scoreFont);
		scoreTextSep->addChild(scoreTextTrans);
		scoreTextSep->addChild(scoreTextMat);
		scoreTextSep->addChild(scoreText);

		root->addChild(scoreTextSep);	

		scoreTextSep2->addChild(scoreFont);
		scoreTextSep2->addChild(scoreTextTrans2);
		scoreTextSep2->addChild(scoreTextMat);
		scoreTextSep2->addChild(scoreText);

		root->addChild(scoreTextSep2);	


	}

	if(effect->warningOn)
	{
		SoSeparator *warningTextSep;
		SoTransform *warningTextTrans;
		SoSeparator *warningTextSep2;
		SoTransform *warningTextTrans2;
		SoMaterial  *warningTextMat;
		SoFont *warningFont = new SoFont;
		warningFont->name.setValue("Verdana");
		warningFont->size.setValue(6.0f);

		warningTextSep = new SoSeparator;
		warningTextTrans = new SoTransform;
		warningTextTrans->translation.setValue(fwx, 10, -.3*fwx);//(-.3*fwx, 0, -1.5*fwx);//(0, 3, 12);
		warningTextTrans->rotation.setValue(SbVec3f(1,0,0), TO_RADIANS(-45));
		warningTextSep2 = new SoSeparator;
		warningTextTrans2 = new SoTransform;
		warningTextTrans2->translation.setValue(fwx+OFFSET, 10, -.3*fwx);//(-.3*fwx, 0, -1.5*fwx);//(0, 3, 12);
		warningTextTrans2->rotation.setValue(SbVec3f(1,0,0), TO_RADIANS(-45));
		warningTextMat = new SoMaterial;
		warningTextMat->diffuseColor.setValue(0.9,0.15,0.30);
		warningText = new SoAsciiText;
		warningText->justification = SoAsciiText::CENTER;
		warningText->string = " ";

		warningTextSep->addChild(warningFont);
		warningTextSep->addChild(warningTextTrans);
		warningTextSep->addChild(warningTextMat);
		warningTextSep->addChild(warningText);

		root->addChild(warningTextSep);

		warningTextSep2->addChild(warningFont);
		warningTextSep2->addChild(warningTextTrans2);
		warningTextSep2->addChild(warningTextMat);
		warningTextSep2->addChild(warningText);

		root->addChild(warningTextSep2);

	}

#ifdef  START_TARGET
	SoSeparator *startTextSep;
	SoTransform *startTextTrans;
	SoMaterial  *startTextMat;
	SoFont *startFont = new SoFont;
	startFont->name.setValue("Verdana");
	startFont->size.setValue(3.0f);

	startTextSep = new SoSeparator;
	startTextTrans = new SoTransform;
	startTextTrans->translation.setValue(fwx-BOARD_WIDTH*0.5+12, 10,  fw-15);//(0, 0, BOARD_WIDTH*0.65);
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
	targetTextTrans->translation.setValue(fwx+BOARD_WIDTH*0.5-10, 10,  fw-15);//(0, 0, BOARD_WIDTH*0.65);
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

	root->addChild(targetTextSep);	

#endif

	SoSeparator *messageTextSep;
	SoTransform *messageTextTrans;
	SoSeparator *messageTextSep2;
	SoTransform *messageTextTrans2;
	SoMaterial  *messageTextMat;
	SoFont *messageFont = new SoFont;
	messageFont->name.setValue("Verdana");
	messageFont->size.setValue(6.0f);

	messageTextSep = new SoSeparator;
	messageTextTrans = new SoTransform;
	//messageTextTrans->translation.setValue(0, 3, 0);
	messageTextTrans->translation.setValue(-1.2*fw, 10, -0.8*fwx);
	messageTextTrans->rotation.setValue(SbVec3f(1,0,0), TO_RADIANS(-45.0f));


	messageTextSep2 = new SoSeparator;
	messageTextTrans2 = new SoTransform;
	//messageTextTrans->translation.setValue(0, 3, 0);
	messageTextTrans2->translation.setValue(-.9*fw+OFFSET, 10, -0.8*fwx);
	messageTextTrans2->rotation.setValue(SbVec3f(1,0,0), TO_RADIANS(-45.0f));


	messageTextMat = new SoMaterial;
	messageTextMat->ambientColor.setValue(0.15f,1,0.30f);
	messageText = new SoAsciiText;
	messageText->justification = SoAsciiText::CENTER;
	messageText->string = "Press Esc, then Up Arrow to Start";

	messageTextSep->addChild(messageFont);
	messageTextSep->addChild(messageTextTrans);
	messageTextSep->addChild(messageTextMat);
	messageTextSep->addChild(messageText);

	root->addChild(messageTextSep);

	messageTextSep2->addChild(messageFont);
	messageTextSep2->addChild(messageTextTrans2);
	messageTextSep2->addChild(messageTextMat);
	messageTextSep2->addChild(messageText);

	//root->addChild(messageTextSep2);
}