/*************************************************************************************

Simple timer class to do some task at given tymes

**************************************************************************************/
#include "HapticCallBack.h"
//#include "public.h"
#include <iostream>
using namespace std;

#using <system.dll>

using namespace System;
using namespace System::Timers;

public ref class SimHapCallback
{
public:
	static void run()
	{
		aTimer = gcnew System::Timers::Timer;

		aTimer->Elapsed += gcnew ElapsedEventHandler( SimHapCallback::OnTimedEvent );

		// Set the Interval to <SYNC_PERIOD> milliseconds
		aTimer->Interval = SYNC_PER;
		aTimer->Enabled = true;

		// Keep the timer alive until the end of the Demo method.
		// Included to prevent the JIT compiler from allowing 
		// aggressive garbage collection to occur before Demo
		// ends.
		GC::KeepAlive(aTimer);
	}

	static System::Timers::Timer^ aTimer;
//private:
	static int eventCount = -1;
	static bool calibrationOver = false;
	static int eventTriggered = -1;
	static double SYNC_PER = 10.0;	// ms
	static HapticCallBack *hCb;

	// signal for object translation and screen redraws at given time intervals.
	static void OnTimedEvent( Object^ /*source*/, ElapsedEventArgs^ /*e*/ )
	{
		eventCount++;
		//hCb->dummyHapticLoop();
		//eventTriggered = (++eventTriggered) % NUM_EVENT_TYPES;
	}

};
