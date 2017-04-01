/*************************************************************************************

Simple timer class to do some task at given tymes

**************************************************************************************/
#pragma once
//#include "public.h"
#include "HapticCallBack.h"
#include <iostream>
#include <mmsystem.h> // for PlaySound method
#using <system.dll>

using namespace std;
using namespace System;
using namespace System::Timers;

#define FX_BEEP_PIT		"sound/beep-7.wav"

public ref class SoundPlayerThread
{
public:
	static void run()
	{
		aTimer = gcnew System::Timers::Timer;

		playing = false;
		aTimer->Elapsed += gcnew ElapsedEventHandler( SoundPlayerThread::OnTimedEvent );

		// Set the Interval to <SYNC_PERIOD> milliseconds
		aTimer->Interval = SYNC_PERIOD;
		aTimer->Enabled = true;

		// Keep the timer alive until the end of the Demo method.
		// Included to prevent the JIT compiler from allowing 
		// aggressive garbage collection to occur before Demo
		// ends.
		GC::KeepAlive(aTimer);
	}

	// signal for object translation and screen redraws at given time intervals.
	static void OnTimedEvent( Object^ /*source*/, ElapsedEventArgs^ /*e*/ )
	{
		if (hapticCb->soundOn && playing == false)
		{
			if (hapticCb->hitWalls)
			{
				playing = true;
				PlaySound(FX_BEEP_PIT, NULL, SND_FILENAME|SND_SYNC);
				Sleep(200);
				playing = false;
			}
		}
	}


	static bool playing;
	static System::Timers::Timer^ aTimer;
	static double SYNC_PERIOD = 200.0;	// ms
	static HapticCallBack *hapticCb;
};
