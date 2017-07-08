#pragma once
#include "Public.h"

/* Special globals for NT performance timers */ 
LONGLONG freq; 
LONGLONG start; 
int initialized = 0;

/* Initialize everything to 0 */
void sec_init(void)
{
	LARGE_INTEGER lFreq, lCnt;
	QueryPerformanceFrequency(&lFreq);
	freq = (LONGLONG)lFreq.QuadPart;
	//cout << "freq: " << freq << endl;
	freq = freq / 1000.0;
	//cout << "freq: " << freq << endl;
	QueryPerformanceCounter(&lCnt);
	//cout << "start: " << lCnt.QuadPart << endl;
	start = (LONGLONG)lCnt.QuadPart;
	//cout << "start: " << lCnt << endl;
	initialized = 1;
}
/* return number of mseconds since sec_init was called with
** a gross amount of detail */
long sec(void)
{
	LARGE_INTEGER lCnt;
	LONGLONG tcnt;
	QueryPerformanceCounter(&lCnt);
	//tcnt = lCnt.LowPart - start;
	//cout << "end: " << lCnt.QuadPart << endl;
	tcnt = (LONGLONG)lCnt.QuadPart - start;
	return tcnt/(freq);
}

long getTicks()
{
	long time = 0l;
	if ( initialized == 0 ) {
		sec_init();
	}

	return (long)(sec());
}
