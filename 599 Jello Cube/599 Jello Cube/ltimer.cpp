#include "ltimer.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#endif
#include "jello.h"
#include <stdio.h>

//----------------------------------------------------

static bool timerInitialized = false;
static LTimer globalTimer;

LTimer* Timer()
{
    if (!timerInitialized)
    {    
        globalTimer.Initialize();
        timerInitialized = true;
    }
    return &globalTimer;
}
    

//----------------------------------------------------


LTimer::LTimer()
{
    m_ticks = 0;
    m_frequency = 0;
}

LTimer::~LTimer()
{

}

/** update the rtdcs counter */
int64 LTimer::UpdateTicks()
{
#ifdef _WIN32
    LARGE_INTEGER i;
    QueryPerformanceCounter(&i);
    return i.QuadPart;
#else
    int64 x;
    __asm__ volatile (".byte 0x0f, 0x31" : "=A" (x));
    return x;
#endif
}

/** this is called to get the frequency, normally /proc/cpuinfo if used,
if this file cant be opened the func measures the time for 5 seconds to 
find out the resolution */
void LTimer::UpdateFrequency()
{
#ifdef _WIN32
    LARGE_INTEGER i;
    QueryPerformanceFrequency(&i);
    m_frequency = i.QuadPart;
    if (m_frequency == 0)
        printf("LTimer::UpdateFrequency: could not determine the timer frequency\n");
#else
    FILE *f;
    float tmp = 0;
    char s[256];

    if ((f = fopen("/proc/cpuinfo", "r")) != 0)
    {
        while(fgets(s,sizeof(s),f)!=0)
        {
            if(strstr(s,"cpu MHz"))
            {
		        sscanf(strchr(s,':')+1, "%e", &tmp);
		        break;
		    }
        }
    }

    if (f)
        fclose(f);

    if (fabs(tmp)<0.01)
        printf("Could not detect processor speed\n");
    else
        m_frequency = (int64)(tmp*1000000.0);
#endif
}

/** UpdateFrequency is called here */
void LTimer::Initialize()
{
    UpdateFrequency();
    m_oldTicks = UpdateTicks();
}

/** returns UpdateTicks() */
int64 LTimer::GetTicks()
{
    return UpdateTicks();
}

/** returns time in seconds */
float LTimer::GetTime()
{
    return ((float)(UpdateTicks())/(float)m_frequency);
}

/** returns time since last TimeElapsed call, in microseconds */
float LTimer::TimeElapsed()
{
    return (float)m_ticks/(float)m_frequency;
}

/** returns the processor frequency */
int64 LTimer::GetFrequency()
{
    return m_frequency;
}

/** returns the time elapsed from the last call in seconds */
int LTimer::GetFPS()
{
    return (int)(1/TimeElapsed());
}

void LTimer::NewFrame()
{
    m_ticks = UpdateTicks() - m_oldTicks;
    m_oldTicks = UpdateTicks();
}

void LTimer::SetReference()
{
	m_oldTicks = UpdateTicks();
}

float LTimer::SinceReference(){
	return (float)(UpdateTicks() - m_oldTicks)/(float)m_frequency;
}
