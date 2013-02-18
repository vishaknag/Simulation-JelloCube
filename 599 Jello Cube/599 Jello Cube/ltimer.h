#ifndef LTIMER_H
#define LTIMER_H

#ifdef _WIN32
typedef __int64 int64;
typedef unsigned __int64 uint64;
#else
typedef long long int64;
typedef unsigned long long uint64;
#endif

class LTimer
{
public: 
    LTimer();
    virtual ~LTimer();
    /** UpdateFrequency is called here */
    void Initialize();
    /** returns UpdateTicks() */
    int64 GetTicks();
    /** returns system uptime in microseconds */
    float GetTime();
    /** returns time since last TimeElapsed call, in microseconds */
    float TimeElapsed();
    /** returns the processor frequency */
    int64 GetFrequency();
    /** returns the fps */
    int GetFPS();
    // recalcs the time
    void NewFrame();

	void SetReference();
	float SinceReference();
protected: // Protected methods
    /** update the rtdcs counter */
    int64 UpdateTicks();
    /** this is called to get the resolution, normally /proc/cpuinfo if used,
        if this file cant be opened the func measures the time for 5 seconds to
        find out the resolution */
    void UpdateFrequency();
    /** the rdtcs counter */
    int64 m_ticks;
    /** ticks per second */
    int64 m_frequency;
    /** the last measured ticks */
    int64 m_oldTicks;
};

LTimer* Timer();

#endif
