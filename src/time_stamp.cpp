#include "time_stamp.h"

TimeStamp::TimeStamp(const double &_period) : period(_period)
{
}

void TimeStamp::timeStampInit(void)
{
    clock_gettime(CLOCK_REALTIME, &tick);
    timeStampInc(period);
}

void TimeStamp::timeStampSync(void)
{
    timeStampInc(period);

    //-- check run timeout
    clock_gettime(CLOCK_REALTIME, &realTime);
    double overrunTime = (realTime.tv_sec + double(realTime.tv_nsec) / NSEC_PER_SECOND) - 
                        (tick.tv_sec + double(tick.tv_nsec) / NSEC_PER_SECOND);
    if (overrunTime > 0.0f)
    {
        cout << "ERROR, overrun: time is " << overrunTime << endl;

        {
            //-- TODO: process for overrun
        }
    }

    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
}

void TimeStamp::timeStampInc(const int &_nsec)
{
    tick.tv_nsec += _nsec;

    while (tick.tv_nsec >= NSEC_PER_SECOND)
    {
        tick.tv_nsec -= NSEC_PER_SECOND;
        tick.tv_sec++;
    }
}

void TimeStamp::printMsg(void)
{
    cout << "TimeStamp: " << dec << tick.tv_sec << "." << tick.tv_nsec;
    cout << "   ";
    cout << "RealTime: " << dec << realTime.tv_sec << "." << realTime.tv_nsec;
    cout << endl;
}