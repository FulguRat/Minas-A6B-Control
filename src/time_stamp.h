#ifndef _TIME_STAMP_H
#define _TIME_STAMP_H

#define NSEC_PER_SECOND 1e+9
#define USEC_PER_SECOND 1e+6

#include <iostream>
#include <time.h>

using namespace std;

class TimeStamp
{
public:
    explicit TimeStamp(const double &_period);
    TimeStamp(const TimeStamp &) = delete;
    TimeStamp &operator=(const TimeStamp &) = delete;
    ~TimeStamp() {}

    void timeStampInit(void);

    void timeStampSync(void);

    void timeStampInc(const int &_nsec);

    void printMsg(void);
    
    timespec GetRealTime(){
        return realTime;
    }

    // unit: ms; has 10ms bias
    double operator-(const timespec& t2){
        return ((realTime.tv_sec + double(realTime.tv_nsec) / NSEC_PER_SECOND) - 
                        (t2.tv_sec + double(t2.tv_nsec) / NSEC_PER_SECOND)) * 1e3;
    }

private:
    //-- standard time stamp and real time
    timespec tick;
    timespec realTime;

    //-- period for time stamp
    double period;
};

#endif // _TIME_STAMP_H
