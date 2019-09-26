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

private:
    //-- standard time stamp and real time
    timespec tick;
    timespec realTime;

    //-- period for time stamp
    double period;
};

#endif // _TIME_STAMP_H
