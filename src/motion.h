#include <iostream>
//#include <ctime>
#include <Eigen/Dense>

class Motion{
public:
    Motion(double pitch, double roll, double updown, double elapsed) :
    mPitch(pitch), mRoll(roll), mUpdown(updown), mElapsed(elapsed) {
        mHFront = 0;
        mHLeftBack = 0;
        mHRightBack = 0;
    };

    bool CalculateMotion();
    void GetMotion(double &f,double &l, double &r, double &time);

private:
    double mPitch;
    double mRoll;
    double mUpdown;
    double mElapsed;

    double mHFront;
    double mHLeftBack;
    double mHRightBack;
};