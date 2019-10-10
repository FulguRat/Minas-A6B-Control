#ifndef _MINAS_APP_H
#define _MINAS_APP_H

#define NETWORK_ADAPTER "enp4s0"
#define CylinderLength 88   // ECylinder length in mm

//-- time stamp period in nanoseconds
#define TIME_STAMP_PERIOD 4e+6

#include <iostream>
#include <vector>
#include <string>
#include <time.h>
#include <stdio.h>

#include "ethercat_manager.h"
#include "minas_client.h"
#include "time_stamp.h"

using namespace std;

typedef uint8_t MinasHandle;

class MinasApp
{
public:
    explicit MinasApp(const string &_ifname);
    MinasApp(const MinasApp &) = delete;
    MinasApp &operator=(const MinasApp &) = delete;
    ~MinasApp() {}

    void minasInit(void);
    void minasFree(void);

    void minasConfig(MinasHandle _handle, uint32_t _pos, 
                    uint32_t _vel = 0x2000000, uint32_t _acc = 0x5000000, uint32_t _dec = 0x2500000, uint16_t _toq = 500);
    void minasUnitCtrl(MinasHandle _handle, uint32_t _cycle);

    void minasInitCtrl();
    void minasCtrl(vector<double> _mms);

    void printMsg(MinasHandle _handle, uint32_t _cycle);
    void printMsgInput(MinasHandle _handle);
    void printMsgOutput(MinasHandle _handle);

    void SetZeroPosition();

private:
    //-- handle for minas clients
    vector<MinasHandle> hMinas;

    //-- Time Stamp
    TimeStamp timeStamp;

    //-- network adapter
    string ifname;

    //-- EtherCAT manager
    ethercat::EtherCatManager manager;

    //-- vector for minas clients in application
    vector<minas_control::MinasClient *> vecClient;

    //-- vector for motors' initial position and target position
    //vector<int32_t> vecInitialPos;
    vector<int32_t> vecTargetPos;

    //-- vector for minas input and output
    vector<minas_control::MinasInput> vecInput;
    vector<minas_control::MinasOutput> vecOutput;

    //-- motor arrive at specified position
    vector<bool> vecArriveFlag;
    // position reset at specified velocity
    vector<bool> mVelArriveFlag;
    // motor zero positions
    vector<double> mZeroPosition;
};

#endif // _MINAS_APP_H