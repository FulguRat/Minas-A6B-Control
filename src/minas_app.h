#ifndef _MINAS_APP_H
#define _MINAS_APP_H

#define NETWORK_ADAPTER "enp4s0"

//-- time stamp period in nanoseconds
#define TIME_STAMP_PERIOD 4e+6

//-- Unit command per millimeter
#define UNIT_COMMAND_MM 0x19999a

#include <iostream>
#include <vector>
#include <string>
#include <time.h>

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
                    uint32_t _vel = 0x2000000, uint32_t _acc = 0x5000000, uint32_t _dec = 0x2500000);
    void minasUnitCtrl(MinasHandle _handle, double _round, uint32_t _cycle);

    void minasCtrl(double _round);

    void printMsg(MinasHandle _handle, uint32_t _cycle);
    void printMsgInput(MinasHandle _handle);
    void printMsgOutput(MinasHandle _handle);


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
    vector<int32_t> vecInitialPos;
    vector<int32_t> vecTargetPos;

    //-- vector for minas input and output
    vector<minas_control::MinasInput> vecInput;
    vector<minas_control::MinasOutput> vecOutput;

    //-- motor arrive at specified position
    vector<bool> vecArriveFlag;

};

#endif // _MINAS_APP_H