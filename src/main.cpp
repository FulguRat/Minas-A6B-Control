#include <iostream>
#include <vector>
#include <string>
#include <getopt.h>
#include <time.h>

#include "minas_app.h"
#include "time_stamp.h"

using namespace std;

#define NETWORK_ADAPTER "enp4s0"

int main(int argc, char *argv[])
{
    //-- initialize hollow
    MinasApp hollow(NETWORK_ADAPTER);

    cout << "Panasonic MINAS A6B control using SOEM, start program." << endl;

    hollow.minasInit();

    hollow.minasCtrl(10);

    hollow.minasCtrl(-10);

    hollow.minasCtrl(10);

    hollow.minasCtrl(-10);

    hollow.minasFree();

    cout << "End program." << endl;

    return 0;
}

//-- overrun问题临时屏蔽，位于ethercat_manager.cpp