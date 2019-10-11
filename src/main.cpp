#include <iostream>
#include <vector>
#include <string>
#include <getopt.h>
#include <time.h>

#include "minas_app.h"
#include "time_stamp.h"
#include "xmlManager.h"

using namespace std;

int main(int argc, char *argv[])
{
    ////////////////////////////////////////////  Load XML and Parse
    if(argc != 2){
        cout<< "The Number of Arguments must be 1..."<<endl;
        return 1;
    }
    string xmlFile(argv[1]);

    XmlManager XmlManager(xmlFile);
    XmlManager.XmlInit();

    vector<Motion> motions = XmlManager.GetMotions();

    for(int i=0;i<motions.size();++i){
        if(!motions[i].CalculateMotion()){
            cout<< i <<" motion parse has problems..."<<endl;
            return 0;
        }
    }

    vector<double> hs;
    for(int i=0;i<3;++i)
        hs.push_back(CylinderLength/2);

    //-- initialize hollow
    MinasApp hollow(NETWORK_ADAPTER);

    cout << "Panasonic MINAS A6B control using SOEM, start program." << endl;

    ////////////////////////////////////////////  Reset Position

    //-- Low Point: 9mm
    //-- Hign Point 97mm
    //-- Length: 88mm

    hollow.minasInit();
    hollow.minasInitCtrl();
    hollow.minasFree();
    hollow.SetZeroPosition();
 
    // Sleep 300 msec
    usleep(300000); 

    hollow.minasInit();
    hollow.minasCtrl(hs, 20000);
    hollow.SetZeroPosition();

    char c;
    cout << "Press enter to continue." << endl;
    while(1){
        // getchar();
        c = cin.get();
        if(c == '\n')
        {
            break;
        }
    } 

    ////////////////////////////////////////////  Start Motion

    /////// TEST ///////
    // vector<double> hs;
    // hs.push_back(30); // Truth should be: 29+44 = 73mm
    // hs.push_back(0);
    // hs.push_back(-30);
    // hollow.minasCtrl(hs);

    for(int i=0;i<motions.size();++i){

        double h_front,h_leftback,h_rightback,elapsed;

        motions[i].GetMotion(h_front,h_leftback,h_rightback,elapsed);

        hs[0] = h_front;
        hs[1] = h_leftback;
        hs[2] = h_rightback;

        hollow.minasCtrl(hs, elapsed);

        cout << "Motions: h_front:" << h_front << " h_leftback:"
             << h_leftback << " h_rightback:" << h_rightback << " elapsed:" << elapsed << endl;	
    }



    ////////////////////////////////////////////  End Process

    hollow.minasFree();

    cout << "End program." << endl;

    return 0;
}

//-- overrun问题临时屏蔽，位于ethercat_manager.cpp