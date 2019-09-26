#include "minas_app.h"

MinasApp::MinasApp(const string &_ifname) : ifname(_ifname),
                                            manager(_ifname),
                                            timeStamp(TIME_STAMP_PERIOD)
{
    for (uint8_t clientNo = 0; clientNo < manager.getNumClients(); clientNo++)
    {
        vecClient.push_back(new minas_control::MinasClient(manager, clientNo + 1));
        
        //-- change to MINAS driver ID
        hMinas.push_back(clientNo);

        //-- motor not arrive
        vecArriveFlag.push_back(FALSE);
    }
}

void MinasApp::minasInit(void)
{
    cout << "=========>> [Enter minasInit]" << endl;

    for (uint8_t clientNo = 0; clientNo < hMinas.size(); clientNo++)
    {
        minas_control::MinasClient *client = vecClient[clientNo];

        //-- clear error
        client->reset();

        //-- set paramete from PANATERM test program
        client->setTrqueForEmergencyStop(100);      // 100%
        client->setOverLoadLevel(50);               // 50%
        client->setOverSpeedLevel(1200);             // r/min
        client->setMotorWorkingRange(0.1);          // 0.1

        client->setInterpolationTimePeriod(4000);   // 4 msec

        //-- servo on
        client->servoOn();

        //-- get a initial position for motor action once
        minas_control::MinasInput input = client->readInputs();
        vecInput.push_back(input);

        printMsgInput(clientNo);

        vecInitialPos.push_back(vecInput[clientNo].position_actual_value);

        //-- set target position
        minas_control::MinasOutput output;
        memset(&output, 0x00, sizeof(minas_control::MinasOutput));
        vecOutput.push_back(output);

        //-- Position control
        vecTargetPos.push_back(vecInitialPos[clientNo]);
    }

    cout << "=========>> [Leave minasInit]" << endl;
}

void MinasApp::minasFree(void)
{
    cout << "=========>> [Enter minasFree]" << endl;

    for (vector<minas_control::MinasClient *>::iterator it = vecClient.begin(); it != vecClient.end(); ++it)
    {
        minas_control::MinasClient *client = (*it);
        
        minas_control::MinasInput input = client->readInputs();

        client->printPDSStatus(input);
        client->printPDSOperation(input);

        //-- servo off
        client->servoOff();
    }

    cout << "=========>> [Leave minasFree]" << endl;
}

void MinasApp::minasConfig(MinasHandle _handle, uint32_t _pos, 
                                                    uint32_t _vel, uint32_t _acc, uint32_t _dec)
{
    cout << "=========>> [Enter minasConfig]" << endl;

    minas_control::MinasClient *client = vecClient[_handle];

    //-- Position control
    vecOutput[_handle].target_position = _pos;
    
    vecOutput[_handle].max_motor_speed = 1140;   // rad/min
    vecOutput[_handle].target_torque = 5000;     // 0% (unit 0.1%)
    vecOutput[_handle].max_torque = 5000;        // 50% (unit 0.1%)
    vecOutput[_handle].operation_mode = 0x01;   // position profile mode (pp)
    vecOutput[_handle].controlword = 0x001f;    // move to operation enabled + new set-point (bit4) + change set immediately (bit5)
    
    //-- set profile velocity
    client->setProfileVelocity(_vel);

    //-- set profile acceleration
    client->setProfileAcceleration(_acc);

    //-- set profile deceleration
    client->setProfileDeceleration(_dec);

    //-- pp control model setup
    //-- see controlword(6040.h) p.107 & statusword(6041.h) p.113
    client->writeOutputs(vecOutput[_handle]);
    while (!(vecInput[_handle].statusword & 0x1000))
    {
        vecInput[_handle] = client->readInputs(); // bit12 (set-point acknowledge)
    }
    
    printMsgInput(_handle);

    vecOutput[_handle].controlword &= ~0x0010; // clear new set-point (bit4)
    client->writeOutputs(vecOutput[_handle]);
    while (vecInput[_handle].statusword & 0x0400)
    {
        vecInput[_handle] = client->readInputs(); // bit12 (set-point acknowledge)
    }

    printMsgInput(_handle);

    cout << "Target position has been changed to: "<< hex << vecOutput[_handle].target_position << endl;

    cout << "=========>> [Leave minasConfig]" << endl;
}

void MinasApp::minasUnitCtrl(MinasHandle _handle, double _round, uint32_t _cycle)
{
    minas_control::MinasClient *client = vecClient[_handle];
    vecInput[_handle] = client->readInputs();
    vecOutput[_handle] = client->readOutputs();

    //-- Judge if it is reached
    if (_cycle % 10 == 0)
    {
        //-- print debug message
        printMsg(_handle, _cycle);

        //-- Target reached 
        //-- see statusword (6041h) p.102
        if (vecInput[_handle].statusword & 0x0400)
        {
            vecInitialPos[_handle] = vecInput[_handle].position_actual_value;

            //-- target reached (bit 10)
            cout << "Target reached, current position is: " << hex << vecInitialPos[_handle] << endl;

            vecArriveFlag[_handle] = TRUE;

            return;
        }
    }

    client->writeOutputs(vecOutput[_handle]);
}

void MinasApp::minasCtrl(double _round)
{
    cout << "=========>> [Enter minasCtrl]" << endl;

    for (uint8_t clientNo = 0; clientNo < hMinas.size(); clientNo++)
    {
        cout << "  <<<  client no.  " << dec << clientNo << endl;
        minas_control::MinasClient *client = vecClient[clientNo];

        vecInput[clientNo] = client->readInputs();
        vecInitialPos[clientNo] = vecInput[clientNo].position_actual_value;

        cout << "  <<<  initial position   " << vecInitialPos[clientNo] << endl;

        vecTargetPos[clientNo] = vecInitialPos[clientNo] + 0x800000 * _round;

        //-- config new action
        minasConfig(clientNo, vecTargetPos[clientNo], 0x16000000, 0x80000000, 0x80000000);
    }
    
    //-- initialize time stamp
    timeStamp.timeStampInit();

    for (uint32_t cycle = 0;; cycle++)
    {
        //-- for several clients
        for (uint8_t clientNo = 0; clientNo < hMinas.size(); clientNo++)
        {
            minasUnitCtrl(hMinas[clientNo], _round, cycle);
        }

        timeStamp.timeStampSync();

        bool allArriveFlag = TRUE;
        for (uint8_t clientNo = 0; clientNo < vecArriveFlag.size(); clientNo++)
        {
            allArriveFlag &= vecArriveFlag[clientNo];
        }
        
        if (allArriveFlag == TRUE)
        {
            for (uint8_t clientNo = 0; clientNo < hMinas.size(); clientNo++)
            {
                vecArriveFlag[clientNo] = FALSE;
            }

            break;
        }
    }

    cout << "=========>> [Leave minasCtrl]" << endl;
}

void MinasApp::printMsg(MinasHandle _handle, uint32_t _cycle)
{
    printf("\n\n\n");
    printf("Period   %d\n", _cycle);
    timeStamp.printMsg();
    printMsgInput(_handle);
    printMsgOutput(_handle);
}

void MinasApp::printMsgInput(MinasHandle _handle)
{
    printf("Input:\n");
        printf("   603Fh %08x :Error code\n", vecInput[_handle].error_code);
        printf("   6041h %08x :Statusword\n", vecInput[_handle].statusword);
        printf("   6061h %08x :Modes of operation display\n", vecInput[_handle].operation_mode);
        printf("   6064h %08x :Position actual value\n", vecInput[_handle].position_actual_value);
        printf("   606Ch %08x :Velocity actual value\n", vecInput[_handle].velocity_actual_value);
        printf("   6077h %08x :Torque actual value\n", vecInput[_handle].torque_actual_value);
        printf("   60B9h %08x :Touch probe status\n", vecInput[_handle].touch_probe_status);
        printf("   60BAh %08x :Touch probe pos1 pos value\n", vecInput[_handle].touch_probe_posl_pos_value);
        printf("   60FDh %08x :Digital inputs\n", vecInput[_handle].digital_inputs);
}
    
void MinasApp::printMsgOutput(MinasHandle _handle)
{
    printf("Output:\n");
        printf("   6040h %08x :Controlword\n", vecOutput[_handle].controlword);
        printf("   6060h %08x :Mode of operation\n", vecOutput[_handle].operation_mode);
        printf("   6071h %08x :Target Torque\n", vecOutput[_handle].target_torque);
        printf("   6072h %08x :Max Torque\n", vecOutput[_handle].max_torque);
        printf("   607Ah %08x :Target Position\n", vecOutput[_handle].target_position);
        printf("   6080h %08x :Max motor speed\n", vecOutput[_handle].max_motor_speed);
        printf("   60B8h %08x :Touch Probe function\n", vecOutput[_handle].touch_probe_function);
        printf("   60FFh %08x :Target Velocity\n", vecOutput[_handle].target_velocity);
        printf("   60B0h %08x :Position Offset\n", vecOutput[_handle].position_offset);
}