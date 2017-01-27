/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 * Based on actionsRenderingEngine by Carlo Ciliberto, Vadim Tikhanoff
 *
 */

#include <yarp/sig/Vector.h>

#include "GesturesRenderingEngineDefaults.h"
#include "GesturesRenderingEngineThread.h"

using namespace std;
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;

/**********************************************************/
void GesturesRenderingEngineThread::steerArmToHome()
{
     // TODO: use posArm instead
    IPositionControl *armPosCtrl;

    drvLeftArm->view(armPosCtrl);

    yInfo("homing arm...");

    for (int j=0; j<armHomeVels.length(); j++)
    {
        armPosCtrl->setRefSpeed(j, armHomeVels[j]);
        armPosCtrl->positionMove(j, armHomePoss[j]);
    }

    openHand();

    yInfo("...done homing arm");
}
/**********************************************************/
void GesturesRenderingEngineThread::steerArmToLow()
{
    // TODO: use posArm instead
    IPositionControl *armPosCtrl;

    drvLeftArm->view(armPosCtrl);

    for (int j=0; j<armLowVels.length(); j++)
    {
        armPosCtrl->setRefSpeed(j, armLowVels[j]);
        armPosCtrl->positionMove(j, armLowPoss[j]);
    }
}

/**********************************************************/
void GesturesRenderingEngineThread::steerArmToFront()
{
    IPositionControl *armPosCtrl;

    drvLeftArm->view(armPosCtrl);

    for (int j=0; j<armFrontVels.length(); j++)
    {
        armPosCtrl->setRefSpeed(j, armFrontVels[j]);
        armPosCtrl->positionMove(j, armFrontPoss[j]);
    }
}

/**********************************************************/
void GesturesRenderingEngineThread::steerHeadToHome()
{
    Vector homeHead(3);

    homeHead[0] = -1.0;
    homeHead[1] =  0.0;
    homeHead[2] =  0.3;

    yInfo("homing head...");

    gazeCtrl->lookAtFixationPoint(homeHead);
    gazeCtrl->waitMotionDone();

    yInfo("...done homing head");
}

/**********************************************************/
void GesturesRenderingEngineThread::openHand()
{
    moveHand(HAND_OPEN);
}

/**********************************************************/
void GesturesRenderingEngineThread::closeHand()
{
    moveHand(HAND_PUNCH);
}

/**********************************************************/
void GesturesRenderingEngineThread::moveHand(const int action)
{
    // TODO: use posArm instead
    IPositionControl *armPosCtrl;
    Vector *poss = NULL;

    switch(action)
    {
        case HAND_OPEN:
        {
            poss = &handOpenPoss;
            break;
        }
        case HAND_PUNCH:
        {
            poss = &handPunchPoss;
            break;
        }
        default:
            return;
    }

    drvLeftArm->view(armPosCtrl);

    for (int j=0; j<handVels.length(); j++)
    {
        int k = armHomeVels.length() + j;

        armPosCtrl->setRefSpeed(k, handVels[j]);
        armPosCtrl->positionMove(k, (*poss)[j]);
    }
}

/**********************************************************/
GesturesRenderingEngineThread::GesturesRenderingEngineThread(
    const string &_moduleName,
    ResourceFinder &_rf)
    : RateThread(DefThreadPeriod),
      moduleName(_moduleName),
      rf(_rf)
{
    drvHead = NULL;
    drvGazeCtrl = NULL;
    headPosCtrl = NULL;
    modeHead = NULL;
    drvLeftArm = NULL;
}

/**********************************************************/
void GesturesRenderingEngineThread::close()
{
    steerHeadToHome();
    steerArmToHome();

    Time::delay(1.0);

    drvHead->close();
    drvGazeCtrl->close();
    drvLeftArm->close();

    if (drvHead) delete drvHead;
    if (drvGazeCtrl) delete drvGazeCtrl;
    if (drvLeftArm) delete drvLeftArm;

    return;
}

/**********************************************************/
void GesturesRenderingEngineThread::interrupt()
{
    return;
}

/**********************************************************/
bool GesturesRenderingEngineThread::threadInit()
{
    robotName = rf.check("robot",Value(DefRobot.c_str())).asString().c_str();
    repetitions = rf.check("repetitions",Value(DefRepetitions)).asInt();

    // open head remote_controlboard driver
    Property optHead("(device remote_controlboard)");
    optHead.put("remote",("/"+robotName+"/head").c_str());
    optHead.put("local",("/"+moduleName+"/head").c_str());
    drvHead = new PolyDriver;
    if (!drvHead->open(optHead))
    {
        yError("Position head controller not available");
        close();
        return false;
    }

    // open gazecontrollerclient driver
    Property optGazeCtrl("(device gazecontrollerclient)");
    optGazeCtrl.put("remote","/iKinGazeCtrl");
    optGazeCtrl.put("local",("/"+moduleName+"/gaze").c_str());
    drvGazeCtrl = new PolyDriver;
    if (!drvGazeCtrl->open(optGazeCtrl))
    {
        yError("Gaze controller not available");
        close();
        return false;
    }

    // open left_arm remote_controlboard driver
    Property optLeftArm("(device remote_controlboard)");
    optLeftArm.put("remote",("/"+robotName+"/left_arm").c_str());
    optLeftArm.put("local",("/"+moduleName+"/left_arm").c_str());
    drvLeftArm = new PolyDriver;
    if (!drvLeftArm->open(optLeftArm))
    {
        yError("Position left_arm controller not available");
        close();
        return false;
    }

    if (!drvHead->isValid() || !drvLeftArm->isValid() || !drvGazeCtrl->isValid())
    {
        yError("Problem configuring drivers");
        close();
        return false;
    }

    // open head device views
    bool ok = true;
    ok = ok && drvHead->view(headPosCtrl);
    ok = ok && drvHead->view(encHead);
    ok = ok && drvHead->view(modeHead);
    if (!ok)
    {
        yError("problem acquiring head interfaces");
        close();
        return false;
    }

    // open left_arm device views
    ok = ok && drvLeftArm->view(encArm);
    ok = ok && drvLeftArm->view(posArm);
    //ok = ok && drvLeftArm->view(modeArm);
    if (!ok)
    {
        yError("problem acquiring left_arm interfaces");
        close();
        return false;
    }

    // open gaze device views
    ok = ok && drvGazeCtrl->view(gazeCtrl);
    if (!ok)
    {
        yError("problem acquiring gaze interfaces");
        close();
        return false;
    }
    if (gazeCtrl == NULL)
    {
        yError("problem with gaze interface when initializing IGazeControl");
        close();
        return false;
    }

    // initialize control variables
    headAxes = 0;
    encHead->getAxes(&headAxes);
    const int expectedHeadAxes = 6;
    if (headAxes != expectedHeadAxes)
        yWarning("got %d head axes, was expecting %d", headAxes, expectedHeadAxes);

    head.resize(headAxes, 0.0);

    velHead.resize(headAxes, 10.0);
    headPosCtrl->setRefSpeeds(velHead.data());

    accHead.resize(headAxes, 50.0);
    headPosCtrl->setRefAccelerations(accHead.data());

    armAxes = 0;
    encArm->getAxes(&armAxes);
    const int expectedArmAxes = 16;
    if (armAxes != expectedArmAxes)
        yWarning("got %d arm axes, was expecting %d", armAxes, expectedArmAxes);

    gazeCtrl->setNeckTrajTime(2.0);
    gazeCtrl->setEyesTrajTime(1.0);
    gazeCtrl->setTrackingMode(false); // tracking mode: torso moves => gaze compensates
    gazeCtrl->setSaccadesMode(false);

    armHomePoss.resize(7, 0.0);
    armHomePoss[0]=-30.0; armHomePoss[1]=30.0; armHomePoss[2]=45.0;

    armHomeVels.resize(7, 0.0);
    armHomeVels = 10.0;

    armLowPoss.resize(7, 0.0);
    armLowPoss[0]=-10.0; armLowPoss[1]=30.0;
    //armLowPoss[2]=100.0;
    armLowPoss[2]=0.0; // safer

    armLowVels.resize(7, 0.0);
    armLowVels[0]=armLowVels[3]=20.0;
    armLowVels[1]=armLowVels[2]=armLowVels[4]=armLowVels[5]=armLowVels[6]=10.0;

    armFrontPoss.resize(7, 0.0);
    armFrontPoss[0]=-70.0;
    //armFrontPoss[1]=20.0;
    armFrontPoss[1]=25.0; // safer
    armFrontPoss[3]=30.0;

    armFrontVels.resize(7, 0.0);
    armFrontVels = 10.0;
    armFrontVels[0]=20.0; armFrontVels[3]=30.0;

    handOpenPoss.resize(9, 0.0);

    handPunchPoss.resize(9, 0.0);
    handPunchPoss[1]=20.0; handPunchPoss[2]=25.0;
    //handPunchPoss[3]=40.0;
    handPunchPoss[3]=85.0; // increased in 2017
    handPunchPoss[4]=50.0;
    //handPunchPoss[5]=40.0;
    handPunchPoss[5]=85.0; // increased in 2017
    handPunchPoss[6]=50.0;
    //handPunchPoss[7]=50.0;
    handPunchPoss[7]=70.0; // increased in 2017
    //handPunchPoss[8]=100.0;
    handPunchPoss[8]=110.0; // increased in 2017

    handVels.resize(9, 0.0);
    handVels = 20.0;
    handVels[0]=10.0;

    // start control
    steerHeadToHome();
    steerArmToHome();

    closing = false;

    return true;
}

/**********************************************************/
void GesturesRenderingEngineThread::run()
{
    if (!closing)
    {
        yInfo("running fine, waiting for commands via RPC");
        yarp::os::Time::delay(10.0);
    }
}

// IDL functions

/**********************************************************/
bool GesturesRenderingEngineThread::do_nod()
{
    LockGuard lg(mutex);
    yInfo("doing nod action...");

    steerHeadToHome();

    // try to set position control mode
    yInfo("setting position control mode...");
    for(int i=0; i<headAxes; i++)
        modeHead->setControlMode(i,VOCAB_CM_POSITION);

    // wait a bit
    Time::delay(1.0);

    // check if position control mode was successfully set
    bool gotPositionControlMode = true;
    VectorOf<int> currHeadModes(headAxes);
    modeHead->getControlModes(currHeadModes.getFirst());
    for (size_t i=0; i<currHeadModes.size(); ++i)
    {
        if (currHeadModes[i] != VOCAB_CM_POSITION)
        {
            yWarning("joint %lu is not VOCAB_CM_POSITION as required", i);
            gotPositionControlMode = false;
        }
    }

    // if we were unsuccessful in setting position control, abort
    if (gotPositionControlMode)
    {
        yInfo("...successfully set position control mode");
    }
    else
    {
        yWarning("...failed to set set position control mode, aborting nod action, be careful");
        return false;
    }

    // read current joint values and save them in head variable
    encHead->getEncoders(head.data());

    headPosCtrl->setRefSpeeds(velHead.data());
    headPosCtrl->setRefAccelerations(accHead.data());

    //was: double init_j0 = head[0];
    double init_j0 = 0.0;

    // parameter
    double final_j0 = -35.0;

    //double t_j0 = 2.0;
    double t_j0 = 5.0; // temporarily increased in 2017 for safety

    for (int times=0; times<repetitions; times++)
    {
        head[0] = init_j0;
        headPosCtrl->positionMove(head.data());

        Time::delay(t_j0);

        head[0] = final_j0;
        headPosCtrl->positionMove(head.data());

        Time::delay(t_j0);
    }

    head[0] = init_j0;
    headPosCtrl->positionMove(head.data());

    bool done = false;
    double t0 = Time::now();
    const double timeout = 5.0;
    while (!done && (Time::now()-t0<timeout))
    {
        headPosCtrl->checkMotionDone(&done);
        Time::delay(0.1);
    }

    yInfo("...done nod action");

    return true;
}

/**********************************************************/
bool GesturesRenderingEngineThread::do_punch()
{
    LockGuard lg(mutex);
    yInfo("doing punch action...");

    double timing = 3.0;

    // TODO: increase velocity

    for (int times=0; times<repetitions; times++)
    {
        closeHand();
        Time::delay(timing);
        steerArmToLow();
        Time::delay(timing);
        steerArmToFront();
        Time::delay(timing);
    }

    steerArmToHome();

    yInfo("...done punch action");

    return true;
}

/**********************************************************/
bool GesturesRenderingEngineThread::do_lookout()
{
    LockGuard lg(mutex);
    yInfo("doing lookout action...");

    steerHeadToHome();

    double timing = 3.0;

    // parameter
    double y_far = -0.60;

    Vector fp(3);
    fp[0] = -0.50;    // x-component [m]
    fp[1] = +0.00;    // y-component [m]
    fp[2] = +0.35;    // z-component [m]

    for (int times=0; times<repetitions; times++)
    {
        gazeCtrl->lookAtFixationPoint(fp); // move the gaze to the desired fixation point
        //gazeCtrl->waitMotionDone();        // wait until the operation is done

        fp[1] = y_far;
        fp[2] = +0.60;
        Time::delay(timing);
        gazeCtrl->lookAtFixationPoint(fp);
        //gazeCtrl->waitMotionDone();

        Time::delay(timing);
        fp[1] = +0.00;
        fp[2] = +0.35;
    }

    steerHeadToHome();

    yInfo("...done lookout action");

    return true;
}

/**********************************************************/
bool GesturesRenderingEngineThread::do_thumbsup()
{
    LockGuard lg(mutex);
    yInfo("doing thumbs up action...");

    steerArmToHome();
    closeHand();

    // parameter
    double vel8fin = 30.0;

    Vector posArmInit(16); // should not be hardcoded
    posArmInit     =   0.0;
    posArmInit[0]  = -10.0;
    posArmInit[1]  =  30.0;
    posArmInit[3]  =  60.0; // different from punch
    posArmInit[8]  =  20.0;
    //posArmInit[9]  =  25.0;
    posArmInit[9]  =  35.0; // 2017
    posArmInit[10] =  40.0;
    posArmInit[11] =  50.0;
    posArmInit[12] =  40.0;
    posArmInit[13] =  50.0;
    posArmInit[14] =  50.0;
    posArmInit[15] = 100.0;
    Vector velArmInit(16);
    velArmInit    =   10.0;
    velArmInit[0] =   20.0;
    velArmInit[4] =   20.0;
    velArmInit[8] =   30.0;
    velArmInit[15] =  40.0; // new

    Vector posArmFin(16);
    posArmFin      =    0.0;
    posArmFin[0]   =  -30.0;
    posArmFin[1]   = posArmInit[1];
    posArmFin[2]   = posArmInit[2];
    posArmFin[3]   = posArmInit[3];
    //posArmFin[4]   =  -50.0;
    posArmFin[4]   =  -20.0; // 2017
    posArmFin[5]   = posArmInit[5];
    posArmFin[6]   = posArmInit[6];
    posArmFin[7]   = posArmInit[7];
    posArmFin[8]   =  -10.0;
    //posArmFin[9]   = posArmInit[9];
    posArmFin[9]   =   25.0;
    posArmFin[10]  =    0.0;
    posArmFin[11]  = posArmInit[11];
    posArmFin[12]  = posArmInit[12];
    posArmFin[13]  = posArmInit[13];
    posArmFin[14]  = posArmInit[14];
    posArmFin[15]  = posArmInit[15];
    Vector velArmFin(16);
    velArmFin     =   10.0;
    velArmFin[0]  =   20.0;
    velArmFin[4]  =   20.0;
    velArmFin[8]  = vel8fin;
    velArmFin[10] =   20.0;
    velArmFin[15] =   40.0; // new

    double timing = 3.0;

    for (int times=0; times<repetitions; times++)
    {
        posArm->setRefSpeeds(velArmInit.data());
        posArm->positionMove(posArmInit.data());  

        Time::delay(timing);

        posArm->setRefSpeeds(velArmFin.data());
        posArm->positionMove(posArmFin.data());  

        Time::delay(timing*1.2); // increased in 2017
    }

    steerArmToHome();

    yInfo("...done thumbs up action");

    return true;
}

/**********************************************************/
bool GesturesRenderingEngineThread::do_thumbsdown()
{
    LockGuard lg(mutex);
    yInfo("doing thumbs down action...");

    steerArmToHome();
    closeHand();

    // parameter
    double vel8fin = 30.0;

    Vector posArmInit(16); // should not be hardcoded
    posArmInit     =   0.0;
    //posArmInit[0]  = -10.0;
    posArmInit[0]  = -20.0; // safer
    posArmInit[1]  =  30.0;
    posArmInit[3]  =  80.0;
    posArmInit[8]  =  20.0;
    posArmInit[9]  =  25.0;
    posArmInit[10] =  40.0;
    posArmInit[11] =  50.0;
    posArmInit[12] =  40.0;
    posArmInit[13] =  50.0;
    posArmInit[14] =  50.0;
    posArmInit[15] = 100.0;
    Vector velArmInit(16);
    velArmInit    =   10.0;
    velArmInit[0]  =  20.0;
    velArmInit[4]  =  30.0;
    velArmInit[8]  =  30.0;
    velArmInit[10] =  20.0;
    velArmInit[15] =  40.0; // new

    Vector posArmFin(16);
    posArmFin      =    0.0;
    posArmFin[0]   = posArmInit[0];
    //posArmFin[1]   = posArmInit[1];
    posArmFin[1]   =   40.0; // 2017
    //posArmFin[2]   = posArmInit[2];
    posArmFin[2]   =   13.3; // 2017
    posArmFin[3]   = posArmInit[3];
    posArmFin[4]   =   90.0;
    posArmFin[5]   = posArmInit[5];
    posArmFin[6]   = posArmInit[6];
    posArmFin[7]   = posArmInit[7];
    posArmFin[8]   =   20.0;
    posArmFin[9]   = posArmInit[9];
    posArmFin[10]  =    0.0;
    posArmFin[11]  = posArmInit[11];
    posArmFin[12]  = posArmInit[12];
    posArmFin[13]  = posArmInit[13];
    posArmFin[14]  = posArmInit[14];
    posArmFin[15]  = posArmInit[15];
    Vector velArmFin(16);
    velArmFin     =   10.0;
    velArmFin[0]  =   20.0;
    velArmFin[4]  =   30.0;
    velArmFin[8]  = vel8fin;
    velArmFin[10] =   20.0;
    velArmFin[15] =   40.0; // new

    double timing = 3.0;

    for (int times=0; times<repetitions; times++)
    {
        posArm->setRefSpeeds(velArmInit.data());
        posArm->positionMove(posArmInit.data());  

        Time::delay(timing);

        posArm->setRefSpeeds(velArmFin.data());
        posArm->positionMove(posArmFin.data());  

        Time::delay(timing*1.2); // increased in 2017
    }

    steerArmToHome();

    yInfo("...done thumbs down action");

    return true;
}
