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
void GesturesRenderingEngineThread::steerHeadToHome()
{
    Vector homeHead(3);

    homeHead[0] = -1.0;
    homeHead[1] =  0.0;
    homeHead[2] =  0.3;

    //yInfo("*** Homing head");

    gazeCtrl->lookAtFixationPoint(homeHead);
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
    //modeHead = NULL;
    //drvLeftArm = NULL;
}

/**********************************************************/
void GesturesRenderingEngineThread::close()
{
    //gazeCtrl->restoreContext(startup_context_id_gaze); // TODO: move to threadRelease()

    if (drvHead) delete drvHead;
    if (drvGazeCtrl) delete drvGazeCtrl;

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
    repetitions = 2;

    // open remote_controlboard drivers
    Property optHead("(device remote_controlboard)");
    optHead.put("remote",("/"+robotName+"/head").c_str());
    optHead.put("local",("/"+moduleName+"/head").c_str());

    drvHead = new PolyDriver;
    if (!drvHead->open(optHead))
    {
        close();
        return false;
    }

    // open gazecontrollerclient and cartesiancontrollerclient drivers
    Property optGazeCtrl("(device gazecontrollerclient)");
    optGazeCtrl.put("remote","/iKinGazeCtrl");
    optGazeCtrl.put("local",("/"+moduleName+"/gaze").c_str());

    drvGazeCtrl = new PolyDriver;
    if (!drvGazeCtrl->open(optGazeCtrl))
    {
        close();
        return false;
    }

    /*
    Property optCartLeftArm("(device cartesiancontrollerclient)");
    optCartLeftArm.put("remote",("/"+robotName+"/cartesianController/left_arm").c_str());
    optCartLeftArm.put("local",("/"+moduleName+"/left_arm/cartesian").c_str());

    Property optCartRightArm("(device cartesiancontrollerclient)");
    optCartRightArm.put("remote",("/"+robotName+"/cartesianController/right_arm").c_str());
    optCartRightArm.put("local",("/"+moduleName+"/right_arm/cartesian").c_str());
    */

    // open views
    bool ok = true;
    ok = ok && drvHead->view(encHead);
    ok = ok && drvHead->view(headPosCtrl);
    //ok = ok && drvHead->view(modeHead);

    ok = ok && drvGazeCtrl->view(gazeCtrl);

    if (!ok)
    {
        yError("problem acquiring interfaces");
        return false;
    }

    /*
    if (drvGazeCtrl->isValid())
    {
        drvGazeCtrl->view(gazeCtrl);
    }
    else
    {
        yError("problem with gaze interface when obtaining a view");
        return false;
    }
    */

    if (gazeCtrl == NULL)
    {
        yError("problem with gaze interface when initializing IGazeControl");
        return false;
    }

    // init
    int headJoints;
    encHead->getAxes(&headJoints);
    head.resize(headJoints, 0.0);

    //for(int i=0; i<headJoints; i++)
    //    modeHead->setControlMode(i,VOCAB_CM_MIXED);

    Vector velHead(headJoints);
    velHead = 10.0;
    headPosCtrl->setRefSpeeds(velHead.data());

    //gazeCtrl->storeContext(&startup_context_id_gaze);
    //gazeCtrl->restoreContext(0);
    gazeCtrl->setNeckTrajTime(2.0);
    gazeCtrl->setEyesTrajTime(1.0);
    gazeCtrl->setTrackingMode(false); // tracking mode: torso moves => gaze compensates
    //gazeCtrl->setSaccadesMode(false);
    //gazeCtrl->setStabilizationMode(false);
    //Bottle info;
    //gazeCtrl->getInfo(info);
    //fprintf(stdout,"gaze info = %s\n",info.toString().c_str());

    steerHeadToHome();

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

    // read current joint values and save them in head variable
    encHead->getEncoders(head.data());

    double init_j0 = head[0];

    // parameter
    double final_j0 = -35.0;

    double t_j0 = 2.0;

    for (int times=0; times<repetitions; times++)
    {
        head[0] = init_j0;
        headPosCtrl->positionMove(head.data());

        Time::delay(t_j0);

        head[0] = final_j0;
        headPosCtrl->positionMove(head.data());

        Time::delay(t_j0);
    }

//  Time::delay(t_j0);
//  steerHeadToHome();
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

    yInfo("...done");

    return true;
}

/**********************************************************/
bool GesturesRenderingEngineThread::do_punch()
{
    LockGuard lg(mutex);
    yInfo("doing punch action...");



    yInfo("...done");

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

    /*
    bool done = false;
    double t0 = Time::now();
    const double timeout = 5.0;
    while (!done && (Time::now()-t0<timeout))
    {
        headPosCtrl->checkMotionDone(&done);
        Time::delay(0.1);
    }
    */

    yInfo("...done");

    return true;
}

/**********************************************************/
bool GesturesRenderingEngineThread::do_thumbsup()
{
    LockGuard lg(mutex);
    yInfo("doing thumbs up action...");

    yInfo("...done");

    return true;
}

/**********************************************************/
bool GesturesRenderingEngineThread::do_thumbsdown()
{
    LockGuard lg(mutex);
    yInfo("doing thumbs down action...");

    yInfo("...done");

    return true;
}
