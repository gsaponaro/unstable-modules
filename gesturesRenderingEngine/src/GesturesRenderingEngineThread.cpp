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

    yInfo("*** Homing head");

    gazeCtrl->lookAtFixationPoint(homeHead);
}

/**********************************************************/
GesturesRenderingEngineThread::GesturesRenderingEngineThread(
    const string &_moduleName,
    ResourceFinder &_rf)
    : RateThread(DefThreadPeriod), // initial default period
      moduleName(_moduleName),
      rf(_rf)
{
}

/**********************************************************/
void GesturesRenderingEngineThread::close()
{
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

    // open cartesiancontrollerclient and gazecontrollerclient drivers
    Property optGazeCtrl("(device gazecontrollerclient)");

    optGazeCtrl.put("remote","/iKinGazeCtrl");
    optGazeCtrl.put("local",("/"+moduleName+"/gaze").c_str());

    drvGazeCtrl = new PolyDriver;
    if (!drvGazeCtrl->open(optGazeCtrl))
    {
        close();
        return false;
    }

    // open views
    drvHead->view(encHead);
    drvGazeCtrl->view(gazeCtrl);

    // init
    int headAxes;
    encHead->getAxes(&headAxes);
    head.resize(headAxes, 0.0);

    steerHeadToHome();

    closing = false;

    return true;
}

/**********************************************************/
void GesturesRenderingEngineThread::run()
{
    if (!closing)
    {
        // TODO
    }
}

// IDL functions

/**********************************************************/
bool GesturesRenderingEngineThread::do_nod()
{
    steerHeadToHome();

    return true;
}

/**********************************************************/
bool GesturesRenderingEngineThread::do_punch()
{
    return true;
}

/**********************************************************/
bool GesturesRenderingEngineThread::do_lookout()
{
    return true;
}

/**********************************************************/
bool GesturesRenderingEngineThread::do_thumbsup()
{
    return true;
}

/**********************************************************/
bool GesturesRenderingEngineThread::do_thumbsdown()
{
    return true;
}
