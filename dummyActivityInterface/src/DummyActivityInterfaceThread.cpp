/*
 * Copyright: (C) 2012-2015 POETICON++, European Commission FP7 project ICT-288382
 * Copyright: (C) 2016 VisLab, Institute for Systems and Robotics,
 *                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#include "DummyActivityInterfaceThread.h"

/**********************************************************/
DummyActivityInterfaceThread::DummyActivityInterfaceThread(
    const string &_moduleName,
    const double _period)
    : moduleName(_moduleName),
      RateThread(int(_period*1000.0))
{
}

/**********************************************************/
bool DummyActivityInterfaceThread::openPorts()
{
    bool ret = true;

    return ret;
}

/**********************************************************/
void DummyActivityInterfaceThread::close()
{
    yInfo("closing ports");

    return;
}

/**********************************************************/
void DummyActivityInterfaceThread::interrupt()
{
    closing = true;

    yInfo("interrupting ports");

    return;
}

/**********************************************************/
bool DummyActivityInterfaceThread::threadInit()
{
    closing = false;

    if ( !openPorts() )
    {
        yError("problem opening ports");
        return false;
    }

    return true;
}

/**********************************************************/
void DummyActivityInterfaceThread::run()
{
    while (!closing)
        mainProcessing();
}

/**********************************************************/
void DummyActivityInterfaceThread::mainProcessing()
{
    yarp::os::Time::delay(0.01);

    if (closing)
        return;
}

// IDL functions
bool DummyActivityInterfaceThread::askForTool(const string &handName,
                                              const int32_t xpos,
                                              const int32_t ypos)
{
    return true;
}

bool DummyActivityInterfaceThread::drop(const string &objName)
{
    return true;
}

Bottle DummyActivityInterfaceThread::get2D(const string &objName)
{
    Bottle position2D;

    return position2D;
}

string DummyActivityInterfaceThread::getLabel(const int32_t xpos,
                                              const int32_t ypos)
{
    return "test";
}

Bottle DummyActivityInterfaceThread::getNames()
{
    Bottle names;

    return names;
}

bool DummyActivityInterfaceThread::goHome()
{
    return true;
}

bool DummyActivityInterfaceThread::handStat(const string &handName)
{
    return true;
}

string DummyActivityInterfaceThread::inHand(const std::string &objName)
{
    return "none";
}

bool DummyActivityInterfaceThread::pull(const string &objName, const string &toolName)
{
    return true;
}

Bottle DummyActivityInterfaceThread::pullableWith(const string &objName)
{
    Bottle replyList;

    return replyList;
}

bool DummyActivityInterfaceThread::push(const string &objName, const string &toolName)
{
    return true;
}

bool DummyActivityInterfaceThread::put(const string &objName, const string &targetName)
{
    return true;
}

Bottle DummyActivityInterfaceThread::reachableWith(const string &objName)
{
    Bottle replyList;

    return replyList;
}

bool DummyActivityInterfaceThread::take(const string &objName, const string &handName)
{
    return true;
}

Bottle DummyActivityInterfaceThread::underOf(const string &objName)
{
    Bottle replyList;

    return replyList;
}
