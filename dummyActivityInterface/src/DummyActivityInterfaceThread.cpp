/*
 * Copyright: (C) 2012-2015 POETICON++, European Commission FP7 project ICT-288382
 * Copyright: (C) 2016 VisLab, Institute for Systems and Robotics,
 *                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#include "DummyActivityInterfaceThread.h"

using namespace std;
using namespace yarp::os;

/**********************************************************/
DummyActivityInterfaceThread::DummyActivityInterfaceThread(
    const string &_moduleName,
    ResourceFinder &_rf)
    : RateThread(33), // [ms]
      moduleName(_moduleName),
      rf(_rf)
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

    // parse ini file
    if (rf.check("objects_list"))
    {
        objNames = * rf.find("objects_list").asList();
        if (objNames.isNull() || objNames.size()<1)
            yError("problem with objects_list");
        else
        {
            yInfo("objects_list: %s", objNames.toString().c_str());

            for (int o=0; o<objNames.size(); ++o)
            {
                const string name = objNames.get(o).asString();
                if (! rf.findGroup("OBJECTS").check(name))
                {
                    yError("did not find %s in OBJECTS group", name.c_str());
                }
            }
        }
    }
    else
        yError("did not find objects_list");

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

    if (rf.findGroup("OBJECTS").check(objName) &&
        rf.findGroup("OBJECTS").find(objName).asList()->check("position_2d_left"))
    {
        position2D = * rf.findGroup("OBJECTS").find(objName).asList()->find("position_2d_left").asList();
    }
    else
        yError("problem parsing 2D position of %s", objName.c_str());

    return position2D;
}

string DummyActivityInterfaceThread::getLabel(const int32_t xpos,
                                              const int32_t ypos)
{
    string label;
    Bottle positionBBox;

    Bottle objs = rf.findGroup("OBJECTS");

    for (int i=0; i<objs.size(); ++i)
    {
        if (Bottle *entry = objs.get(i).asList())
        {
            string entryName = entry->get(0).asString();
            if (Bottle *propField = entry->get(1).asList())
            {
                if (propField->check("position_2d_left"))
                {
                    Bottle *propFieldPos = propField->find("position_2d_left").asList();
                    if (propFieldPos->get(0).asDouble() < xpos &&
                        xpos < propFieldPos->get(2).asDouble() &&
                        propFieldPos->get(1).asDouble() < ypos &&
                        ypos < propFieldPos->get(3).asDouble())
                    {
                        label = entryName;
                        break;
                    }
                }
            }
        }
    }

    return label;
}

Bottle DummyActivityInterfaceThread::getNames()
{
    return objNames;
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
