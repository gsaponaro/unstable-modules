/*
 * Copyright: (C) 2012-2015 POETICON++, European Commission FP7 project ICT-288382
 * Copyright: (C) 2016 VisLab, Institute for Systems and Robotics,
 *                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#include <algorithm>
#include <vector>

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

Bottle DummyActivityInterfaceThread::getToolLikeNames()
{
    Bottle toolLikeNames;

    for (int o=0; o<objNames.size(); ++o)
    {
        if (objNames.get(o).asString() == "Rake" ||
            objNames.get(o).asString() == "Stick")
        {
            toolLikeNames.addString(objNames.get(o).asString().c_str());
        }
    }

    return toolLikeNames;
}

// IDL functions
bool DummyActivityInterfaceThread::askForTool(const string &handName,
                                              const int32_t xpos,
                                              const int32_t ypos)
{
    // requested object
    string label = getLabel(xpos, ypos);

    if (label.empty())
    {
        yInfo("I cannot see anything at the position %d %d", xpos, ypos);
        return false;
    }

    yInfo("tato (take tool)");

    //update inHandStatus map
    inHandStatus.insert(pair<string, string>(label.c_str(), handName.c_str()));

    if (availableTools.size()<1)
    {
        availableTools.push_back(label.c_str());
        yInfo("[askForTool] adding %s to list", label.c_str());
    }
    else
    {
        if (std::find(availableTools.begin(), availableTools.end(), label/*.c_str()*/) == availableTools.end())
        {
            yInfo("[askForTool] name %s not available", label.c_str());
            yInfo("[askForTool] adding it to list");
            availableTools.push_back(label.c_str());
        }
    }

    return true;
}

bool DummyActivityInterfaceThread::drop(const string &objName)
{
    string handName = inHand(objName);

    if (handName == "none")
    {
        yInfo("cannot drop %s because it is not in my hands", objName.c_str());
    }

    yInfo("OK, I will drop the %s", objName.c_str());
    // do the drop action probabilistically
    bool success = true;
    if (success)
    {
        // update inHandStatus map
        for (std::map<string, string>::const_iterator it = inHandStatus.begin();
             it!=inHandStatus.end();
             ++it)
        {
            if (it->first == objName)
            {
                inHandStatus.erase(objName.c_str());
                break;
            }
        }
    }

    yInfo("dropped %s", objName.c_str());

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

Bottle DummyActivityInterfaceThread::get3D(const string &objName)
{
    Bottle position3D;

    if (rf.findGroup("OBJECTS").check(objName) &&
        rf.findGroup("OBJECTS").find(objName).asList()->check("position_3d"))
    {
        position3D = * rf.findGroup("OBJECTS").find(objName).asList()->find("position_3d").asList();
    }
    else
        yError("problem parsing 3D position of %s", objName.c_str());

    return position3D;
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
    if (handName != "left" && handName != "right")
    {
        yError("hand name %s not recognized: must be left or right", handName.c_str());
        return false;
    }

    bool handIsFull = false;

    for (int o=0; o<objNames.size(); ++o)
    {
        const string handStatus = inHand(objNames.get(o).asString());
        if (handStatus == handName)
        {
            handIsFull = true;
        }
    }

    return handIsFull;
}

string DummyActivityInterfaceThread::inHand(const std::string &objName)
{
    string handName;
    
    for (std::map<string, string>::const_iterator it = inHandStatus.begin();
         it!=inHandStatus.end();
         ++it)
    {
        if (it->first == objName)
            handName = it->second.c_str();
    }
    if (handName.empty())
        handName = "none";
        
    return handName;
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
    yInfo("trying to put %s on %s", objName.c_str(), targetName.c_str());

    string handName = inHand(objName);

    if (handName == "none")
    {
        yInfo("cannot put %s on %s because I do not have %s in my hands",
              objName.c_str(), targetName.c_str(), objName.c_str());

        return false;
    }

    Bottle position = get3D(targetName);
    if (position.size()>0)
    {
        yInfo("OK, I will put %s on %s", objName.c_str(), targetName.c_str());

        Bottle under = underOf(targetName.c_str());

        // do the put action probabilistically
        bool success = true;
        if (success)
        {
            // update onTopElements and inHandStatus
            if (!targetName.empty())
            {
                if (elements == 0)
                {
                    onTopElements.insert(pair<int, string>(elements, targetName.c_str()));
                    elements++;
                }
                onTopElements.insert(pair<int, string>(elements, objName.c_str()));
                elements++;
            }
            
            for (std::map<string, string>::const_iterator it = inHandStatus.begin();
                 it!=inHandStatus.end();
                 ++it)
            {
                if (it->first == objName)
                {
                    inHandStatus.erase(objName.c_str());
                    break;
                }
            }
        }
    }

    yInfo("finished putting %s on %s", objName.c_str(), targetName.c_str());

    return true;
}

Bottle DummyActivityInterfaceThread::reachableWith(const string &objName)
{
    Bottle replyList;

    return replyList;
}

bool DummyActivityInterfaceThread::take(const string &objName, const string &handName)
{
    if (handName != "left" && handName != "right")
    {
        yError("hand name %s not recognized: must be left or right", handName.c_str());
        return false;
    }

    yInfo("trying to take %s with %s", objName.c_str(), handName.c_str());

    //check for hand status beforehand to make sure that it is empty
    string handStatus = inHand(objName);

    if (handStatus != "none")
    {
        yInfo("cannot take %s with %s because %s is full",
              objName.c_str(), handName.c_str(), handName.c_str());

        return false;
    }

    // do the take action probabilistically
    bool success = true;
    if (success)
    {
        // update inHandStatus map
        inHandStatus.insert(pair<string, string>(objName.c_str(), handName.c_str()));
    }

    yInfo("took %s with %s", objName.c_str(), handName.c_str());

    return true;
}

Bottle DummyActivityInterfaceThread::underOf(const string &objName)
{
    Bottle replyList;

    return replyList;
}
