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

    elements = 0;

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

Bottle DummyActivityInterfaceThread::queryUnderOf(const std::string &objName)
{
    Bottle replyList;
    replyList.clear();

    int id = -1;

    for (std::map<int, string>::reverse_iterator rit = onTopElements.rbegin();
         rit != onTopElements.rend();
         ++rit)
    {
        if (objName == rit->second)
            id = rit->first;
    }

    for (std::map<int, string>::reverse_iterator rit = onTopElements.rbegin();
         rit != onTopElements.rend();
         ++rit)
    {
        if ((objName.c_str() != rit->second) &&
            id >= 0 &&
            rit->first <= id)
        {
            replyList.addString(rit->second.c_str());
        }
    }

    return replyList;
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
             it != inHandStatus.end();
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
         it != inHandStatus.end();
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
    yInfo("trying to pull %s with %s", objName.c_str(), toolName.c_str());

    string handName = inHand(toolName);
    if (handName == "none")
    {
        yInfo("cannot pull %s with %s because I do not have %s in my hands",
              objName.c_str(), toolName.c_str(), toolName.c_str());

        return false;
    }

    /*
    Bottle initPos2D = get2D(objName);
    if (initPos2D.size()<1)
    {
        yError("problem parsing 2D position of %s", objName.c_str());
        return false;
    }

    Bottle initPos3D = get3D(objName);
    if (initPos3D.size()<1)
    {
        yError("problem parsing 3D position of %s", objName.c_str());
        return false;
    }
    */

    // do the pull action probabilistically
    bool success = true;
    if (success)
    {
        yInfo("successfully pulled %s with %s", objName.c_str(), toolName.c_str());
    }

    return true;
}

Bottle DummyActivityInterfaceThread::pullableWith(const string &objName)
{
    bool valid = false;
    for (int o=0; o<objNames.size(); ++o)
    {
        const string name = objNames.get(o).asString();
        if (objName == name)
        {
            valid = true;
        }
    }

    if (!valid)
    {
        yError("invalid object name %s", objName.c_str());
        Bottle empty;
        return empty;
    }

    return getToolLikeNames();
}

bool DummyActivityInterfaceThread::push(const string &objName, const string &toolName)
{
    yInfo("trying to push %s with %s", objName.c_str(), toolName.c_str());

    string handName = inHand(toolName);
    if (handName == "none")
    {
        yInfo("cannot push %s with %s because I do not have %s in my hands",
              objName.c_str(), toolName.c_str(), toolName.c_str());

        return false;
    }

    /*
    Bottle initPos2D = get2D(objName);
    if (initPos2D.size()<1)
    {
        yError("problem parsing 2D position of %s", objName.c_str());
        return false;
    }

    Bottle initPos3D = get3D(objName);
    if (initPos3D.size()<1)
    {
        yError("problem parsing 3D position of %s", objName.c_str());
        return false;
    }
    */

    // do the push action probabilistically
    bool success = true;
    if (success)
    {
        yInfo("successfully pushed %s with %s", objName.c_str(), toolName.c_str());
    }

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

    /*
    bool useStackedObjs = false;
    for (std::map<string, int>::const_iterator it = stackedObject.begin();
         it != stackedObject.end();
         ++it)
    {
        if (it->first == targetName)
        {
            useStackedObjs = true;
        }
    }
    */

    Bottle position = get3D(targetName);
    if (position.size()>0)
    {
        yInfo("OK, I will put %s on %s", objName.c_str(), targetName.c_str());

        Bottle under = underOf(targetName.c_str());

        // do the put action probabilistically
        bool success = true;
        if (success)
        {
            // update onTopElements
            if (!targetName.empty())
            {
                if (elements == 0)
                {
                    yDebug("elements is %d, adding <%d,%s>", elements, elements, targetName.c_str());
                    onTopElements.insert(pair<int, string>(elements, targetName.c_str()));
                    elements++;
                    yDebug("elements is %d", elements);
                }
                yDebug("adding <%d,%s>", elements, objName.c_str());
                onTopElements.insert(pair<int, string>(elements, objName.c_str()));
                elements++;
                yDebug("elements is %d, onTopElements has size %lu", elements, onTopElements.size());
            }

            // update inHandStatus
            for (std::map<string, string>::const_iterator it = inHandStatus.begin();
                 it != inHandStatus.end();
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

    bool valid = false;
    for (int o=0; o<objNames.size(); ++o)
    {
        const string name = objNames.get(o).asString();
        if (objName == name)
        {
            valid = true;
        }
    }

    if (!valid)
    {
        yError("invalid object name %s", objName.c_str());
        return replyList;
    }

    Bottle pos3D = get3D(objName);
    if (pos3D.size()<1)
    {
        yError("problem parsing 3D position of %s", objName.c_str());
    }
    else
    {
        const double xThresh = -0.48;
        if (pos3D.get(0).asDouble() < xThresh)
        {
            // in threshold

            Bottle list = pullableWith(objName);
            for (int i=0; i<list.size(); ++i)
                replyList.addString(list.get(i).asString());

            // check if tool is in left hand
            if (handStat("left"))
            {
                for (std::map<string, string>::const_iterator it = inHandStatus.begin();
                     it != inHandStatus.end();
                     ++it)
                {
                    if (it->second == "left")
                        replyList.addString(it->first.c_str());
                }
            }

            // check if tool is in right hand
            if (handStat("right"))
            {
                for (std::map<string, string>::const_iterator it = inHandStatus.begin();
                     it != inHandStatus.end();
                     ++it)
                {
                    if (it->second == "right")
                        replyList.addString(it->first.c_str());
                }
            }
        }
        else
        {
            // out of threshold

            Bottle list = getNames();

            for (int i=0; i<list.size(); ++i)
            {
                if (objName != list.get(i).asString())
                {
                    replyList.addString(list.get(i).asString());
                }
            }

            // check if tool is in left hand
            if (handStat("left"))
            {
                for (std::map<string, string>::const_iterator it = inHandStatus.begin();
                     it != inHandStatus.end();
                     ++it)
                {
                    if (it->second == "left")
                        replyList.addString(it->first.c_str());
                }
            }

            // check if tool is in right hand
            if (handStat("right"))
            {
                for (std::map<string, string>::const_iterator it = inHandStatus.begin();
                     it != inHandStatus.end();
                     ++it)
                {
                    if (it->second == "right")
                        replyList.addString(it->first.c_str());
                }
            }

            const double yThreshLeft  = -0.15;
            const double yThreshRight =  0.15;
            if (pos3D.get(1).asDouble() < yThreshLeft)
                replyList.addString("left");
            else if (pos3D.get(1).asDouble() > yThreshRight)
                replyList.addString("right");
            else
            {
                replyList.addString("left");
                replyList.addString("right");
            }
        }
    }

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
        // if classifyObserve
        // update underOf
        for (std::map<int, string>::const_iterator it = onTopElements.begin();
             it != onTopElements.end();
             ++it)
        {
            if (it->second == objName.c_str())
            {
                int id = it->first;
                onTopElements.erase(id);
                elements--;
            }
        }

        // update inHandStatus map
        inHandStatus.insert(pair<string, string>(objName.c_str(), handName.c_str()));
    }

    yInfo("took %s with %s", objName.c_str(), handName.c_str());

    return true;
}

Bottle DummyActivityInterfaceThread::underOf(const string &objName)
{
    bool valid = false;
    for (int o=0; o<objNames.size(); ++o)
    {
        const string name = objNames.get(o).asString();
        if (objName == name)
        {
            valid = true;
        }
    }

    if (!valid)
    {
        yError("invalid object name %s", objName.c_str());
        Bottle empty;
        return empty;
    }

    Bottle underOfObjects = queryUnderOf(objName);
    yDebug("first underOfObjects: %s", underOfObjects.toString().c_str());



    Bottle visibleObjects;
    for (int o=0; o<objNames.size(); ++o)
    {
        bool shouldAdd = true;

        // remove objects in hand
        const string handStatus = inHand(objNames.get(o).asString());
        if ((handStatus == "left") ||
            (handStatus == "right"))
        {
            shouldAdd = false;
        }

        // remove invisible objects
        for (int u=0; u<underOfObjects.size(); ++u)
        {
            if (underOfObjects.get(u).asString() == objNames.get(o).asString())
            {
                shouldAdd = false;
            }
        }

        if (shouldAdd)
        {
            yDebug("adding %s to visibleObjects", objNames.get(o).asString().c_str());
            visibleObjects.addString(objNames.get(o).asString());
        }
        else
            yDebug("NOT adding %s to visibleObjects", objNames.get(o).asString().c_str());
    }
    yDebug("visibleObjects: %s", visibleObjects.toString().c_str());

    for (int i=0; i<underOfObjects.size(); ++i)
    {
        for (int ii=0; ii<visibleObjects.size(); ++ii)
        {
            if (underOfObjects.get(i).asString() == visibleObjects.get(ii).asString())
            {
                for (std::map<int, string>::const_iterator it = onTopElements.begin();
                     it != onTopElements.end();
                     ++it)
                {
                    //yDebug("does %s equal %s?", it->second.c_str(), objName.c_str());
                    if (it->second == objName)
                    {
                        //yDebug("yes");
                        onTopElements.erase(it->first);
                        // elements-- ?
                        break;
                    }
                    //else
                    //    yDebug("no");
                }
            }
        }
    }

    underOfObjects.clear();
    underOfObjects = queryUnderOf(objName);
    yDebug("second underOfObjects: %s", underOfObjects.toString().c_str());

    return underOfObjects;
}
