/*
 * Copyright: (C) 2012-2015 POETICON++, European Commission FP7 project ICT-288382
 * Copyright: (C) 2016 VisLab, Institute for Systems and Robotics,
 *                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 * Based on activityInterface by Vadim Tikhanoff
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
    ret = ret && rpcMemory.open(("/"+moduleName+"/memory:rpc").c_str());

    return ret;
}

/**********************************************************/
void DummyActivityInterfaceThread::close()
{
    yInfo("closing ports");
    rpcMemory.close();

    return;
}

/**********************************************************/
void DummyActivityInterfaceThread::interrupt()
{
    closing = true;
    yInfo("interrupting ports");
    rpcMemory.interrupt();

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

/**********************************************************/
Bottle DummyActivityInterfaceThread::getMemoryBottle()
{
    if (! memoryConnected())
    {
        Bottle empty;
        return empty;
    }

    Bottle memoryReply;
    Bottle cmdMemory;
    Bottle replyMemory;
    Bottle replyMemoryProp;
    memoryReply.clear();
    cmdMemory.clear();
    replyMemory.clear();
    replyMemoryProp.clear();

    cmdMemory.addVocab(Vocab::encode("ask"));
    Bottle &cont = cmdMemory.addList().addList();
    cont.addString("entity");
    cont.addString("==");
    cont.addString("object");

    LockGuard lg(mutex);
    //yDebug() << __func__ << "sending command to OPC:" << cmdMemory.toString().c_str();
    rpcMemory.write(cmdMemory,replyMemory);
    //yDebug() << __func__ << "received response:" << replyMemory.toString().c_str();

    if (replyMemory.get(0).asVocab() == Vocab::encode("ack"))
    {
        if (Bottle *idField = replyMemory.get(1).asList())
        {
            if (Bottle *idValues = idField->get(1).asList())
            {
                //cycle over items
                for (int i=0; i<idValues->size(); ++i)
                {
                    int id = idValues->get(i).asInt();

                    /*
                    Bottle cmdTime;
                    Bottle cmdReply;
                    cmdTime.addVocab(Vocab::encode("time"));
                    Bottle &contmp = cmdTime.addList();
                    Bottle &list_tmp = contmp.addList();
                    list_tmp.addString("id");
                    list_tmp.addInt(id);
                    //yDebug() << __func__ << "sending command to OPC:" << cmdTime.toString().c_str();
                    rpcMemory.write(cmdTime,cmdReply);
                    //yDebug() << __func__ << "received response:" << cmdReply.toString().c_str();

                    Bottle *timePassed = cmdReply.get(1).asList();
                    double time = timePassed->get(0).asDouble();
                    */

                    //if (time < 1.0)
                    //{
                    cmdMemory.clear();
                    cmdMemory.addVocab(Vocab::encode("get"));
                    Bottle &content = cmdMemory.addList();
                    Bottle &list_bid = content.addList();
                    list_bid.addString("id");
                    list_bid.addInt(id);
                    //yDebug() << __func__ << "sending command to OPC:" << cmdMemory.toString().c_str();
                    rpcMemory.write(cmdMemory,replyMemoryProp);
                    //yDebug() << __func__ << "received response:" << replyMemoryProp.toString().c_str();

                    memoryReply.addList() = *replyMemoryProp.get(1).asList();
                    //}
                }
            }
        }
    }

    return memoryReply;
}

/**********************************************************/
Bottle DummyActivityInterfaceThread::getToolLikeNames()
{
    Bottle names = getNames();
    Bottle toolLikeNames;

    for (int o=0; o<names.size(); ++o)
    {
        if (names.get(o).asString() == "Rake" ||
            names.get(o).asString() == "Stick")
        {
            toolLikeNames.addString(names.get(o).asString().c_str());
        }
    }

    return toolLikeNames;
}

/**********************************************************/
bool DummyActivityInterfaceThread::memoryConnected()
{
    return (rpcMemory.getOutputCount() > 0);
}

/**********************************************************/
int DummyActivityInterfaceThread::name2id(const std::string &objName)
{
    int id = -1;

    if (! memoryConnected())
    {
        return id;
    }

    Bottle memoryReply;
    Bottle cmdMemory;
    Bottle replyMemory;
    Bottle replyMemoryProp;

    cmdMemory.addVocab(Vocab::encode("ask"));
    Bottle &cont = cmdMemory.addList().addList();
    cont.addString("name");
    cont.addString("==");
    cont.addString(objName);

    LockGuard lg(mutex);
    rpcMemory.write(cmdMemory,replyMemory);

    // valid reply: [ack] (id (11))
    // invalid reply: [ack] (id ())
    bool valid = replyMemory.size()>1 &&
                 replyMemory.get(0).asVocab()==Vocab::encode("ack") &&
                 replyMemory.get(1).isList() &&
                 replyMemory.get(1).asList()->size()==2 &&
                 replyMemory.get(1).asList()->get(0)=="id" &&
                 replyMemory.get(1).asList()->get(1).isList() &&
                 replyMemory.get(1).asList()->get(1).asList()->size()>0; // the important condition

    if (valid)
        id = replyMemory.get(1).asList()->get(1).asList()->get(0).asInt();
    else
        yWarning("did not find ID for %s", objName.c_str());

    return id;
}

/**********************************************************/
bool DummyActivityInterfaceThread::setObjProperty(const std::string &objName,
                                                  const std::string &prop,
                                                  const yarp::os::Bottle &v)
{
    if (! validateName(objName))
        return false;

    Bottle opcCmd;
    Bottle opcCmdContent;
    Bottle opcReply;

    // [set] (("id" <num>) ("prop0" <val0>) ...)
    opcCmd.clear();
    opcCmdContent.clear();
    opcReply.clear();
    opcCmd.addVocab(Vocab::encode("set"));

    Bottle bID;
    bID.clear();
    bID.addString("id");
    bID.addInt(name2id(objName));
    opcCmdContent.addList() = bID;

    Bottle bPropAndValue;
    bPropAndValue.addString(prop);
    bPropAndValue.addList() = v;
    opcCmdContent.addList() = bPropAndValue;
    opcCmd.addList() = opcCmdContent;

    LockGuard lg(mutex);
    //yDebug() << __func__ << "sending command to OPC:" << opcCmd.toString().c_str();
    rpcMemory.write(opcCmd, opcReply);
    //yDebug() << __func__ << "received response:" << opcReply.toString().c_str();

    bool valid = opcReply.size()>0 &&
                 opcReply.get(0).asVocab()==Vocab::encode("ack");

    return valid;
}

/**********************************************************/
bool DummyActivityInterfaceThread::validate2D(const string &objName)
{
    Bottle pos2D = get2D(objName);
    if (pos2D.size()<1)
    {
        yError("problem parsing 2D position of %s", objName.c_str());
        return false;
    }

    return true;
}

/**********************************************************/
bool DummyActivityInterfaceThread::validate3D(const string &objName)
{
    Bottle pos3D = get3D(objName);
    if (pos3D.size()<1)
    {
        yError("problem parsing 3D position of %s", objName.c_str());
        return false;
    }

    return true;
}

/**********************************************************/
bool DummyActivityInterfaceThread::validateName(const string &objName)
{
    bool valid = false;

    Bottle names = getNames();
    for (int o=0; o<names.size(); ++o)
    {
        const string currName = names.get(o).asString();
        if (objName == currName)
        {
            valid = true;
        }
    }

    if (!valid)
        yWarning("invalid object name %s", objName.c_str());

    return valid;
}

/**********************************************************/
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
        if ((objName != rit->second) &&
            id >= 0 &&
            rit->first <= id)
        {
            replyList.addString(rit->second.c_str());
        }
    }

    return replyList;
}

// IDL functions
/**********************************************************/
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

    yDebug("tato (take tool)");

    //update inHandStatus map
    inHandStatus.insert(pair<string, string>(label.c_str(), handName.c_str()));

    if (availableTools.size()<1)
    {
        availableTools.push_back(label.c_str());
        yDebug() << __func__ << "adding" << label.c_str() << "to list";
    }
    else
    {
        if (std::find(availableTools.begin(), availableTools.end(), label/*.c_str()*/) == availableTools.end())
        {
            yDebug() << __func__ << "name" << label.c_str() << "not available";
            yDebug() << __func__ << "adding it to list";
            availableTools.push_back(label.c_str());
        }
    }

    return true;
}

/**********************************************************/
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

/**********************************************************/
Bottle DummyActivityInterfaceThread::get2D(const string &objName)
{
    if (! memoryConnected())
    {
        Bottle empty;
        return empty;
    }

    Bottle memory = getMemoryBottle();
    Bottle position2D;

    for (int i=0; i<memory.size(); ++i)
    {
        if (Bottle *propField = memory.get(i).asList())
        {
            if (propField->check("name"))
            {
                if (propField->find("name").asString() == objName)
                {
                    if (propField->check("position_2d_left"))
                    {
                        Bottle *propFieldPos = propField->find("position_2d_left").asList();
                        
                        for (int ii=0; ii< propFieldPos->size(); ++ii)
                        {
                            position2D.addDouble(propFieldPos->get(ii).asDouble());
                        }
                    }
                }
            }
        }
    }

    return position2D;
}

/**********************************************************/
Bottle DummyActivityInterfaceThread::get3D(const string &objName)
{
    if (! memoryConnected())
    {
        Bottle empty;
        return empty;
    }

    Bottle memory = getMemoryBottle();
    Bottle position3D;

    for (int i=0; i<memory.size(); ++i)
    {
        if (Bottle *propField = memory.get(i).asList())
        {
            if (propField->check("name"))
            {
                if (propField->find("name").asString() == objName)
                {
                    if (propField->check("position_3d"))
                    {
                        Bottle *propFieldPos = propField->find("position_3d").asList();

                        for (int i2=0; i2<propFieldPos->size(); ++i2)
                        {
                            position3D.addDouble(propFieldPos->get(i2).asDouble());
                        }
                    }
                }
            }
        }
    }

    return position3D;
}

/**********************************************************/
string DummyActivityInterfaceThread::getLabel(const int32_t xpos,
                                              const int32_t ypos)
{
    string label;

    if (! memoryConnected())
    {
        return label;
    }

    Bottle memory = getMemoryBottle();

    for (int i=0; i<memory.size(); ++i)
    {
        if (Bottle *propField = memory.get(i).asList())
        {
            if (propField->check("position_2d_left"))
            {
                Bottle *propFieldPos = propField->find("position_2d_left").asList();

                if (propFieldPos->get(0).asDouble() < xpos &&
                    xpos < propFieldPos->get(2).asDouble() &&
                    propFieldPos->get(1).asDouble() < ypos &&
                    ypos < propFieldPos->get(3).asDouble())
                {
                    label = propField->find("name").asString().c_str();
                    break;
                }
            }
        }
    }

    return label;
}

/**********************************************************/
Bottle DummyActivityInterfaceThread::getNames()
{
    Bottle names;

    if (! memoryConnected())
    {
        return names;
    }

    Bottle memory = getMemoryBottle();
    //yDebug("%d entries, full memory: %s", memory.size(), memory.toString().c_str());

    for (int i=0; i<memory.size(); ++i)
    {
        if (Bottle *propField = memory.get(i).asList())
        {
            //yDebug("i=%d, %s", i, propField->toString().c_str());
            if (propField->check("position_2d_left"))
            {
                if (propField->check("name"))
                {
                    //yDebug("adding the name of %s to names=%s", propField->find("name").asString().c_str(), names.toString().c_str());
                    names.addString(propField->find("name").asString());
                    //yDebug("now names=%s", names.toString().c_str());
                }
            }
        }
    }

    return names;
}

/**********************************************************/
bool DummyActivityInterfaceThread::goHome()
{
    return true;
}

/**********************************************************/
bool DummyActivityInterfaceThread::handStat(const string &handName)
{
    if (handName != "left" && handName != "right")
    {
        yError("hand name %s not recognized: must be left or right", handName.c_str());
        return false;
    }

    bool handIsFull = false;

    Bottle names = getNames();
    for (int o=0; o<names.size(); ++o)
    {
        const string handStatus = inHand(names.get(o).asString());
        if (handStatus == handName)
        {
            handIsFull = true;
        }
    }

    return handIsFull;
}

/**********************************************************/
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

/**********************************************************/
bool DummyActivityInterfaceThread::pull(const string &objName, const string &toolName)
{
    if (! memoryConnected())
    {
        return false;
    }

    yInfo("trying to pull %s with %s", objName.c_str(), toolName.c_str());

    string handName = inHand(toolName);
    if (handName == "none")
    {
        yInfo("cannot pull %s with %s because I do not have %s in my hands",
              objName.c_str(), toolName.c_str(), toolName.c_str());

        return false;
    }

    if (! validate2D(objName))
        return false;

    if (! validate3D(objName))
        return false;

    // do the pull action probabilistically
    bool success = true;
    if (success)
    {
        Bottle initPos3D = get3D(objName);
        Bottle finPos3D;
        const double displacementX = 0.10;
        finPos3D.addDouble(initPos3D.get(0).asDouble() + displacementX);
        finPos3D.addDouble(initPos3D.get(1).asDouble());
        finPos3D.addDouble(initPos3D.get(2).asDouble());

        setObjProperty(objName, "position_3d", finPos3D);

        yInfo("successfully pulled %s with %s", objName.c_str(), toolName.c_str());
    }

    return true;
}

/**********************************************************/
Bottle DummyActivityInterfaceThread::pullableWith(const string &objName)
{
    Bottle names = getNames();

    if (! validateName(objName))
    {
        Bottle empty;
        return empty;
    }

    return getToolLikeNames();
}

/**********************************************************/
bool DummyActivityInterfaceThread::push(const string &objName, const string &toolName)
{
    if (! memoryConnected())
    {
        return false;
    }

    yInfo("trying to push %s with %s", objName.c_str(), toolName.c_str());

    string handName = inHand(toolName);
    if (handName == "none")
    {
        yInfo("cannot push %s with %s because I do not have %s in my hands",
              objName.c_str(), toolName.c_str(), toolName.c_str());

        return false;
    }

    if (! validate2D(objName))
        return false;

    if (! validate3D(objName))
        return false;

    // do the push action probabilistically
    bool success = true;
    if (success)
    {
        Bottle initPos3D = get3D(objName);
        Bottle finPos3D;
        const double displacementX = 0.10;
        finPos3D.addDouble(initPos3D.get(0).asDouble() - displacementX);
        finPos3D.addDouble(initPos3D.get(1).asDouble());
        finPos3D.addDouble(initPos3D.get(2).asDouble());

        setObjProperty(objName, "position_3d", finPos3D);

        yInfo("successfully pushed %s with %s", objName.c_str(), toolName.c_str());
    }

    return true;
}

/**********************************************************/
bool DummyActivityInterfaceThread::put(const string &objName, const string &targetName)
{
    if (! memoryConnected())
    {
        return false;
    }

    yInfo("trying to put %s on %s", objName.c_str(), targetName.c_str());

    string handName = inHand(objName);
    if (handName == "none")
    {
        yInfo("cannot put %s on %s because I do not have %s in my hands",
              objName.c_str(), targetName.c_str(), objName.c_str());

        return false;
    }

    if (! validate2D(targetName))
        return false;

    if (! validate3D(targetName))
        return false;

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

/**********************************************************/
Bottle DummyActivityInterfaceThread::reachableWith(const string &objName)
{
    Bottle replyList;

    if (! validateName(objName))
        return replyList;

    if (! validate3D(objName))
        return replyList;

    Bottle pos3D = get3D(objName);
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


    return replyList;
}

/**********************************************************/
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

/**********************************************************/
Bottle DummyActivityInterfaceThread::underOf(const string &objName)
{
    if (! validateName(objName))
    {
        Bottle empty;
        return empty;
    }

    Bottle underOfObjects = queryUnderOf(objName);
    yDebug("first underOfObjects: %s", underOfObjects.toString().c_str());



    Bottle visibleObjects;
    /*
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
    */

    return underOfObjects;
}
