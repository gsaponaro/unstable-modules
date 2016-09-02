/*
 * Copyright: (C) 2012-2015 POETICON++, European Commission FP7 project ICT-288382
 * Copyright: (C) 2016 VisLab, Institute for Systems and Robotics,
 *                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#include "DummyActivityInterfaceModule.h"

bool DummyActivityInterfaceModule::configure(ResourceFinder &rf)
{
    // module parameters
    moduleName = rf.check("name", Value("dummyActivityInterface")).asString();
    setName(moduleName.c_str());

    handlerPortName = "/" + moduleName + "/rpc:i";
    handlerPort.open(handlerPortName.c_str());
    attach(handlerPort);

    closing = false;

    // thread stuff
    threadPeriod = 0.033; // [s]

    // create new thread and pass pointers to the module parameters
    thread = new DummyActivityInterfaceThread(moduleName,
                                              threadPeriod);

    // start the thread to do the work
    if (!thread->start())
    {
        delete thread;
        return false;
    }

    return true;
}

bool DummyActivityInterfaceModule::interruptModule()
{
    handlerPort.interrupt();

    return true;
}

bool DummyActivityInterfaceModule::close()
{
    yInfo("closing RPC port");
    handlerPort.close();

    yInfo("starting shutdown procedure");
    thread->interrupt();
    thread->close();
    thread->stop();
    yInfo("deleting thread");
    if (thread) delete thread;
    yInfo("done deleting thread");

    return true;
}

bool DummyActivityInterfaceModule::updateModule()
{
    return !closing;
}

double DummyActivityInterfaceModule::getPeriod()
{
    return 0.0;
}

// IDL functions
bool DummyActivityInterfaceModule::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}

bool DummyActivityInterfaceModule::askForTool(const string &handName,
                                              const int32_t xpos,
                                              const int32_t ypos)
{
    return thread->askForTool(handName, xpos, ypos);
}

bool DummyActivityInterfaceModule::drop(const string &objName)
{
    return thread->drop(objName);
}

Bottle DummyActivityInterfaceModule::get2D(const string &objName)
{
    return thread->get2D(objName);
}

string DummyActivityInterfaceModule::getLabel(const int32_t xpos, const int32_t ypos)
{
    return thread->getLabel(xpos, ypos);
}

Bottle DummyActivityInterfaceModule::getNames()
{
    return thread->getNames();
}

bool DummyActivityInterfaceModule::goHome()
{
    return thread->goHome();
}

bool DummyActivityInterfaceModule::handStat(const string &handName)
{
    return thread->handStat(handName);
}

string DummyActivityInterfaceModule::inHand(const std::string &objName)
{
    return thread->inHand(objName);
}

bool DummyActivityInterfaceModule::pull(const string &objName, const string &toolName)
{
    return thread->pull(objName, toolName);
}

Bottle DummyActivityInterfaceModule::pullableWith(const string &objName)
{
    return thread->pullableWith(objName);
}

bool DummyActivityInterfaceModule::push(const string &objName, const string &toolName)
{
    return thread->push(objName, toolName);
}

bool DummyActivityInterfaceModule::put(const string &objName, const string &targetName)
{
    return thread->put(objName, targetName);
}

Bottle DummyActivityInterfaceModule::reachableWith(const string &objName)
{
    return thread->reachableWith(objName);
}

bool DummyActivityInterfaceModule::take(const string &objName, const string &handName)
{
    return thread->take(objName, handName);
}

Bottle DummyActivityInterfaceModule::underOf(const string &objName)
{
    return thread->underOf(objName);
}

bool DummyActivityInterfaceModule::quit()
{
    yInfo("quitting");
    closing = true;

    return true;
}
