/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 * Based on actionsRenderingEngine by Carlo Ciliberto, Vadim Tikhanoff
 *
 */

#include <yarp/os/Log.h>

#include "GesturesRenderingEngineDefaults.h"
#include "GesturesRenderingEngineModule.h"

using namespace std;
using namespace yarp::os;

bool GesturesRenderingEngineModule::configure(ResourceFinder &rf)
{
    // module parameters
    moduleName = rf.check("name", Value("gesturesRenderingEngine")).asString();
    setName(moduleName.c_str());

    handlerPortName = "/" + moduleName + "/rpc:i";
    handlerPort.open(handlerPortName.c_str());
    attach(handlerPort);

    closing = false;

    // create new thread, pass parameters with ResourceFinder
    thread = new GesturesRenderingEngineThread(moduleName,
                                               rf);

    // start the thread to do the work
    if (!thread->start())
    {
        delete thread;
        return false;
    }

    return true;
}

bool GesturesRenderingEngineModule::interruptModule()
{
    handlerPort.interrupt();

    return true;
}

bool GesturesRenderingEngineModule::close()
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

bool GesturesRenderingEngineModule::updateModule()
{
    return !closing;
}

double GesturesRenderingEngineModule::getPeriod()
{
    return 0.0;
}

// IDL functions
bool GesturesRenderingEngineModule::attach(yarp::os::RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}

bool GesturesRenderingEngineModule::do_nod()
{
    return thread->do_nod();
}

bool GesturesRenderingEngineModule::do_punch()
{
    return thread->do_punch();
}

bool GesturesRenderingEngineModule::do_lookout()
{
    return thread->do_lookout();
}

bool GesturesRenderingEngineModule::do_thumbsup()
{
    return thread->do_thumbsup();
}

bool GesturesRenderingEngineModule::do_thumbsdown()
{
    return thread->do_thumbsdown();
}

bool GesturesRenderingEngineModule::quit()
{
    yInfo("quitting");
    closing = true;

    return true;
}
