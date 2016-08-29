/*
 * Copyright: (C) 2012-2015 POETICON++, European Commission FP7 project ICT-288382
 * Copyright: (C) 2016 VisLab, Institute for Systems and Robotics,
 *                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#ifndef DUMMY_ACTIVITY_INTERFACE_MODULE_H
#define DUMMY_ACTIVITY_INTERFACE_MODULE_H

#include <string>

#include <yarp/os/Bottle.h>
#include <yarp/os/RFModule.h>

#include "DummyActivityInterface_IDL.h"
#include "DummyActivityInterfaceThread.h"

using namespace std;
using namespace yarp::os;

class DummyActivityInterfaceModule : public RFModule,
                                     public DummyActivityInterface_IDL
{
    private:
        // module parameters
        string moduleName;
        string handlerPortName;
        RpcServer handlerPort;
        bool closing;

        // pointer to a new thread
        DummyActivityInterfaceThread *thread;

        // thread stuff
        double threadPeriod;

    public:
        virtual bool configure(ResourceFinder &rf);
        virtual bool interruptModule();
        virtual bool close();
        virtual bool updateModule();
        virtual double getPeriod();

        // IDL functions
        bool attach(yarp::os::RpcServer &source);
        bool askForTool(const string &handName, const int32_t xpos, const int32_t ypos);
        bool drop(const string &objName);
        Bottle get2D(const string &objName);
        string getLabel(const int32_t xpos, const int32_t ypos);
        Bottle getNames();
        bool goHome();
        bool handStat(const string &handName);
        string inHand(const string &objName);
        bool pull(const string &objName, const string &toolName);
        Bottle pullableWith(const string &objName);
        bool push(const string &objName, const string &toolName);
        bool put(const string &objName, const string &targetName);
        Bottle reachableWith(const string &objName);
        bool take(const string &objName, const string &handName);
        Bottle underOf(const string &objName);
        bool quit();
};

#endif
