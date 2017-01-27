/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#ifndef GESTURES_RENDERING_ENGINE_MODULE_H
#define GESTURES_RENDERING_ENGINE_MODULE_H

#include <string>

#include <yarp/os/Bottle.h>
#include <yarp/os/RFModule.h>

#include "GesturesRenderingEngine_IDL.h"
#include "GesturesRenderingEngineThread.h"

class GesturesRenderingEngineModule : public yarp::os::RFModule,
                                      public GesturesRenderingEngine_IDL
{
    private:
        // module parameters
        std::string moduleName;
        std::string handlerPortName;
        yarp::os::RpcServer handlerPort;
        bool closing;

        // pointer to a new thread
        GesturesRenderingEngineThread *thread;

    public:
        virtual bool configure(yarp::os::ResourceFinder &rf);
        virtual bool interruptModule();
        virtual bool close();
        virtual bool updateModule();
        virtual double getPeriod();

        // IDL functions
        bool attach(yarp::os::RpcServer &source);
        bool do_nod();
        bool do_punch();
        bool do_lookout();
        bool do_thumbsup();
        bool do_thumbsdown();
        bool quit();
};

#endif
