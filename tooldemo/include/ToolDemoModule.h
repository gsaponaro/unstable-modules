/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 */

#ifndef TOOL_DEMO_MODULE_H
#define TOOL_DEMO_MODULE_H

#include <string>
#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Time.h>

#include "tooldemo_IDL.h"

/***************************************************/
class ToolDemoModule : public yarp::os::RFModule,
                             public tooldemo_IDL
{
private:

    bool closing;
    yarp::os::RpcServer rpcPort;

    yarp::os::BufferedPort<yarp::os::Bottle> fullBlobDescriptorInputPort;
    yarp::os::BufferedPort<yarp::os::Bottle> partsBlobDescriptorInputPort;

    yarp::os::Port affPredictionsOutPort;
    yarp::os::Port affPredictionsInPort;

public:

    bool configure(yarp::os::ResourceFinder &rf);
    bool interruptModule();
    bool close();
    double getPeriod();
    bool updateModule();

    // IDL functions
    bool attach(yarp::os::RpcServer &source);
    bool select();
    bool quit();
};

#endif // TOOL_DEMO_MODULE_H
