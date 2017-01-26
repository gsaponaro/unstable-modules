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
#include <yarp/os/Network.h>

#include "GesturesRenderingEngineDefaults.h"
#include "GesturesRenderingEngineModule.h"

using namespace yarp::os;

int main(int argc, char *argv[])
{
    Network yarp;

    ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("gesturesRenderingEngine");    // overridden by --context
    rf.setDefaultConfigFile("gesturesRenderingEngine.ini");  // overridden by --from
    rf.configure(argc, argv);

    if(rf.check("help"))
    {
        yInfo("Options:");
        yInfo("--name <port prefix> (default %s)", DefModuleName.c_str());
        yInfo("--robot <robot name> (icub or icubSim, default %s)", DefRobot.c_str());
        yInfo("--repetitions <int> (default %d)", DefRepetitions);
        return 0; // EXIT_SUCCESS
    }

    if(! yarp::os::Network::checkNetwork() )
    {
        yError("YARP server not available!");
        return 1; // EXIT_FAILURE
    }

    GesturesRenderingEngineModule mod;
    return mod.runModule(rf);
}
