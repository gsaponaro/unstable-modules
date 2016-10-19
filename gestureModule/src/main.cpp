/*
 * Copyright: (C) 2015 VisLab, Institute for Systems and Robotics,
 *                     Istituto Superior Técnico, Lisbon, Portugal
 * Author: Giovanni Saponaro
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 */

#include "gesture.h"

int main(int argc, char *argv[])
{
    Network yarp;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("gesture");
    rf.setDefaultConfigFile("gesture.ini");
    rf.setDefault("name", "gesture");
    rf.configure(argc, argv);

    if (! yarp::os::Network::checkNetwork())
    {
        yError("yarpserver not available!");
        return 1; // EXIT_FAILURE
    }

    GestureModule mod;
    return mod.runModule(rf);
}