/* 
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * This program uses Gestoos SDK by Fezoo Labs:
 *     http://www.gestoos.com/
 *     http://www.fezoo.cat/
 *
 */

#include <yarp/os/Log.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

// TODO: fix path
#include "../include/GestoosModule.h"

/*
namespace gesture_tester
{
    // Tool name
    static const std::string TOOL_NAME   = "WHAI Hand Tracker + Gestoos Detector";
    // Tool brief
    static const std::string BRIEF       = "Run the core detector behind Gestoos on live depth streams\n";
    // Tool description
    static const std::string DESCRIPTION = (std::string) "This demo detects Gestoos in real-time on a RGBD camera at QVGA resolution. \n " +
    "\n" +
    "Current available gestures and class labels:\n"+
    "(tbc)\n"+
    "\n\n To run the demo, simply invoke the tool from command line and tell the path to the desired gestures ini file:" +
    "\n ./executable --ini_file <path to>/<ini file name>\n\n" +
    "\n\n You can also activate/deactivate gestures in the ini file by removing lines." ;
}

using namespace gesture_tester;
*/

int main(int argc, char* argv[])
{
    // configure YARP
    yarp::os::Network yarp;
    if (!yarp::os::Network::checkNetwork())
    {
        yError("yarpserver not available!");
        return 1; // EXIT_FAILURE
    }

    ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("gestoos");
    rf.setDefaultConfigFile("gestoos.ini");
    rf.configure(argc, argv);

    GestoosModule mod;
    return mod.runModule(rf);
}
