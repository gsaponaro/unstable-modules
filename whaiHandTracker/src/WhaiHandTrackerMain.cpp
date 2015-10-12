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

#include "WhaiHandTrackerModule.h"

int main(int argc, char* argv[])
{
    yarp::os::Network yarp;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("whaiHandTracker");
    rf.setDefaultConfigFile("whaiHandTracker.ini");
    rf.configure(argc, argv);

    if(rf.check("help"))
    {
        yInfo("Available options:");
        yInfo("--name prefix (default whai)");
        yInfo("--videoMode number (default 0=QVGA_30FPS, alternatives 1=VGA_30FPS 2=QVGA_60FPS)");
        yInfo("--render <on|off> (default on)");
        return 0; // EXIT_SUCCESS
    }

    if (!yarp::os::Network::checkNetwork())
    {
        yError("yarpserver not available!");
        return 1; // EXIT_FAILURE
    }

    HandTrackerModule mod;
    return mod.runModule(rf);
}
