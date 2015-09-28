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

#include "GestoosModule.h"

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
