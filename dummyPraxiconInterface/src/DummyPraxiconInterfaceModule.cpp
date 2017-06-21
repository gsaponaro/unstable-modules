/*
 * Copyright: (C) 2012-2015 POETICON++, European Commission FP7 project ICT-288382
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 * Based on praxiconInterface by Vadim Tikhanoff
 *
 */

#include <yarp/os/Log.h>

#include "DummyPraxiconInterfaceModule.h"

using namespace std;
using namespace yarp::os;

bool DummyPraxiconInterfaceModule::configure(ResourceFinder &rf)
{
    moduleName = rf.check("name", Value("praxInterface")).asString();
    setName(moduleName.c_str());

    const std::string speechPortName = "/"+moduleName+"/speech:i";
    if (!speechPort.open(speechPortName.c_str())) {
        yError("unable to open port %s", speechPortName.c_str());
        return false;
    }

    return true;
}

bool DummyPraxiconInterfaceModule::interruptModule()
{
    speechPort.interrupt();

    return true;
}

bool DummyPraxiconInterfaceModule::close()
{
    speechPort.close();

    return true;
}

bool DummyPraxiconInterfaceModule::updateModule()
{
    yInfo("waiting for a message...");
    speech.clear();
    response.clear();
    speechPort.read(speech, true);

    if (!speech.isNull())
    {
        yInfo("received message: %s", speech.toString().c_str());

        // response to simple3 and complex6 scenarios:
        // ((hand grasp Ham) (Ham reach Bun-bottom) (hand put Ham) (hand grasp Bun-top) (Bun-top reach Ham) (hand put Bun-top))
        Bottle list1;
        list1.clear();
        list1.addString("hand");
        list1.addString("grasp");
        list1.addString("Ham");
        Bottle list2;
        list2.clear();
        list2.addString("Ham");
        list2.addString("reach");
        list2.addString("Bun-bottom");
        Bottle list3;
        list3.clear();
        list3.addString("hand");
        list3.addString("put");
        list3.addString("Ham");
        Bottle list4;
        list4.clear();
        list4.addString("hand");
        list4.addString("grasp");
        list4.addString("Bun-top");
        Bottle list5;
        list5.clear();
        list5.addString("Bun-top");
        list5.addString("reach");
        list5.addString("Ham");
        Bottle list6;
        list6.clear();
        list6.addString("hand");
        list6.addString("put");
        list6.addString("Bun-top");
        Bottle responseContent;
        responseContent.clear();
        responseContent.addList() = list1;
        responseContent.addList() = list2;
        responseContent.addList() = list3;
        responseContent.addList() = list4;
        responseContent.addList() = list5;
        responseContent.addList() = list6;
        response.addList() = responseContent;

        yInfo("sending response: %s", response.toString().c_str());
        speechPort.reply(response);
    }

    return true;
}

double DummyPraxiconInterfaceModule::getPeriod()
{
    return 0.0;
}
