/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 */

#include "ToolDemoModule.h"

using namespace std;
using namespace yarp::os;

/***************************************************/
bool ToolDemoModule::configure(ResourceFinder &rf)
{
    string moduleName = rf.check("name",Value("tooldemo")).asString();

    closing = false;

    rpcPort.open("/"+moduleName+"/rpc:i");
    attach(rpcPort);

    fullBlobDescriptorInputPort.open(("/"+moduleName+"/fullObjDesc:i").c_str());
    partsBlobDescriptorInputPort.open(("/"+moduleName+"/partsObjDesc:i").c_str());

    affPredictionsOutPort.open("/"+moduleName+"/affPred:o");
    affPredictionsInPort.open("/"+moduleName+"/affPred:i");

    return true;
}

/***************************************************/
bool ToolDemoModule::interruptModule()
{
    fullBlobDescriptorInputPort.interrupt();
    partsBlobDescriptorInputPort.interrupt();
    affPredictionsOutPort.interrupt();
    affPredictionsInPort.interrupt();

    return true;
}

/***************************************************/
bool ToolDemoModule::close()
{
    fullBlobDescriptorInputPort.close();
    partsBlobDescriptorInputPort.close();
    affPredictionsOutPort.close();
    affPredictionsInPort.close();

    return true;
}

/***************************************************/
double ToolDemoModule::getPeriod()
{
    return 0.01;
}

/***************************************************/
bool ToolDemoModule::updateModule()
{
    return !closing;
    //return true;
}

// IDL functions

/***************************************************/
bool ToolDemoModule::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}

/***************************************************/
bool ToolDemoModule::select()
{
    if (fullBlobDescriptorInputPort.getInputCount()<1 || partsBlobDescriptorInputPort.getInputCount()<1)
    {
        yWarning("not connected to blob descriptor");
        return false;
    }

    int numBlobs = 0;
    const double elongThresh = 50.0; // objects are roughly 30, tools 80

    Bottle *fullBlobs = fullBlobDescriptorInputPort.read();
    Bottle *partsBlobs = partsBlobDescriptorInputPort.read();

    // separate objects from tools
    vector<int> objs;
    objs.clear();
    vector<int> tools;
    tools.clear();

    if (fullBlobs!=NULL && partsBlobs!=NULL)
    {
        numBlobs = static_cast<int>( fullBlobs->get(0).asDouble() );

        for(int b=0; b<numBlobs; ++b)
        {
            // last element is elongation
            if (fullBlobs->get(1+b).asList()->get( fullBlobs->get(1+b).asList()->size()-1 ).asDouble() > elongThresh)
            {
                //yDebug("blob %d, elongation ABOVE threshold -> tool", b);
                tools.push_back(b);
            }
            else
            {
                //yDebug("blob %d, elongation BELOW threshold -> object", b);
                objs.push_back(b);
            }
        }
    }
    yInfo("Current scene: %lu tools, %lu objects", tools.size(), objs.size());

    if (objs.size()>1)
    {
        yWarning("I see more than 1 target object, please clean the scene and retry");
        return false;
    }
    else if (objs.size()==0)
    {
        yWarning("I see 0 target objects, please add one and retry");
        return false;
    }

    // only one target object
    const int obj_idx = objs[0];
    //yDebug("obj_idx is %d", obj_idx);
    const double obj_conv = fullBlobs->get(1+obj_idx).asList()->get(24).asDouble();
    const double obj_ecce = fullBlobs->get(1+obj_idx).asList()->get(25).asDouble();
    const double obj_comp = fullBlobs->get(1+obj_idx).asList()->get(26).asDouble();
    const double obj_circ = fullBlobs->get(1+obj_idx).asList()->get(27).asDouble();
    const double obj_squa = fullBlobs->get(1+obj_idx).asList()->get(28).asDouble(); // use...
    const double obj_elon = fullBlobs->get(1+obj_idx).asList()->get(29).asDouble(); // ...only one

    // make a query for each tool
    const double action = 1.0; // draw
    for(std::vector<int>::const_iterator iter = tools.begin();
        iter != tools.end();
        ++iter)
    {
        const int tool_idx = tools[*iter];

        const double tool_conv = partsBlobs->get(1+tool_idx).asList()->get(0).asList()->get(3).asDouble();
        const double tool_ecce = partsBlobs->get(1+tool_idx).asList()->get(0).asList()->get(4).asDouble();
        const double tool_comp = partsBlobs->get(1+tool_idx).asList()->get(0).asList()->get(5).asDouble();
        const double tool_circ = partsBlobs->get(1+tool_idx).asList()->get(0).asList()->get(6).asDouble();
        const double tool_squa = partsBlobs->get(1+tool_idx).asList()->get(0).asList()->get(7).asDouble(); // use...
        const double tool_elon = partsBlobs->get(1+tool_idx).asList()->get(0).asList()->get(8).asDouble(); // ...only one

        Bottle query;
        Bottle queryResponse;

        query.clear();

        query.addDouble(tool_conv);
        query.addDouble(tool_ecce);
        query.addDouble(tool_comp);
        query.addDouble(tool_circ);
        query.addDouble(tool_squa); // use...
        //query.addDouble(tool_elon); // ...only one

        query.addDouble(obj_conv);
        query.addDouble(obj_ecce);
        query.addDouble(obj_comp);
        query.addDouble(obj_circ);
        query.addDouble(obj_squa); // use...
        //query.addDouble(obj_elon); // ...only one

        query.addDouble(action);

        yInfo("sending query: %s", query.toString().c_str());
        affPredictionsOutPort.write(query);

        bool got = false;
        while (!got)
        {
            got = affPredictionsInPort.read(queryResponse);
        }

        yarp::os::Time::delay(0.5);
    }

    return true;
}

/***************************************************/
bool ToolDemoModule::quit()
{
    yInfo("quitting");
    closing = true;

    return true;
}
