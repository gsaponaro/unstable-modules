/*
 * Copyright: (C) 2015 VisLab, Institute for Systems and Robotics,
 *                     Istituto Superior Técnico, Lisbon, Portugal
 * Author: Giovanni Saponaro
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 */

#include "gesture.h"

/* RateThread class */

GestureThread::GestureThread(unsigned int _period)
: RateThread(_period)
{
}

GestureThread::~GestureThread()
{
}

bool GestureThread::threadInit()
{
    return true;
}

void GestureThread::run()
{

}

void GestureThread::threadRelease()
{

}

/* RFModule class */

bool GestureModule::configure(ResourceFinder &rf)
{
    moduleName = rf.check( "name",Value("gesture") ).asString();
    setName(moduleName.c_str());
    
    inSkelPort.open(("/"+moduleName+"/skel:i").c_str());
    outScorePort.open(("/"+moduleName+"/score:o").c_str());
    
    return true;
}

bool GestureModule::interruptModule()
{
    inSkelPort.interrupt();
    return true;
}


bool GestureModule::close()
{
    inSkelPort.close();
    return true;
}

bool GestureModule::updateModule()
{
    const int expSkelSize = 91;
    Bottle *skel = inSkelPort.read(true);
    if (skel != NULL)
    {
        if (skel->size() != expSkelSize)
        {
            yWarning("skeleton bottle size mismatch");
            return true;
        }

    }

    return true;
}

double GestureModule::getPeriod()
{
    return 0.0;
}

