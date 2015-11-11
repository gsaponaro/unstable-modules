/*
 * Copyright: (C) 2015 VisLab, Institute for Systems and Robotics,
 *                     Istituto Superior Técnico, Lisbon, Portugal
 * Author: Giovanni Saponaro
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 */

#ifndef __GESTURE_H__
#define __GESTURE_H__

#include <iostream>
#include <string>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Log.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/RFModule.h>

using namespace std;
using namespace yarp::os;

/* RateThread class */

class GestureThread : public RateThread
{
public:
    GestureThread(unsigned int _period);
    ~GestureThread();
    bool threadInit();
    void run();
    void threadRelease();
};

/* RFModule class */

class GestureModule : public RFModule
{
private:
    string moduleName;
    //GestureThread *thr;

    BufferedPort<Bottle> inSkelPort;
    
    BufferedPort<Bottle> outScorePort;

public:
    bool configure(ResourceFinder &rf);
    bool interruptModule();
    bool close();

    double getPeriod();
    bool updateModule();
};

#endif // __GESTURE_H__

