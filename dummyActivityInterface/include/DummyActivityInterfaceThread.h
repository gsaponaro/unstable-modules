/*
 * Copyright: (C) 2012-2015 POETICON++, European Commission FP7 project ICT-288382
 * Copyright: (C) 2016 VisLab, Institute for Systems and Robotics,
 *                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#ifndef DUMMY_ACTIVITY_INTERFACE_THREAD_H
#define DUMMY_ACTIVITY_INTERFACE_THREAD_H

#include <iostream> // __func__
#include <string>

#include <yarp/os/Bottle.h>
#include <yarp/os/Log.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>

// make sure __func__ is set correctly, http://stackoverflow.com/a/17528983
#if __STDC_VERSION__ < 199901L
# if __GNUC__ >= 2
#  define __func__ __FUNCTION__
# else
#  define __func__ "<unknown>"
# endif
#endif

class DummyActivityInterfaceThread : public yarp::os::RateThread
{
    private:
        std::string moduleName;
        bool closing;

    public:
        DummyActivityInterfaceThread(const std::string &_moduleName,
                                     const double _period);
        bool openPorts();
        void close();
        void interrupt();
        bool threadInit();
        void run();

        void mainProcessing();

        // IDL functions
        bool askForTool(const std::string &handName, const int32_t xpos, const int32_t ypos);
        bool drop(const std::string &objName);
        yarp::os::Bottle get2D(const std::string &objName);
        std::string getLabel(const int32_t xpos, const int32_t ypos);
        yarp::os::Bottle getNames();
        bool goHome();
        bool handStat(const std::string &handName);
        std::string inHand(const std::string &objName);
        bool pull(const std::string &objName, const std::string &toolName);
        yarp::os::Bottle pullableWith(const std::string &objName);
        bool push(const std::string &objName, const std::string &toolName);
        bool put(const std::string &objName, const std::string &targetName);
        yarp::os::Bottle reachableWith(const std::string &objName);
        bool take(const std::string &objName, const std::string &handName);
        yarp::os::Bottle underOf(const std::string &objName);
};

#endif
