/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 * Based on actionsRenderingEngine by Carlo Ciliberto, Vadim Tikhanoff
 *
 */

#ifndef GESTURES_RENDERING_ENGINE_THREAD_H
#define GESTURES_RENDERING_ENGINE_THREAD_H

#include <iostream> // __func__
#include <string>

//#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LockGuard.h>
#include <yarp/os/Log.h>
//#include <yarp/os/LogStream.h>
#include <yarp/os/Mutex.h>
//#include <yarp/os/Port.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
//#include <yarp/os/RpcClient.h>
#include <yarp/os/Time.h>
//#include <yarp/os/Vocab.h>
#include <yarp/sig/Vector.h>

// make sure __func__ is set correctly, http://stackoverflow.com/a/17528983
#if __STDC_VERSION__ < 199901L
# if __GNUC__ >= 2
#  define __func__ __FUNCTION__
# else
#  define __func__ "<unknown>"
# endif
#endif

class GesturesRenderingEngineThread : public yarp::os::RateThread
{
    private:
        std::string moduleName;
        yarp::os::ResourceFinder rf;
        bool closing;
        std::string robotName;
        int repetitions;
        //int startup_context_id_gaze;
        yarp::os::Mutex mutex;

        yarp::dev::PolyDriver *drvHead;
        yarp::dev::PolyDriver *drvGazeCtrl;

        yarp::dev::IEncoders *encHead;
        yarp::dev::IPositionControl *headPosCtrl;

        yarp::dev::IGazeControl *gazeCtrl;

        //yarp::dev::IControlMode2 *modeHead;

        yarp::sig::Vector head;

        //IControlMode2     *modeTorso;
        //IPositionControl  *posTorso;

        //IEncoders         *encArm;
        //IControlMode2     *modeArm;
        //IPositionControl  *posArm;
        //ICartesianControl *cartArm;

        void steerHeadToHome();

    public:
        GesturesRenderingEngineThread(const std::string &_moduleName,
                                      yarp::os::ResourceFinder &_rf);
        void close();
        void interrupt();
        bool threadInit();
        void run();

        // IDL functions
        bool do_nod();
        bool do_punch();
        bool do_lookout();
        bool do_thumbsup();
        bool do_thumbsdown();
};

#endif
