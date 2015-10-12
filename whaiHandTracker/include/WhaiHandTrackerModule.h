/* 
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * This program uses Gestoos SDK by Fezoo Labs:
 *     http://www.gestoos.com/
 *     http://www.fezoo.cat/
 *
 */

#ifndef __WHAI_MODULE_H__
#define __WHAI_MODULE_H__

#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/legacy/compat.hpp> // cvCopyImage

#include <fezoolib/Core/BoundingBox.hpp>
#include <fezoolib/Tracking/WHAITracker.hpp>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Image.h>

#include "WhaiHandTrackerSupport.h"

using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;

class HandTrackerModule : public RFModule
{
private:
    std::string moduleName;
    std::string outHandsInfoPortName;
    std::string outHandTemplatePortName;
    BufferedPort<Bottle> outHandsInfoPort;
    BufferedPort<ImageOf<PixelMono16> > outHandTemplatePort;

    gestoos::CaptureRGBD capture;                  // camera
    gestoos::tracking::WHAITracker whai;           // hand tracker
    gestoos::tracking::ObjectTrack::ts_type frame; // current frame number
    int videoMode;
    gestoos::CaptureRGBD::VideoModes gestoosVideoMode;
    bool render;
    cv::Mat depth_map;

public:
    bool configure(ResourceFinder &rf);
    bool interruptModule();
    bool close();

    double getPeriod();
    bool updateModule();
};

#endif // __WHAI_MODULE_H__
