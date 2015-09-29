#ifndef __GESTOOS_MODULE_H__
#define __GESTOOS_MODULE_H__

#include <string>

#include <boost/format.hpp>
#include <boost/thread/thread.hpp>

#include <fezoolib/Core/BoundingBox.hpp>
#include <fezoolib/Core/CaptureRGBD.hpp>
#include <fezoolib/Core/DepthFiltering.hpp>
#include <fezoolib/Core/DepthGeometry.hpp>
#include <fezoolib/Core/Serialization.hpp>
#include <fezoolib/Core/Timestamp.hpp>
#include <fezoolib/Core/ToolConfig.hpp>
#include <fezoolib/Core/Visualization.hpp>
#include <fezoolib/Detection/GestureDetector.hpp>
#include <fezoolib/Tracking/WHAITracker.hpp>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Log.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>

using namespace yarp::os;

class GestoosModule : public RFModule
{
private:
    gestoos::CaptureRGBD capture;
    gestoos::detection::GestureDetector detector;
    gestoos::tracking::WHAITracker whai;
    gestoos::tracking::ObjectTrack::ts_type frame; // current frame number

    std::vector<int> labels;
    std::vector<double> thresholds;

    // variables to visualize score maps
    std::vector<cv::Mat> sm;
    std::vector<cv::Mat> colored;
    bool first_run;

    cv::Mat depth_map;

    std::string moduleName;
    std::string outTraitsPortName;
    BufferedPort<Bottle> outTraitsPort;

public:
    bool configure(ResourceFinder &rf);
    bool interruptModule();
    bool close();

    double getPeriod();
    bool updateModule();
};

#endif // __GESTOOS_MODULE_H__
