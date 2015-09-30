#include "GestoosModule.h"
#include "GestoosSupport.h"
#include "TestConfig.h"

// Rendering function called by the render thread
void render_func(const cv::Mat &depth_map, gestoos::tracking::WHAITracker *tracker)
{
    // Exit if empty image
    if( depth_map.cols == 0 )
    {
        return;
    }

    cv::Mat color_img;

    // Normalizing depth data values
    depth_map.convertTo(color_img, CV_8UC1, 1/15.0);
    // Converting from Grayscale to RGB
    cv::cvtColor(color_img, color_img, CV_GRAY2BGR);

    //Set to false to hide trajectory
    bool show_trajectory = true;
    //Comment to hide hand position and trajectory
    tracker->visualize(color_img,
                       1, // scale
                       show_trajectory);
    //Comment to hide hand labels
    tracker->show_labels(color_img,
                         3); // scale

    cv::imshow("Hand Tracker", color_img);

    int key = cv::waitKey(1);
}

void GestoosModule::dumpScoreMapProbabilities(const int class_no)
{
    cv::Mat myMat = detector.get_probability_map(class_no);
    double myMin, myMax;
    cv::minMaxLoc(myMat, &myMin, &myMax, NULL, NULL);
    std::cout << "class " << class_no << " probabilities:"
              //<< " min=" << myMin <<
              << " max=" << myMax << std::endl;
}

bool GestoosModule::configure(ResourceFinder &rf)
{
    // yarp
    moduleName = rf.check("name", Value("gestoos")).asString();
    useMultithreading =
        rf.check("useMultithreading",Value("on")).asString()=="on"?true:false;
    useMotionDetection =
        rf.check("useMotionDetection",Value("on")).asString()=="on"?true:false;
    samplingStride = rf.check("samplingStride", Value(4)).asInt();
    yarp::os::RFModule::setName(moduleName.c_str());
    outScorePortName = "/" + moduleName + "/gesture:o";
    outScorePort.open(outScorePortName);

    if (useMultithreading)
        yInfo("multithreading on");
    else
        yInfo("multithreading off");

    if (useMotionDetection)
        yInfo("motion detection on");
    else
        yInfo("motion detection off");

    yInfo("sampling stride %d", samplingStride);

    // camera
    capture.init("", // oni_file
                 0,  // usingkinect
                 gestoos::CaptureRGBD::QVGA_30FPS);

    // hand tracker
    frame = 0;
    whai.init("./config/whai.ini");

    // gesture detector
    //TestConfig cfg(argc, argv);
    TestConfig cfg;
    detector.set_video_mode(gestoos::CaptureRGBD::QVGA_30FPS);
    detector.init_detector(cfg.ini_file, // ini_file
                           ".");         // bundle_path
    detector.activate_multithreading(useMultithreading);
    detector.use_motion_detection(useMotionDetection);
    detector.set_sampling_downscale(samplingStride);

    labels = detector.get_labels();
    std::cout << "gesture detector labels: ";
    dumpVector(labels);

    thresholds = detector.get_thresholds(); // index 0: first positive gesture
    std::cout << "gesture detector thresholds: ";
    dumpVector(thresholds);

    first_run = true;

    return true;
}

bool GestoosModule::interruptModule()
{
    outScorePort.interrupt();
    return true;
}

bool GestoosModule::close()
{
    outScorePort.close();
    return true;
}

double GestoosModule::getPeriod()
{
    return 0.0; // sync on incoming data
}

bool GestoosModule::updateModule()
{
    /*
     * Capture
     */
    capture.get_depth_frame();
    depth_map = capture.depth_frame();

    /*
     * Filter data, mainly filling depth holes
     */
    depth_map = gestoos::depth_error_filter(depth_map);

    /*
     * Detect hands using WHAI
     */
    whai.update(depth_map, frame);

    render_func(depth_map, &whai);

    //Get hand positions
    std::vector<gestoos::tracking::ObjectTrack*> objects = whai.active_tracks();
    cv::Mat mask;
    mask.create(cv::Size(320, 240), CV_8UC1);
    mask = cv::Scalar(0);
    cv::Rect roi;
    roi.width = 31;
    roi.height = 31;
    for (int i=0; i < objects.size(); ++i)
    {
        cv::Point2f p = objects[i]->get_position_2d();
        roi.x = p.x-15;
        roi.y = p.y-15;
        //Set local neighbourhood to 255 to apply gesture detection on these regions
        mask(gestoos::crop_to_image(roi, mask)) = cv::Scalar(255);
    }

    /*
     * Still gesture detector
     */

    //detector.process(); // input from RGBD sensor
    detector.process(depth_map); // input from filtered hand tracker map

    //detector.set_depth_limits(300.0, 3500.0);
    double minDepth, maxDepth;
    detector.min_max_depth(minDepth, maxDepth); // forced 800, 3500 !?
    //std::cout << "minDepth=" << minDepth << " maxDepth=" << maxDepth << std::endl;

    //dumpScoreMapProbabilities(0); // negative class
    //dumpScoreMapProbabilities(1);
    //dumpScoreMapProbabilities(2);

    // compute score map of detector for each gesture
    double min_val=1., max_val=10.;

    for (std::vector<int>::const_iterator i = labels.begin();
         i != labels.end();
         ++i)
    {
        int idx = i - labels.begin(); // 0, 1, 2, ...
        //std::cout << "i=" << *i << ", idx=" << idx << std::endl;
        if (first_run)
        {
            // create and compute sm[idx]
            sm.push_back(detector.get_probability_map(1+idx)); // 1, 2, ... (skip 0: negative class)

            // create and compute colored[idx]
            colored.push_back(cv::Mat());
            gestoos::score_heat_map(sm[idx], colored[idx], min_val, max_val);
        }
        else
        {
            // just update
            sm[idx] = detector.get_probability_map(1+idx); // 1, 2, ... (skip 0: negative class)
            gestoos::score_heat_map(sm[idx], colored[idx], min_val, max_val);
        }
    }

    gestoos::detection::GestureDetector::GestureTraits traits;
    traits = detector.get_gesture();
    if (traits.id > 0)
    {
        std::cout << "*** ";

        if (traits.id==1)
            std::cout << "CLOSE";
        else if (traits.id==2)
            std::cout << "VICTORY";
        else if (traits.id==3)
            std::cout << "TEE";
        else if (traits.id==4)
            std::cout << "SILENCE";

        std::cout << " id=" << traits.id
                  << " u=" << traits.u
                  << " v=" << traits.v
                  << " z=" << traits.z << std::endl;

        yarp::os::Bottle &b = outScorePort.prepare();
        b.clear();
        b.addInt(traits.id);
        b.addInt(traits.u);
        b.addInt(traits.v);
        b.addDouble( static_cast<double>(traits.z) );
        outScorePort.write();
    }

    // actual visualization
    for (std::vector<int>::const_iterator i = labels.begin();
         i != labels.end();
         ++i)
    {
        std::stringstream win_name;
        win_name << *i << " score";
        int idx = i - labels.begin(); // 0, 1, 2, ...
        cv::imshow(win_name.str(), colored[idx]);
        if (first_run)
        {
            cv::moveWindow(win_name.str(), 1600, 260*(1+idx));
            first_run = false;
        }
    }

    // show (filtered) depth map
    //detector.visualize();

    cv::waitKey(33);

    ++frame;

    return true;
}
