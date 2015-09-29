#include "GestoosModule.h"
#include "GestoosSupport.h"
#include "TestConfig.h"

// Rendering function called by the render thread
void render_func(const cv::Mat &depth_map, gestoos::tracking::WHAITracker *tracker)
{
    //Exit if empty image!
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

bool GestoosModule::configure(ResourceFinder &rf)
{
    moduleName = rf.check("name", Value("gestoos")).asString();
    yarp::os::RFModule::setName(moduleName.c_str());
    outTraitsPortName = "/" + moduleName + "/gestureTraits:o";
    outTraitsPort.open(outTraitsPortName);

    // configure camera
    capture.init("", // oni_file
                 0,  // usingkinect
                 gestoos::CaptureRGBD::QVGA_30FPS);

    // configure hand tracker
    frame = 0;
    whai.init("./config/whai.ini");

    // configure gesture detector
    //TestConfig cfg(argc, argv);
    TestConfig cfg;
    detector.set_video_mode(gestoos::CaptureRGBD::QVGA_30FPS);
    detector.init_detector(cfg.ini_file, // ini_file
                           ".");         // bundle_path
    detector.use_motion_detection(true);
    //detector.activate_multithreading(true);

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
    outTraitsPort.interrupt();
    return true;
}

bool GestoosModule::close()
{
    outTraitsPort.close();
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

    /*
    cv::Mat myMat0 = detector.get_probability_map(0);
    double myMin0, myMax0;
    cv::minMaxLoc(myMat0, &myMin0, &myMax0, NULL, NULL);
    std::cout << "class 0 probabilities: min=" << myMin0 << ", max=" << myMax0 << std::endl;

    cv::Mat myMat1 = detector.get_probability_map(1);
    double myMin1, myMax1;
    cv::minMaxLoc(myMat1, &myMin1, &myMax1, NULL, NULL);
    std::cout << "class 1 probabilities: min=" << myMin1 << ", max=" << myMax1 << std::endl;

    cv::Mat myMat2 = detector.get_probability_map(2);
    double myMin2, myMax2;
    cv::minMaxLoc(myMat2, &myMin2, &myMax2, NULL, NULL);
    std::cout << "class 2 probabilities: min=" << myMin2 << ", max=" << myMax2 << std::endl;
    */

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
            sm.push_back(detector.get_probability_map(1+idx)); // 1, 2, ... note: skip 0 (negative class)

            // create and compute colored[idx]
            colored.push_back(cv::Mat());
            gestoos::score_heat_map(sm[idx], colored[idx], min_val, max_val);
        }
        else
        {
            // just update
            sm[idx] = detector.get_probability_map(1+idx); // 1, 2, ... note: skip 0 (negative class)
            gestoos::score_heat_map(sm[idx], colored[idx], min_val, max_val);
        }
    }

    gestoos::detection::GestureDetector::GestureTraits traits;
    traits = detector.get_gesture();
    if (traits.id > 0)
    {
        std::cout << "*** ";
        if (traits.id==6)
            std::cout << "CLOSE ";
        else if (traits.id==15)
            std::cout << "OPEN  ";
        std::cout << " id=" << traits.id
                  << " u=" << traits.u
                  << " v=" << traits.v
                  << " z=" << traits.z << std::endl;

        yarp::os::Bottle &b = outTraitsPort.prepare();
        b.clear();
        b.addInt(traits.id);
        b.addInt(traits.u);
        b.addInt(traits.v);
        b.addDouble( static_cast<double>(traits.z) );
        outTraitsPort.write();
    }

    // actual visualization
    for (std::vector<int>::const_iterator i = labels.begin();
         i != labels.end();
         ++i)
    {
        std::stringstream win_name;
        win_name << *i << " score"; // "6 score", "15 score" etc.
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
