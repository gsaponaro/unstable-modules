#include "GestoosModule.h"
#include "GestoosSupport.h"

// rendering function called by the render thread
void render_func(const cv::Mat &depth_map, gestoos::tracking::WHAITracker *tracker)
{
    if (depth_map.cols == 0)
        return;

    cv::Mat color_img;

    // normalize
    depth_map.convertTo(color_img, CV_8UC1, 1/15.0);

    // convert from grayscale to RGB
    cv::cvtColor(color_img, color_img, CV_GRAY2BGR);

    // set to false to hide trajectory
    bool show_trajectory = true;

    // comment to hide hand position and trajectory
    tracker->visualize(color_img,
                       1, // scale
                       show_trajectory);

    // comment to hide hand labels
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
    outHandsPortName = "/" + moduleName + "/hands:o";
    outHandsPort.open(outHandsPortName);
    outGesturesPortName = "/" + moduleName + "/gestures:o";
    outGesturesPort.open(outGesturesPortName);

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
    capture.init("", // oni_file: use "" for live images
                 0,  // usingkinect
                 gestoos::CaptureRGBD::QVGA_30FPS);

    // hand tracker
    frame = 0;
    whai.init("./config/whai.ini");

    // gesture detector
    detector.set_video_mode(gestoos::CaptureRGBD::QVGA_30FPS);
    detector.init_detector("./config/handGestures.ini", // ini_file
                           ".");                        // bundle_path
    detector.activate_multithreading(useMultithreading);
    detector.use_motion_detection(useMotionDetection);
    detector.set_sampling_downscale(samplingStride);

    labels = detector.get_labels(); // integers
    std::cout << "gesture detector labels: ";
    dumpVector(labels);

    // save alphabetic names of gestures, currently not exposed by Gestoos API
    std::vector<std::string> names =
        {"close","victory","tee","silence"};
    if (labels.size() != names.size())
        yError("size mismatch between gesture IDs and gesture names, cannot initialize map");
    else
        initializeMap(labels,names,nameContainer);

    thresholds = detector.get_thresholds(); // index 0: first positive gesture
    std::cout << "gesture detector thresholds: ";
    dumpVector(thresholds);

    first_run = true;

    return true;
}

bool GestoosModule::interruptModule()
{
    outHandsPort.interrupt();
    outGesturesPort.interrupt();
    return true;
}

bool GestoosModule::close()
{
    outHandsPort.close();
    outGesturesPort.close();
    return true;
}

double GestoosModule::getPeriod()
{
    return 0.0; // sync on incoming data
}

bool GestoosModule::updateModule()
{
    // capture new frame
    capture.get_depth_frame();
    depth_map = capture.depth_frame();

    // filter data, mainly filling depth holes
    depth_map = gestoos::depth_error_filter(depth_map);

    // detect hands using WHAI hand tracker
    whai.update(depth_map, frame);

    // show WHAI hand tracker
    render_func(depth_map, &whai);

    // get hand positions
    std::vector<gestoos::tracking::ObjectTrack*> objects = whai.active_tracks();

    // prepare mask with rectangles over hand regions
    cv::Mat mask;
    mask.create(cv::Size(depth_map.cols,  // normally 320
                         depth_map.rows), // normally 240
                CV_8UC1);
    mask = cv::Scalar(0);
    cv::Rect roi;
    roi.width = 31;
    roi.height = 31;
    for (int i=0; i < objects.size(); ++i)
    {
        //double score = objects[i]->get_score();
        //std::cout << "hand score=" << score;

        cv::Point2f p2d = objects[i]->get_position_2d();

        //std::cout << " pos2d=" << p.x << "," << p.y;
        //cv::Point3f p3d = objects[i]->get_position();
        //std::cout << " pos3d=" << p3d.x << "," << p3d.y << "," << p3d.z << std::endl;

        roi.x = p2d.x-15;
        roi.y = p2d.y-15;
        //Set local neighbourhood to 255 to apply gesture detection on these regions
        mask(gestoos::crop_to_image(roi, mask)) = cv::Scalar(255);
    }

    // write to yarp hands:o port
    yarp::os::Bottle &bHands = outHandsPort.prepare();
    bHands.clear();
    for (int i=0; i < objects.size(); ++i)
    {
        int id = objects[i]->get_id();
        //cv::Point2f p2d = objects[i]->get_position_2d();
        cv::Point3f p3d = objects[i]->get_position();
        //int depth = objects[i]->get_depth();
        double score = objects[i]->get_score();

        // skip if zero score
        if (score<=0.0)
            continue;

        yarp::os::Bottle &h = bHands.addList();

        yarp::os::Bottle &bid = h.addList();
        bid.addString("id");
        bid.addInt(id);

        yarp::os::Bottle &bcoords = h.addList();
        bcoords.addString("coords");
        yarp::os::Bottle &bcoordscontent = bcoords.addList();
        bcoordscontent.addDouble(p3d.x);
        bcoordscontent.addDouble(p3d.y);
        bcoordscontent.addDouble(p3d.z);

        yarp::os::Bottle &bscore = h.addList();
        bscore.addString("score");
        bscore.addDouble(score);
    }
    if (bHands.size()>0)
        outHandsPort.write();

    // still gesture detector

    detector.process(depth_map);
    //detector.process(depth_map, mask); // restrict to hand region

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

    // write to yarp gestures:o port
    yarp::os::Bottle &bGest = outGesturesPort.prepare();
    bGest.clear();
    for (std::vector<cv::Mat>::const_iterator i = sm.begin();
         i != sm.end();
         ++i)
    {
        int idx = i - sm.begin(); // 0, 1, 2, ...
        double score = 0.0;
        cv::Point point;
        cv::minMaxLoc(sm[idx], // prob. map
                      NULL, // minimum value
                      &score, // maximum value
                      NULL, // location of minimum
                      &point // location of maximum
                      // mask
                      );
        double z = 0.0;
        z = static_cast<double>(depth_map.at<unsigned short>(point));

        // skip if zero score
        if (score<=0.0)
            continue;

        yarp::os::Bottle &g = bGest.addList();

        yarp::os::Bottle &bid = g.addList();
        bid.addString("id");
        bid.addInt(labels[idx]);

        yarp::os::Bottle &bname = g.addList();
        bname.addString("name");
        bname.addString(nameContainer[labels[idx]]);

        yarp::os::Bottle &bscore = g.addList();
        bscore.addString("score");
        bscore.addDouble(score);

        yarp::os::Bottle &bcoords = g.addList();
        bcoords.addString("coords");
        yarp::os::Bottle &bcoordscontent = bcoords.addList();
        bcoordscontent.addDouble(static_cast<double>(point.x));
        bcoordscontent.addDouble(static_cast<double>(point.y));
        bcoordscontent.addDouble(z);
    }
    if (bGest.size()>0)
        outGesturesPort.write();

    // actual visualization
    for (std::vector<int>::const_iterator i = labels.begin();
         i != labels.end();
         ++i)
    {
        std::stringstream win_name;
        win_name << nameContainer[*i];
        int idx = i - labels.begin(); // 0, 1, 2, ...
        cv::imshow(win_name.str(), colored[idx]);
        if (first_run)
        {
            int new_x, new_y, win_height_incl_spacing;
            new_x = depth_map.cols * 1.2;
            win_height_incl_spacing = depth_map.rows * 1.1;
            new_y = win_height_incl_spacing * idx;
            cv::moveWindow(win_name.str(), new_x, new_y);
        }
    }
    if (first_run)
        first_run = false;

    // show (filtered) depth map
    detector.visualize();

    cv::waitKey(33);

    ++frame;

    return true;
}
