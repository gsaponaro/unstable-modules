/* 
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * This program uses Gestoos SDK by Fezoo Labs:
 *     http://www.gestoos.com/
 *     http://www.fezoo.cat/
 *
 */

#include "WhaiHandTrackerModule.h"

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

bool HandTrackerModule::configure(ResourceFinder &rf)
{
    // parse parameters and open yarp ports
    moduleName = rf.check("name",Value("whai")).asString();
    videoMode = rf.check("videoMode",Value(0)).asInt();
    render = rf.check("render",Value("on")).asString()=="on"?true:false;
    yarp::os::RFModule::setName(moduleName.c_str());
    outHandsInfoPortName = "/" + moduleName + "/handInfo:o";
    outHandsInfoPort.open(outHandsInfoPortName);
    outHandTemplatePortName = "/" + moduleName + "/handTemplate:o";
    outHandTemplatePort.open(outHandTemplatePortName);

    if (videoMode==0)
        gestoosVideoMode = gestoos::CaptureRGBD::QVGA_30FPS;
    else if (videoMode==1)
        gestoosVideoMode = gestoos::CaptureRGBD::VGA_30FPS;
    else if (videoMode==2)
        gestoosVideoMode = gestoos::CaptureRGBD::QVGA_60FPS;
    else
    {
        yWarning("invalid video mode specified, will use QVGA_30FPS");
        videoMode = 0;
        gestoosVideoMode = gestoos::CaptureRGBD::QVGA_30FPS;
    }

    // open camera
    bool initRet = capture.init("", // oni_file: use "" for live images
                                0,  // usingkinect
                                gestoosVideoMode); // video_mode
    if (!initRet)
    {
        yError("gestoos::CaptureRGBD::init could not open device");
        return false;
    }

    // start hand tracker
    frame = 0;
    whai.init("./conf/whaiHandTracker.ini"); // TODO: ResourceFinder path and filename

    return true;
}

bool HandTrackerModule::interruptModule()
{
    outHandsInfoPort.interrupt();
    outHandTemplatePort.interrupt();
    return true;
}

bool HandTrackerModule::close()
{
    outHandsInfoPort.close();
    outHandTemplatePort.close();
    return true;
}

double HandTrackerModule::getPeriod()
{
    return 0.0; // sync on incoming data
}

bool HandTrackerModule::updateModule()
{
    // capture new frame
    capture.get_depth_frame();
    depth_map = capture.depth_frame();

    // filter data, mainly filling depth holes
    depth_map = gestoos::depth_error_filter(depth_map);

    // update WHAI tracker, detect hands
    whai.update(depth_map, frame);

    if (render)
    {
        // show WHAI hand tracker
        render_func(depth_map, &whai);
    }

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

    // write to yarp port
    yarp::os::Bottle &bHands = outHandsInfoPort.prepare();
    bHands.clear();
    std::vector<cv::Mat> handTemplates;
    handTemplates.clear();
    std::vector<double> scores;
    scores.clear();
    for (int i=0; i < objects.size(); ++i)
    {
        int id = objects[i]->get_id();
        cv::Point3f p3d = objects[i]->get_position();
        //cv::Point3f vel = objects[i]->get_velocity();
        //cv::Point3f acc = objects[i]->get_acceleration();
        cv::Rect bb = objects[i]->get_bounding_box();
        double currScore = objects[i]->get_score();

        // skip if zero score
        if (currScore<=0.0)
            continue;

        scores.push_back(currScore);
        cv::Mat currTemplate;
        objects[i]->get_template(currTemplate);
        handTemplates.push_back(currTemplate);

        yarp::os::Bottle &h = bHands.addList();

        yarp::os::Bottle &bid = h.addList();
        bid.addString("id");
        bid.addInt(id);

        yarp::os::Bottle &bscore = h.addList();
        bscore.addString("score");
        bscore.addDouble(currScore);

        yarp::os::Bottle &bcoords = h.addList();
        bcoords.addString("coords");
        yarp::os::Bottle &bcoordscontent = bcoords.addList();
        bcoordscontent.addDouble(p3d.x);
        bcoordscontent.addDouble(p3d.y);
        bcoordscontent.addDouble(p3d.z);

        /*
        yarp::os::Bottle &bvel = h.addList();
        bvel.addString("vel");
        yarp::os::Bottle &bvelcontent = bvel.addList();
        bvelcontent.addDouble(vel.x);
        bvelcontent.addDouble(vel.y);
        bvelcontent.addDouble(vel.z);

        yarp::os::Bottle &bacc = h.addList();
        bacc.addString("acc");
        yarp::os::Bottle &bacccontent = bacc.addList();
        bacccontent.addDouble(acc.x);
        bacccontent.addDouble(acc.y);
        bacccontent.addDouble(acc.z);
        */
    }
    if (bHands.size()>0)
    {
        outHandsInfoPort.write();

        int indexOfBest = posMaxElement(scores);
        cv::Mat handTemplateMat;
        handTemplates[indexOfBest].convertTo(handTemplateMat, CV_16UC1); // original is CV_8UC1, was expecting CV_16UC1

        ImageOf<PixelMono16> &handTemplateYarp = outHandTemplatePort.prepare();

        IplImage handTemplateIpl = handTemplateMat;
        handTemplateYarp.resize(handTemplateIpl.width,handTemplateIpl.height);

        // FIXME
        //cvCopyImage(&handTemplateIpl,
        //            static_cast<IplImage*>(handTemplateYarp.getIplImage()));
        //cvCopy(&handTemplateIpl,
        //       static_cast<IplImage*>(handTemplateYarp.getIplImage()));

        outHandTemplatePort.write();
    }

    //cv::waitKey(33);

    ++frame;

    return true;
}
