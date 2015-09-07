/* 
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * This program uses Gestoos SDK by Fezoo Labs:
 *     http://www.gestoos.com/
 *     http://www.fezoo.cat/
 *
 */

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
#include <boost/format.hpp>
#include <boost/thread/thread.hpp>

//cv::Mat heat_prob;

cv::Mat depth_map;

namespace gesture_tester
{
    // Tool name
    static const std::string TOOL_NAME   = "Gestoos Detector Demo";
    // Tool brief
    static const std::string BRIEF       = "Run the core detector behind Gestoos on live depth streams\n";
    // Tool description
    static const std::string DESCRIPTION = (std::string) "This demo detects Gestoos in real-time on a RGBD camera at QVGA resolution. \n " +
    "\n" +
    "Current available gestures and class labels:\n"+
    " 1: Tee\n"+
    " 2: Pause\n"+
    " 3: Voldown\n"+
    " 4: Volup\n"+
    " 5: Cross\n"+
    " 6: Ok\n"+
    " 7: Fwd (note: asymmetric gesture)\n"+
    " 8: Rewind(note: asymmetric gesture)\n"+
    "\nFor more details, ask contact@fezoo.cat \n" +
    "\n\n To run the demo, simply invoke the tool from command line and tell the path to the desired gestures320.ini file" +
    "\n gestureDemo --ini_file <path to>/gestures320.ini\n\n" +
    "\n\n You can also activate/deactivate gestures in gestures320.ini by removing lines." ;
}

using namespace gesture_tester;
using namespace gestoos;

class TestConfig : public gestoos::ToolConfig
{
public:
    std::string ini_file;
    int score;

    TestConfig(int argc, char* argv[]) :
        gestoos::ToolConfig(TOOL_NAME, BRIEF, DESCRIPTION)
    {

        ini_file ="./config/gestures320.ini";

        #ifdef MSVC
            ini_file ="./config/gesturesWin.ini";
        #endif

        score=0;

        //add options
        add_option(ini_file, "ini_file", "Configuration file with gestures and thresholds");
        add_option(score, "score", "Display score map for a given gesture ordinal (the n-th gesture in your gesture.ini)");

        // configure
        configure(argc, argv);

        //read
        read("ini_file", ini_file);
        read("score", score);
    }
};

int main(int argc, char* argv[])
{
    /*
    //Test on a live device
    gestoos::detection::GestureDetector detector;
    int frame_count = 0;

    TestConfig cfg(argc, argv);

    detector.init_camera(gestoos::CaptureRGBD::QVGA_30FPS); // 320x240
    if (!detector.is_connected_to_camera())
    {
        //std::cout << "ERROR:  RGBD sensor not found" << std::endl;
        exit(-1);
    }
    detector.init_detector(cfg.ini_file, ".");
    detector.use_motion_detection(true);
    */

    gestoos::detection::GestureDetector detector;
    TestConfig cfg(argc, argv);

    //Configure camera
    gestoos::CaptureRGBD capture;
    capture.init("", // oni_file
                 0,  // usingkinect
                 gestoos::CaptureRGBD::QVGA_30FPS);

    // initialize gesture detector
    detector.set_video_mode(gestoos::CaptureRGBD::QVGA_30FPS);
    detector.init_detector("./config/handGestures.ini", "."); 
    detector.use_motion_detection(false);

    //Declare WHAITracker
    gestoos::tracking::WHAITracker whai;
    //To store the current frame num
    gestoos::tracking::ObjectTrack::ts_type frame = 0;
    //Configure the tracker
    whai.init("./config/whai.ini");


    /*
    double minDepth;
    double maxDepth;
    double draw_scale = 2.0;
    #ifndef AVOID_QT
        if (cfg.score) {
            cv::namedWindow("Detection map");
            cv::moveWindow("Detection map", 320*draw_scale, 240*draw_scale);
        }
    #endif
    */

    while(1) // processing loop
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

        // not necessary?
        //depth_map.convertTo(depth_map, CV_16UC1);

        /*
         * Still gesture detector
         */

        //detector.process(); // input from RGBD sensor
        detector.process(depth_map); // input from filtered hand tracker map

        ++frame;



        /*
        // overlay tracking results on the depth map
        cv::Mat color_img;
        // Normalizing depth data values
        depth_map.convertTo(color_img, CV_8UC1, 1/15.0);
        // Converting from Grayscale to RGB
        cv::cvtColor(color_img, color_img, CV_GRAY2BGR);
        //Set to false to hide trajectory
        bool show_trajectory = true;
        //Comment to hide hand position and trajectory
        whai.visualize(color_img, 3, show_trajectory);
        //Comment to hide hand labels
        whai.show_labels(color_img, 3);
        */



        /*
        detector.min_max_depth(minDepth, maxDepth);
        if (minDepth < 600 )
        {
            std::cout << "YOU (OR SOMETHING) ARE TOO CLOSE! " << minDepth << std::endl;
        }
        */

        /*
        //Show score map
        #ifndef AVOID_QT
            detector.visualize(draw_scale);
            if (cfg.score)
            {
                gestoos::score_heat_map(detector.get_probability_map(cfg.score), heat_prob, 0., 10.);
                cv::resize(heat_prob, heat_prob, cv::Size(0,0), draw_scale, draw_scale);
                cv::imshow("Detection map", heat_prob);
            }

            int key = cv::waitKey(1);

            // ESC
            if ( key==27 ) break;
        #else
            int id =  detector.get_gesture().id;
            if (id > 0 ) std::cout << "Detected gesture " << id << std::endl;
        #endif
        */
    }

    return 0;
}
