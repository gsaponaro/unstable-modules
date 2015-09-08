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

cv::Mat depth_map;

namespace gesture_tester
{
    // Tool name
    static const std::string TOOL_NAME   = "WHAI Hand Tracker + Gestoos Detector";
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
        ini_file = "./config/gestures320.ini";

        score = 0;

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
    //Configure camera
    gestoos::CaptureRGBD capture;
    capture.init("", // oni_file
                 0,  // usingkinect
                 gestoos::CaptureRGBD::QVGA_30FPS);

    // initialize gesture detector
    gestoos::detection::GestureDetector detector;
    TestConfig cfg(argc, argv);
    detector.set_video_mode(gestoos::CaptureRGBD::QVGA_30FPS);
    detector.init_detector("./config/handGestures.ini", // ini_file
                           ".");                        // bundle_path
    detector.use_motion_detection(false);

    std::vector<int> labels;
    labels = detector.get_labels();
    //dumpDetectorLabels(labels); // print IDs of gestures, for debug

    // variables to visualize score maps
    std::vector<cv::Mat> sm;
    std::vector<cv::Mat> colored;
    bool first_run = true;

    // initialize WHAI hand tracker
    gestoos::tracking::WHAITracker whai;
    //To store the current frame num
    gestoos::tracking::ObjectTrack::ts_type frame = 0;
    //Configure the tracker
    whai.init("./config/whai.ini");

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

        /*
         * Still gesture detector
         */

        //detector.process(); // input from RGBD sensor
        detector.process(depth_map); // input from filtered hand tracker map

        ++frame;

        // compute score map of detector for each gesture
        double min_val=1, max_val=10;

        for (std::vector<int>::const_iterator i = labels.begin();
             i != labels.end();
             ++i)
        {
            int idx = i - labels.begin(); // 0, 1, 2, ...
            if (first_run)
            {
                // create and compute sm[idx]
                sm.push_back(detector.get_probability_map(1+idx)); // skip 0 (negative class)

                // create and compute colored[idx]
                colored.push_back(cv::Mat());
                gestoos::score_heat_map(sm[idx], colored[idx], min_val, max_val);
            }
            else
            {
                // just update
                sm[idx] = detector.get_probability_map(1+idx);
                gestoos::score_heat_map(sm[idx], colored[idx], min_val, max_val);
            }

        }

        if (first_run)
            first_run = false;

        // visualize
        for (std::vector<int>::const_iterator i = labels.begin();
             i != labels.end();
             ++i)
        {
            std::stringstream win_name;
            win_name << *i << " score";
            int idx = i - labels.begin(); // 0, 1, 2, ...
            cv::imshow(win_name.str(), colored[idx]);
        }
        cv::waitKey(33);

        /*
        cv::Mat sm0 = detector.get_probability_map(1);
        cv::Mat colored0;
        gestoos::score_heat_map(sm0, colored0, 1, 10);

        cv::Mat sm1 = detector.get_probability_map(2);
        cv::Mat colored1;
        gestoos::score_heat_map(sm1, colored1, 1, 10);

        cv::imshow("Responses gesture0", colored0);
        cv::imshow("Responses gesture1", colored1);
        cv::waitKey(33);
        */
    }

    return 0;
}

void dumpDetectorLabels(std::vector<int> &labels)
{
    std::cout << "gesture detector labels: ";
    for (std::vector<int>::const_iterator i = labels.begin();
         i != labels.end();
         ++i)
    {
        std::cout << *i << ' ';
    }
    std::cout << std::endl;
}
