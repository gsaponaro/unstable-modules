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
    "(tbc)\n"+
    "\n\n To run the demo, simply invoke the tool from command line and tell the path to the desired gestures ini file:" +
    "\n ./executable --ini_file <path to>/<ini file name>\n\n" +
    "\n\n You can also activate/deactivate gestures in the ini file by removing lines." ;
}

using namespace gesture_tester;

class TestConfig : public gestoos::ToolConfig
{
public:
    std::string ini_file;
    int score;

    //TestConfig(int argc, char* argv[]) :
    TestConfig() :
        gestoos::ToolConfig(TOOL_NAME, BRIEF, DESCRIPTION)
    {
        ini_file = "./config/handGestures.ini";

        score = 0;

        //add options
        add_option(ini_file, "ini_file", "Configuration file with gestures and thresholds");
        add_option(score, "score", "Display score map for a given gesture ordinal (the n-th gesture in your gesture ini)");

        // configure
        //configure(argc, argv);

        //read
        read("ini_file", ini_file);
        read("score", score);
    }
};
