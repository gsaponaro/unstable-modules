/*
 * Copyright: (C) 2012-2015 POETICON++, European Commission FP7 project ICT-288382
 * Copyright: (C) 2014 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#ifndef	BLOB_DESC_THREAD_H
#define BLOB_DESC_THREAD_H

#include <algorithm>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Image.h>

using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;

class BlobDescriptorThread : public RateThread
{
    private:
        string moduleName;

        bool closing;
        Semaphore mutex;

        int maxObjects;
        string mode;

        // 2d mode

        string inRawImgPortName;
        string inBinImgPortName;
        string inLabImgPortName;
        string inRoiPortName;

        string outRawImgPortName;
        string outAffPortName;
        string outToolAffPortName;

        BufferedPort<ImageOf<PixelBgr> > inRawImgPort;
        BufferedPort<ImageOf<PixelBgr> > inBinImgPort;
        BufferedPort<ImageOf<PixelInt> > inLabImgPort;
        BufferedPort<Bottle>             inRoiPort;

        BufferedPort<ImageOf<PixelBgr> > outRawImgPort;
        BufferedPort<Bottle>             outAffPort;
        BufferedPort<Bottle>             outToolAffPort;

        Bottle aff;
        Bottle toolAff;
        
        int minArea, maxArea;

        // for debug
        string                           outViewImgPortName;
        string                           outBothPartsImgPortName;
        BufferedPort<ImageOf<PixelBgr> > outViewImgPort;
        Port                             outBothPartsImgPort;
        Mat                              bothParts;

        // 3d mode

    public:
        BlobDescriptorThread(const string &_moduleName, const double _period,
                             const int &_maxObjects,
                             const string &_mode,
                             const int &_minArea, const int &_maxArea);

        bool openPorts();
        void close();
        void interrupt();

        bool threadInit();
        void run();
};

// helper functions

std::vector<int> unique(const Mat &input, bool sort=false);

template <class T>
Mat iplToMat(ImageOf<T> &ipl);

int largestContour(std::vector< std::vector<Point> > cnt);

#endif
