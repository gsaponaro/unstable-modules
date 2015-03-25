/*
 * Copyright: (C) 2012-2015 POETICON++, European Commission FP7 project ICT-288382
 * Copyright: (C) 2014 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#include <yarp/os/Log.h>
#include "BlobDescriptorThread.h"
#include "Obj2D.h"

BlobDescriptorThread::BlobDescriptorThread(const string &_moduleName,
    const double _period, const int &_maxObjects, const string &_mode,
    const int &_minArea, const int &_maxArea)
    : moduleName(_moduleName), RateThread(int(_period*1000.0)),
      maxObjects(_maxObjects),
      mode(_mode),
      minArea(_minArea), maxArea(_maxArea)
{
    /*
    fprintf(stdout, "%s: constructed thread with variables ", moduleName.c_str());
    fprintf(stdout, "maxObjects=%d ", maxObjects);
    fprintf(stdout, "mode=%s ", mode.c_str());
    fprintf(stdout, "minArea=%d maxArea=%d", minArea, maxArea);
    fprintf(stdout, "\n");
    */
    yInfo("constructed thread with variables "
          "maxObjects=%d mode=%s minArea=%d maxArea=%d",
          maxObjects, mode.c_str(), minArea, maxArea
          );
}

bool BlobDescriptorThread::openPorts()
{
    if (mode=="2d")
    {
        inRawImgPortName = "/" + moduleName + "/rawImg:i";
        inRawImgPort.open(inRawImgPortName.c_str());

        inBinImgPortName = "/" + moduleName + "/binImg:i";
        inBinImgPort.open(inBinImgPortName.c_str());

        inLabImgPortName = "/" + moduleName + "/labImg:i";
        inLabImgPort.open(inLabImgPortName.c_str());

        // tbc
        //inRoiPortName = "/" + moduleName + "/blobs:i";
        //inRoiPort.open(inRoiPortName.c_str());

        outRawImgPortName = "/" + moduleName + "/rawImg:o";
        outRawImgPort.open(outRawImgPortName.c_str());

        outAffPortName = "/" + moduleName + "/affDescriptor:o";
        outAffPort.open(outAffPortName.c_str());

        // tbc
        //outToolAffPortName = "/" + moduleName + "/toolAffDescriptor:o";
        //outToolAffPort.open(outToolAffPortName.c_str());

        // for debug
        //outViewImgPortName = "/" + moduleName + "/viewImg:o";
        //outViewImgPort.open(outViewImgPortName.c_str());

        //outBothPartsImgPortName = "/" + moduleName + "/bothPartsImg:o";
        //outBothPartsImgPort.open(outBothPartsImgPortName.c_str());
    }

    return true;
}

void BlobDescriptorThread::close()
{
    yInfo("closing ports");

    // critical section
    mutex.wait();

    if (mode=="2d")
    {
        inRawImgPort.close();
        inBinImgPort.close();
        inLabImgPort.close();
        inRoiPort.close();

        outRawImgPort.writeStrict();
        outRawImgPort.close();
        outAffPort.writeStrict();
        outAffPort.close();
        outToolAffPort.writeStrict();
        outToolAffPort.close();

        // for debug
        outViewImgPort.close();
        outBothPartsImgPort.close();
    }

    mutex.post();
}

void BlobDescriptorThread::interrupt()
{
    yInfo("interrupting ports");

    closing = true;

    if (mode=="2d")
    {
        inRawImgPort.interrupt();
        inBinImgPort.interrupt();
        inLabImgPort.interrupt();
        inRoiPort.interrupt();

        outRawImgPort.interrupt();
        outAffPort.interrupt();
        outToolAffPort.interrupt();

        // for debug
        outViewImgPort.interrupt();
        outBothPartsImgPort.interrupt();
    }
}

bool BlobDescriptorThread::threadInit()
{
    yInfo("thread initialization");

    if( !openPorts() )
    {
        yError("problem opening ports");
    };

    closing = false;

    return true;
}

void BlobDescriptorThread::run()
{
    while( !closing && mode=="2d" )
    {
        // acquire new input images
        ImageOf<PixelBgr> *inRawImg = inRawImgPort.read(true);
        ImageOf<PixelBgr> *inBinImg = inBinImgPort.read(true);
        ImageOf<PixelInt> *inLabImg = inLabImgPort.read(true);

        // read timestamps
        Stamp tsRaw, tsBin, tsLab;
        if ( !inRawImgPort.getEnvelope(tsRaw) || !inBinImgPort.getEnvelope(tsBin) ||
             !inLabImgPort.getEnvelope(tsLab) )
        {
            yWarning("timestamp(s) missing");
        }

        if (inRawImg!=NULL && inBinImg!=NULL && inLabImg!=NULL)
        {
            // check dimensions of input images to be equal
            const int refWidth  = inRawImg->width(); 
            const int refHeight = inRawImg->height();
            if ( refWidth!=inBinImg->width() || refHeight!=inBinImg->height() ||
                 refWidth!=inLabImg->width() || refHeight!=inLabImg->height() )
            {
                yWarning("image dimensions mismatch");
            }

            // initialize object instances using labelled image
            // (alternative: use bounding boxes ROIs from blobSpotter)
            Mat inLab = iplToMat(*inLabImg);
            std::vector<int> uniq = unique(inLab,true); // vector of labels incl. 0 (background)
            uniq.erase( std::remove( uniq.begin(),uniq.end(),0 ), uniq.end() ); // get rid of 0
            int numObjects = uniq.size();
            // container of "binary" masks: labelValue!=0 at object points, 0 elsewhere
            std::vector<Mat> mask(numObjects);
            // container of contours: i'th object associated to vector<vector<Point> >
            std::vector< std::vector<std::vector<Point> > > cont(numObjects);
            // container of objects/blobs with shape descriptors
            std::vector<Obj2D> objs;
            for (std::vector<int>::iterator it=uniq.begin(); it!=uniq.end(); ++it)
            {
                // *it is the current label value: firstLabel, secondLabel, ...
                // intIdx is an auxiliary current index: 0, 1, ...
                int intIdx = std::distance(uniq.begin(),it);
                bitwise_and(*it, inLab, mask[intIdx]);
                // mask needs to be 8uC1 or 32sC1 for findContours
                mask[intIdx].convertTo(mask[intIdx], CV_8UC1);
                // extract contour of current object
                findContours(mask[intIdx], cont[intIdx], CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
                // ideally in the previous step we found 1 contour, but
                // in general they can be many; in bad cases, select the
                // largest-area contour
                int largest = largestContour(cont[intIdx]);
                double largestArea = contourArea(cont[intIdx][largest]);
                bool isValid = (largestArea>minArea && largestArea<maxArea);
                // construct Obj2D with validity,contour,area
                objs.push_back( Obj2D(isValid, cont[intIdx][largest], largestArea) );
                yInfo("%d: initialized object %d with area %.1f\n", intIdx, *it, largestArea);
                // compute remaining shape descriptors
                objs[intIdx].computeDescriptors();
            }

            // compute hue histogram
            Mat inRaw = iplToMat(*inRawImg);
            Mat inHSV;
            cvtColor(inRaw, inHSV, COLOR_BGR2HSV);
            int h_bins = 32;
            int histSize[] = { h_bins };
            float h_ranges[] = { 0, 180 }; // in 8-bit images, hue varies from 0 to 179
            const float *ranges[] = { h_ranges };
            int channels[] = { 0 };
            MatND histH;
            calcHist(&inHSV, 1, channels, Mat(), histH, 1, histSize, ranges);
            double maxVal = 0;
            minMaxLoc(histH, 0, &maxVal, 0, 0);
            // tbc
        }
        
        if (inRawImg!=NULL)
        {
            // propagate raw image
            ImageOf<PixelBgr> &outRawImg = outRawImgPort.prepare();
            outRawImg = *inRawImg;
            outRawImgPort.setEnvelope(tsRaw);
            outRawImgPort.write();
        }
    }

    while( !closing && mode=="3d" )
    {

    }
}

/**
  * Find the unique elements of a single-channel Mat.
  * adapted by stackoverflow.com/questions/24716932
  */
std::vector<int> unique(const Mat &input, bool sort)
{
    if (input.channels()>1 || input.type()!=CV_32S)
    {
        yError("unique() only works with CV_32S 1-channel Mat");
        return std::vector<int>();
    }

    std::vector<int> out;
    for (int y=0; y<input.rows; ++y)
    {
        const int *row_ptr = input.ptr<int>(y);
        for (int x=0; x<input.cols; ++x)
        {
            float value = row_ptr[x];

            if ( std::find(out.begin(), out.end(), value) == out.end() )
                out.push_back(value);
        }
    }

    if (sort)
        std::sort(out.begin(), out.end());

    return out;
}

/**
  * Convert YARP IplImage (any type) to OpenCV Mat.
  */
template <class T>
Mat iplToMat(ImageOf<T> &ipl)
{
    return cvarrToMat(ipl.getIplImage());
}

/**
  * Given a vector of OpenCV contours, return the index of the one with the
  * largest area.
  */
int largestContour(std::vector< std::vector<Point> > cnt)
{
    double largestArea = 0.0;
    int largestIdx = 0;
    
    for (int c=0; c<cnt.size(); ++c)
    {
        double currArea = contourArea(cnt[c], false);
        if (currArea > largestArea)
        {
            largestArea = currArea;
            largestIdx = c;
        }
    }
}
