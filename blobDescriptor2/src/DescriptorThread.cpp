/*
 * Copyright: (C) 2012-2015 POETICON++, European Commission FP7 project ICT-288382
 * Copyright: (C) 2015 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#include <yarp/os/Log.h>

#include "DescriptorThread.h"
#include "Obj2D.h"

BlobDescriptorThread::BlobDescriptorThread(const string &_moduleName,
    const double _period, const int &_maxObjects, const string &_mode,
    const int &_minArea, const int &_maxArea)
    : moduleName(_moduleName), RateThread(int(_period*1000.0)),
      maxObjects(_maxObjects),
      mode(_mode),
      minArea(_minArea), maxArea(_maxArea)
{
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

        inLabImgPortName = "/" + moduleName + "/labeledImg:i";
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
        /*
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
        */
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
        /*
        inRoiPort.interrupt();

        outRawImgPort.interrupt();
        outAffPort.interrupt();
        outToolAffPort.interrupt();

        // for debug
        outViewImgPort.interrupt();
        outBothPartsImgPort.interrupt();
        */
    }
}

bool BlobDescriptorThread::threadInit()
{
    //yInfo("thread initialization");

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
        if ( (inRawImgPort.getInputCount() && !inRawImgPort.getEnvelope(tsRaw)) ||
             (inBinImgPort.getInputCount() && !inBinImgPort.getEnvelope(tsBin)) ||
             (inLabImgPort.getInputCount() && !inLabImgPort.getEnvelope(tsBin)) )
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
            //yDebug() << "unique labels" << uniq;
            int numObjects = uniq.size();
            // container of binary masks that are 1 where inLab==labelValue, 0 elsewhere
            std::vector<Mat> binMask(numObjects);
            // container of contours: i'th object associated to vector<vector<Point> >
            std::vector< std::vector<std::vector<Point> > > cont(numObjects);
            // container of objects/blobs with shape descriptors
            std::vector<Obj2D> objs;
            // count number of valid objects
            int valid_objs = 0;
            yDebug("");
            // iterate over label values
            for (std::vector<int>::iterator it=uniq.begin(); it!=uniq.end(); ++it)
            {
                // *it is the current label value: firstLabel, secondLabel, ...
                // intIdx is an auxiliary current index: 0, 1, ...
                int intIdx = std::distance(uniq.begin(),it);

                inLab.convertTo(inLab, CV_8UC1);
                binaryMaskFromLabel(inLab, *it, binMask[intIdx]);

                // extract contour of current object - requires 8UC1 or 32SC1
                findContours(binMask[intIdx], cont[intIdx], CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
                int largest;
                if (cont[intIdx].size() == 1)
                    largest = 0;
                else
                {
                    yDebug("findContours returned more than one contour! selecting the largest one");
                    largestContour(cont[intIdx], largest);
                    yDebug() << "largest contour index" << largest << "out of" << cont[intIdx].size();
                }

                double largestArea = contourArea(cont[intIdx][largest]);
                bool isValid = (largestArea>minArea && largestArea<maxArea);
                if (isValid)
                    ++valid_objs;

                // construct Obj2D with validity,contour,area
                objs.push_back( Obj2D(isValid, cont[intIdx][largest], largestArea) );
                // compute remaining shape descriptors
                objs[intIdx].computeDescriptors();
                // compute colour histogram - tbc
                objs[intIdx].computeHueHistogram();
            }

            // output shape descriptors of whole blobs
            Bottle &bDesc = outAffPort.prepare();
            bDesc.clear();
            bDesc.addInt(valid_objs);
            for (std::vector<Obj2D>::iterator it=objs.begin(); it!=objs.end(); ++it)
            {
                if (it->isValid())
                {
                    Bottle &bObj = bDesc.addList();
                    bObj.clear();
                    RotatedRect er = it->getEnclosingRect();
                    /*0*/bObj.addDouble(er.center.x);
                    /*1*/bObj.addDouble(er.center.y);
                    /*2*/bObj.addDouble(er.size.width);
                    /*3*/bObj.addDouble(er.size.height);
                    /*4*/bObj.addDouble(er.angle);
                    Rect br = it->getBoundingRect();
                    // estimate of point over the table
                    /*5*/bObj.addDouble(br.x + br.width/2.);
                    /*6*/bObj.addDouble(br.y + br.height);
                    // tbc colour histograms

                    // shape descriptors
                    bObj.addDouble(it->getArea());
                    bObj.addDouble(it->getConvexity());
                    bObj.addDouble(it->getEccentricity());
                    bObj.addDouble(it->getCompactness());
                    bObj.addDouble(it->getCircleness());
                    bObj.addDouble(it->getSquareness());
                }
            }
            outAffPort.setEnvelope(tsRaw);
            outAffPort.write();
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
        // tbc
    }
}
