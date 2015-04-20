/*
 * Copyright: (C) 2012-2015 POETICON++, European Commission FP7 project ICT-288382
 * Copyright: (C) 2015 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#include "Obj2D.h"

/**
  * Constructor.
  */
Obj2D::Obj2D()
: valid(true),
  contour(std::vector<cv::Point>()),
  area(1.0), convexity(0.0), eccentricity(0.0), compactness(0.0),
  circleness(0.0), squareness(0.0),
  perimeter(0.0), elongation(0.0),
  moments(cv::Moments()),
  boundingRect(cv::Rect()), enclosingRect(cv::RotatedRect()),
  circleCenter(cv::Point2f()), circleRadius(0.0),
  poly(std::vector<cv::Point>()), hull(std::vector<cv::Point>()),
  defects(std::vector<cv::Vec4i>())
{
}

/**
  * Constructor specifying contour.
  */
Obj2D::Obj2D(std::vector<cv::Point> _contour)
: valid(true),
  contour(_contour),
  area(1.0), convexity(0.0), eccentricity(0.0), compactness(0.0),
  circleness(0.0), squareness(0.0),
  perimeter(0.0), elongation(0.0),
  moments(cv::Moments()),
  boundingRect(cv::Rect()), enclosingRect(cv::RotatedRect()),
  circleCenter(cv::Point2f()), circleRadius(0.0),
  poly(std::vector<cv::Point>()), hull(std::vector<cv::Point>()),
  defects(std::vector<cv::Vec4i>())
{
}

/**
  * Constructor specifying contour and area.
  */
Obj2D::Obj2D(std::vector<cv::Point> _contour, double _area)
: valid(true),
  contour(_contour),
  area(_area), convexity(0.0), eccentricity(0.0), compactness(0.0),
  circleness(0.0), squareness(0.0),
  perimeter(0.0), elongation(0.0),
  moments(cv::Moments()),
  boundingRect(cv::Rect()), enclosingRect(cv::RotatedRect()),
  circleCenter(cv::Point2f()), circleRadius(0.0),
  poly(std::vector<cv::Point>()), hull(std::vector<cv::Point>()),
  defects(std::vector<cv::Vec4i>())
{
}

/**
  * Constructor specifying validity, contour and area.
  */
Obj2D::Obj2D(bool _isValid, std::vector<cv::Point> _contour, double _area)
: valid(_isValid),
  contour(_contour),
  area(_area), convexity(0.0), eccentricity(0.0), compactness(0.0),
  circleness(0.0), squareness(0.0),
  perimeter(0.0), elongation(0.0),
  moments(cv::Moments()),
  boundingRect(cv::Rect()), enclosingRect(cv::RotatedRect()),
  circleCenter(cv::Point2f()), circleRadius(0.0),
  poly(std::vector<cv::Point>()), hull(std::vector<cv::Point>()),
  defects(std::vector<cv::Vec4i>())
{
}

/**
  * Given an existing object with validity,contour,area already set,
  * compute the remaining descriptors.
  */
void Obj2D::computeDescriptors()
{
    convexHull(contour, hull);
    double hull_perimeter = arcLength(hull, true);
    approxPolyDP(contour, poly, arcLength(contour,true)*0.02, true);
    perimeter = arcLength(poly, true);
    convexity = (perimeter>0 ? hull_perimeter/perimeter : 0);

    enclosingRect = minAreaRect(contour);
    double major_axis, minor_axis; // TODO: should come from ellipse
    major_axis = (enclosingRect.size.width>enclosingRect.size.height ?
                  enclosingRect.size.width :
                  enclosingRect.size.height);
    minor_axis = (enclosingRect.size.width>enclosingRect.size.height ?
                  enclosingRect.size.height :
                  enclosingRect.size.width);
    eccentricity = (major_axis>0 ? minor_axis/major_axis : 0);

    compactness = (perimeter>0 ? area/(perimeter*perimeter) : 0);

    minEnclosingCircle(contour, circleCenter, circleRadius);
    const double PI = 3.141592653589793;
    circleness = (circleRadius>0 ? area/(PI*circleRadius*circleRadius) : 0);

    double enclosingRectArea = major_axis * minor_axis;
    squareness = (enclosingRectArea>0 ? area/enclosingRectArea : 0);

    yDebug("area=%f "
           "convexity=%.2f eccentricity=%.2f compactness=%.2f "
           "circleness=%.2f squareness=%.2f",
           area, convexity, eccentricity, compactness, circleness, squareness);
}
