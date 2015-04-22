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
    perimeter = arcLength(contour, true);
    convexHull(contour, hull);
    double hull_perimeter = arcLength(hull, true);
    double convexity_temp = (perimeter>0 ? hull_perimeter/perimeter : 0);
    const int conv_exponent = 2;
    double convexity = (perimeter>0 ? pow(convexity_temp,conv_exponent) : 0);
    //yDebug("hull_perimeter=%.2f perimeter=%.2f \t convexity=%.2f convexity^2=%.2f",
    //       hull_perimeter, perimeter, convexity_temp, convexity);

    double majorAxisEll, minorAxisEll;
    RotatedRect enclosingRectEll = fitEllipse(contour);
    majorAxisEll = (enclosingRectEll.size.width>enclosingRectEll.size.height ?
                    enclosingRectEll.size.width :
                    enclosingRectEll.size.height);
    minorAxisEll = (enclosingRectEll.size.width>enclosingRectEll.size.height ?
                    enclosingRectEll.size.height :
                    enclosingRectEll.size.width);
    eccentricity = (majorAxisEll>0 ? minorAxisEll/majorAxisEll : 0);
    //yDebug("minorAxisEll=%.2f majorAxisEll=%.2f \t eccentricity=%.2f",
    //       minorAxisEll, majorAxisEll, eccentricity);

    compactness = (perimeter>0 ? (4*CV_PI*area)/pow(perimeter,2) : 0);
    if (compactness > 1.0)
    {
        yWarning("compactness was >1.0 -> set to 1.0. check computation of this descriptor!");
        compactness = 1.0;
    }
    //yDebug("4*pi*area=%.2f per^2=%.2f \t compactness=%.2f",
    //       4*CV_PI*area, pow(perimeter,2), compactness);

    minEnclosingCircle(contour, circleCenter, circleRadius);
    circleness = (circleRadius>0 ? area/(CV_PI*pow(circleRadius,2)) : 0);
    //yDebug("area=%.2f pi*radius^2=%.2f \t circleness=%.2f",
    //       area, CV_PI*pow(circleRadius,2), circleness);

    enclosingRect = minAreaRect(contour);
    double majorAxisRect, minorAxisRect;
    majorAxisRect = (enclosingRect.size.width>enclosingRect.size.height ?
                     enclosingRect.size.width :
                     enclosingRect.size.height);
    minorAxisRect = (enclosingRect.size.width>enclosingRect.size.height ?
                     enclosingRect.size.height :
                     enclosingRect.size.width);
    double enclosingRectArea = majorAxisRect * minorAxisRect;
    squareness = (enclosingRectArea>0 ? area/enclosingRectArea : 0);
    //yDebug("area=%.2f enclosingRectArea=%.2f \t squareness=%.2f",
    //       area, enclosingRectArea, squareness);

    yDebug("perimeter=%.2f area=%.2f convexity=%.2f eccentricity=%.2f compactness=%.2f circleness=%.2f squareness=%.2f",
           perimeter, area, convexity, eccentricity, compactness, circleness, squareness);
}

// accessors

/**
  * Return whether the object is valid.
  */
bool Obj2D::isValid()
{
    return valid;
}

/**
  * Return enclosing rectangle (rotated rectangle containing best-fit ellipse).
  */
RotatedRect Obj2D::getEnclosingRect()
{
    return enclosingRect;
}

/**
  * Return bounding rectangle (up-right bounding box in image).
  */
Rect Obj2D::getBoundingRect()
{
    return cv::boundingRect(contour);
}

/**
  * Return object area.
  */
double Obj2D::getArea()
{
    return area;
}

/**
  * Return object convexity.
  */
double Obj2D::getConvexity()
{
    return convexity;
}

/**
  * Return object eccentricity.
  */
double Obj2D::getEccentricity()
{
    return eccentricity;
}

/**
  * Return object compactness.
  */
double Obj2D::getCompactness()
{
    return compactness;
}

/**
  * Return object circleness.
  */
double Obj2D::getCircleness()
{
    return circleness;
}

/**
  * Return object squareness.
  */
double Obj2D::getSquareness()
{
    return squareness;
}
