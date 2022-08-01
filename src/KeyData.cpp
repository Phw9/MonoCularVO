// #include "KeyData.h"
#include "../include/KeyData.h"


mvo::HomoVec::HomoVec()
{
    mhomogeneousVector.clear();
}
mvo::HomoVec::HomoVec(const cv::Vec4f& v)
{
    mhomogeneousVector.push_back(v);
}


mvo::LocalPoints::LocalPoints()
{
    mlocalPoints.clear();
}
mvo::LocalPoints::LocalPoints(const cv::Point3f& pts)
{
    mlocalPoints.push_back(pts);
}


mvo::KeyFrame::KeyFrame()
{
    mkeyframe.clear();
}
mvo::KeyFrame::KeyFrame(const cv::Mat& rt)
{
    mkeyframe.push_back(rt);
}


mvo::ImagePoint::ImagePoint()
{
    mimagePoints.clear();
}
mvo::ImagePoint::ImagePoint(const cv::Point2f& pts)
{
    mimagePoints.push_back(pts);
}

mvo::FeaturePoint::FeaturePoint()
{
    mfeaturePoints.clear();
}
mvo::FeaturePoint::FeaturePoint(const cv::KeyPoint& v)
{
    mfeaturePoints.push_back(v);
}