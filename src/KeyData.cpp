// #include "KeyData.h"
#include "../include/KeyData.h"


mvo::HomoVec::HomoVec()
{
    mhomogeneousVector.clear();
}
mvo::HomoVec::HomoVec(const cv::Vec4f& v)
{
    mhomogeneousVector.emplace_back(v);
}


mvo::LocalPoints::LocalPoints()
{
    mlocalPoints.clear();
}
mvo::LocalPoints::LocalPoints(const cv::Vec3f& pts)
{
    mlocalPoints.emplace_back(pts);
}


mvo::KeyFrame::KeyFrame()
{
    mkeyframe.clear();
}
mvo::KeyFrame::KeyFrame(const cv::Mat& rt)
{
    mkeyframe.emplace_back(rt);
}


mvo::ImagePoint::ImagePoint()
{
    mimagePoints.clear();
}
mvo::ImagePoint::ImagePoint(const cv::Vec2f& pts)
{
    mimagePoints.emplace_back(pts);
}

mvo::FeaturePoint::FeaturePoint()
{
    mfeaturePoints.clear();
}
mvo::FeaturePoint::FeaturePoint(const cv::KeyPoint& v)
{
    mfeaturePoints.emplace_back(v);
}