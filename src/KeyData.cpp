// #include "KeyData.h"
#include "../include/KeyData.h"

// 3D Map Point vec
mvo::LocalPoints::LocalPoints()
{
    mlocalPoints.clear();
}
mvo::LocalPoints::LocalPoints(const cv::Point3f& pts)
{
    mlocalPoints.push_back(pts);
}


// Camera Pose vec

mvo::KeyFrame::KeyFrame()
{
    mkeyframe.clear();
}
mvo::KeyFrame::KeyFrame(const cv::Mat& rt)
{
    mkeyframe.push_back(rt);
}

mvo::KeyPoint::KeyPoint()
{
    mkeyPoints.clear();
}
mvo::KeyPoint::KeyPoint(const cv::Point2f& pts)
{
    mkeyPoints.push_back(pts);
}