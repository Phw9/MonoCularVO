#pragma once

#include <iostream>
#include <string>
#include "opencv2/core.hpp"

namespace mvo
{
    // 3D Map Point vec
    class LocalPoints
    {
    public:
        LocalPoints();
        LocalPoints(const cv::Point3f& pts);

        std::vector<cv::Point3f> mlocalPoints;
    };

    // Camera Pose vec
    class KeyFrame
    {
    public:
        KeyFrame();
        KeyFrame(const cv::Mat& rt);

        std::vector<cv::Mat> mkeyframe;
    };

    // 2D image Point vec
    class KeyPoint
    {
    public:
        KeyPoint();
        KeyPoint(const cv::Point2f& pts);

        std::vector<cv::Point2f> mkeyPoints;
    };

}//namespace mvo