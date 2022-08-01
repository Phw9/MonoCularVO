#pragma once

#include <iostream>
#include <string>
#include "opencv2/core.hpp"

namespace mvo
{
    // 4D Homogeneous vec in triangulate Calc
    class HomoVec
    {
    public:
        HomoVec();
        HomoVec(const cv::Vec4f& v);
    public:
        std::vector<cv::Vec4f> mhomogeneousVector;
    };


    // 3D Map Point vec
    class LocalPoints
    {
    public:
        LocalPoints();
        LocalPoints(const cv::Point3f& pts);

        std::vector<cv::Point3f> mlocalPoints;
    };

    // Camera Pose Mat
    class KeyFrame
    {
    public:
        KeyFrame();
        KeyFrame(const cv::Mat& rt);

        std::vector<cv::Mat> mkeyframe;
    };

    // 2D image Point vec
    class ImagePoint
    {
    public:
        ImagePoint();
        ImagePoint(const cv::Point2f& pts);

        std::vector<cv::Point2f> mimagePoints;
    };
    
    class FeaturePoint
    {
    public:
        FeaturePoint();
        FeaturePoint(const cv::KeyPoint& v);
    public:
        std::vector<cv::KeyPoint> mfeaturePoints;
    };

}//namespace mvo