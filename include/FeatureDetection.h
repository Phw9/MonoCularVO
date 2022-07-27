#pragma once

#include "opencv2/features2d.hpp"
#include "opencv2/stitching.hpp"

namespace mvo
{
    class FeatureDescriptor
    {
    public:
        FeatureDescriptor();
        bool ConerFAST(const cv::Mat& src);
        bool ConerORB(const cv::Mat& src);
        bool GoodMatchesORB(mvo::FeatureDescriptor f1, mvo::FeatureDescriptor f2, cv::OutputArray pts1, cv::OutputArray pts2);

        std::vector<cv::KeyPoint> mfastKeyPoints;
        std::vector<cv::KeyPoint> morbKeyPoints;
        cv::Mat mdesc;
    };
}
