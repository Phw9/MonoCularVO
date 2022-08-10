#pragma once

#include "opencv2/features2d.hpp"
#include "opencv2/stitching.hpp"
#include "opencv2/video.hpp"
namespace mvo
{
    class FeatureDetect
    {
    public:
        FeatureDetect();
        
        bool ConerFAST(const cv::Mat& src);
        // bool ConerORB(const cv::Mat& src);
        // bool BFMatcherORB(mvo::FeatureDetect f1, mvo::FeatureDetect f2, cv::OutputArray pts1, cv::OutputArray pts2);

        bool GoodFeatureToTrack(const cv::Mat& src);


        std::vector<cv::KeyPoint> mfastKeyPoints;
        // std::vector<cv::KeyPoint> morbKeyPoints;
        cv::Mat mdesc;
        std::vector<cv::Point2f> mfeatures;
    };

    class FeatureTracking
    {
    public:
        FeatureTracking();

        bool OpticalFlowPyrLK(const cv::Mat& src1, const cv::Mat& src2, std::vector<cv::Point2f>& pts1);

        std::vector<uchar> mstatus;
        std::vector<float> merr;
        std::vector<cv::Point2f> mfeatures;
    };
}
