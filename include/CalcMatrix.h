#pragma once

#include "KeyData.h"
#include "FeatureDetection.h"
#include "opencv2/calib3d.hpp"

namespace mvo
{
    class CalcMatrix
    {
    public:
        CalcMatrix();
    public:
        bool CreateEssentialMatrix(const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2, const cv::InputArray& K);
        bool GetEssentialRt(const cv::InputArray& E, const cv::InputArray& K, const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2);
        // bool CreateEssentialMatrix(mvo::FeatureDetect desc1, mvo::FeatureDetect desc2, const cv::InputArray& K);
        // bool CreateHomographyMatrix(mvo::FeatureDetect desc1, mvo::FeatureDetect desc2);
        // bool GetEssentialRt(const cv::InputArray& E, const cv::InputArray& K);
        // bool GetHomographyRt(const cv::InputArray& H, const cv::InputArray& K);
        bool CombineRt();
    public:
        cv::Mat mEssential;
        // cv::Mat mHomography;
        cv::Mat mRotation;
        cv::Mat mTranslation;
        std::vector<cv::Point2f> mVector1;
        std::vector<cv::Point2f> mVector2;
        cv::Mat mVecMat1;
        cv::Mat mVecMat2;
        cv::Mat mCombineRt;
    };
    cv::Mat GetPosePosition(const cv::Mat& rt, const cv::Mat& pos);
}//namespace mvo

