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
        bool CreateEssentialMatrix(mvo::FeatureDescriptor desc1, mvo::FeatureDescriptor desc2, const cv::InputArray& K);
        bool CreateHomographyMatrix(mvo::FeatureDescriptor desc1, mvo::FeatureDescriptor desc2);
        bool GetEssentialRt(const cv::InputArray& E, const cv::InputArray& pts1, cv::InputArray pts2, const cv::InputArray& K);
        bool GetHomographyRt(const cv::InputArray& H, const cv::InputArray& K);
    public:
        cv::Mat mEssential;
        cv::Mat mHomography;
        cv::Mat mRotation;
        cv::Mat mTranslation;
        cv::Mat mNormals;
    };
    
    cv::Point3f DotProduct3D(const cv::Mat& m, cv::Point3f v);
}//namespace mvo
