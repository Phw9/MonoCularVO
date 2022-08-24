#include<iostream>
// #include "FeatureDetection.h"
#include "../include/FeatureDetection.h"

mvo::FeatureDetect::FeatureDetect() : mdesc{cv::Mat()}
{
    mfastKeyPoints.clear();
    // morbKeyPoints.clear();
    mfeatures.clear();
}

mvo::FeatureTracking::FeatureTracking()
{
    mstatus.clear();
    merr.clear();
    mfeatures.clear();
}

bool mvo::FeatureDetect::ConerFAST(const cv::Mat& src)
{
    // threshold는 120이고 비최대 억제를 수행한다
    cv::FAST(src, mfastKeyPoints, 120, true);
    if (mfastKeyPoints.empty())
    {
        std::cerr << "Failed FAST detection" << std::endl;
        return false;
    }
    return true;
}

bool mvo::FeatureDetect::GoodFeatureToTrack(const cv::Mat& src)
{
    cv::goodFeaturesToTrack(src, mfeatures, 1000, 0.01, 10);
    if(mfeatures.empty())
    {
        std::cerr << "failed to goodFeaturesToTrack" << std::endl;
        return false;
    }
    return true;
}

bool mvo::FeatureTracking::OpticalFlowPyrLK(const cv::Mat& src1, const cv::Mat& src2, std::vector<cv::Point2f>& pts1)
{
    cv::Size winSize = cv::Size(21,21);
    cv::TermCriteria termcrit = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
    
    cv::calcOpticalFlowPyrLK(src1, src2, pts1, mfeatures, mstatus, merr, winSize, 3, termcrit, 0, 0.0001);

    int indexCorrection = 0;
    for(int i = 0; i < mstatus.size(); i++)
    {
        cv::Point2f pt = mfeatures.at(i - indexCorrection);
        if((mstatus.at(i) == 0) || (pt.x < 0) || (pt.y < 0))
        {
            if(((pt.x < 0) || (pt.y < 0)))
            {
                mstatus.at(i) = 0;
            }
            pts1.erase(pts1.begin() + (i - indexCorrection));           // time complexity
            mfeatures.erase(mfeatures.begin() + (i - indexCorrection));
            indexCorrection++;
        }
    }
    if(mfeatures.empty())
    {
        std::cerr << "failed calcOpticalFlowPyrLK" << std::endl;
        return false;
    }

    return true;
}


// bool mvo::FeatureDetect::ConerORB(const cv::Mat& src)
// {
//     //ORB 클래스 객체를 생성해 feature smart ptr에 저장 후
//     cv::Ptr<cv::Feature2D> feature = cv::ORB::create();
//     feature->detectAndCompute(src, cv::Mat(), morbKeyPoints, mdesc);

//     if (morbKeyPoints.empty())
//     {
//         std::cerr << "Failed ORB detection" << std::endl;
//         return false;
//     }
//     return true;
// }

// // not yet
// bool BFMatcherORB(mvo::FeatureDetect f1, mvo::FeatureDetect f2, cv::OutputArray pts1, cv::OutputArray pts2)
// {
//     cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NormTypes::NORM_HAMMING);

//     std::vector<cv::DMatch> matches;
//     matcher->match(f1.mdesc, f2.mdesc, matches);

//     std::sort(matches.begin(), matches.end());
//     std::vector<cv::DMatch> good_matches(matches.begin(), matches.begin() + 50);
//     return true;
// }