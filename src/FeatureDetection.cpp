#include<iostream>
// #include "FeatureDetection.h"
#include "../include/FeatureDetection.h"

mvo::FeatureDescriptor::FeatureDescriptor()
{
    mfastKeyPoints.clear();
    morbKeyPoints.clear();
}
bool mvo::FeatureDescriptor::ConerFAST(const cv::Mat& src)
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
bool mvo::FeatureDescriptor::ConerORB(const cv::Mat& src)
{
    //ORB 클래스 객체를 생성해 feature smart ptr에 저장 후
    cv::Ptr<cv::Feature2D> feature = cv::ORB::create();
    feature->detectAndCompute(src, cv::Mat(), morbKeyPoints, mdesc);

    if (morbKeyPoints.empty())
    {
        std::cerr << "Failed ORB detection" << std::endl;
        return false;
    }
    return true;
}

// not yet
bool GoodMatchesORB(mvo::FeatureDescriptor f1, mvo::FeatureDescriptor f2, cv::OutputArray pts1, cv::OutputArray pts2)
{
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NormTypes::NORM_HAMMING);

    std::vector<cv::DMatch> matches;
    matcher->match(f1.mdesc, f2.mdesc, matches);

    std::sort(matches.begin(), matches.end());
    std::vector<cv::DMatch> good_matches(matches.begin(), matches.begin() + 50);
    return true;
}