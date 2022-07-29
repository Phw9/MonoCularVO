#include <iostream>
// #include "CalcMatrix.h"
// #include "FeatureDetection.h"
#include "../include/CalcMatrix.h"
#include "../include/FeatureDetection.h"

#include "opencv2/core.hpp"

mvo::CalcMatrix::CalcMatrix()
{
    mEssential = cv::Mat();
    mHomography = cv::Mat();
    mRotation = cv::Mat();
    mTranslation = cv::Mat();
    mNormals = cv::Mat();
}
// 각 image에서 5개 이상의 2D image Points를 각각 pts1, pts2에 같은 사이즈로 넣고 Intrinsic matrix를 K에 넣으면 된다. RANSAC 이용
bool mvo::CalcMatrix::CreateEssentialMatrix(mvo::FeatureDescriptor desc1, mvo::FeatureDescriptor desc2, const cv::InputArray& K)
{
    int n = (desc1.mfastKeyPoints.size() > desc2.mfastKeyPoints.size()) ? desc2.mfastKeyPoints.size() : desc1.mfastKeyPoints.size();
    desc1.mfastKeyPoints.resize(n);
    desc2.mfastKeyPoints.resize(n);
    std::vector<cv::Point2f> vec1;        
    std::vector<cv::Point2f> vec2;
    for (cv::KeyPoint kp : desc1.mfastKeyPoints)
    {
        cv::Point pt(cvRound(kp.pt.x), cvRound(kp.pt.y));
        vec1.push_back(pt);
    }
    for (cv::KeyPoint kp : desc2.mfastKeyPoints)
    {
        cv::Point pt(cvRound(kp.pt.x), cvRound(kp.pt.y));
        vec2.push_back(pt);
    }

    mEssential = cv::findEssentialMat(vec1, vec2, K, cv::RANSAC, 0.999, 1.0, cv::noArray());
    if (mEssential.empty())
    {
        std::cerr << "Can't find essential matrix" << std::endl;
        return false;
    }
    return true;
}

// 각 image에서 4개 이상의 2D image Points를 각각 src1, src2에 같은 사이즈로 넣는다. RANSAC 이용
bool mvo::CalcMatrix::CreateHomographyMatrix(mvo::FeatureDescriptor desc1, mvo::FeatureDescriptor desc2)
{
    int n = (desc1.mfastKeyPoints.size() > desc2.mfastKeyPoints.size()) ? desc2.mfastKeyPoints.size() : desc1.mfastKeyPoints.size();
    desc1.mfastKeyPoints.resize(n);
    desc2.mfastKeyPoints.resize(n);
    std::vector<cv::Point2f> vec1;
    std::vector<cv::Point2f> vec2;
    for (cv::KeyPoint kp : desc1.mfastKeyPoints)
    {
        cv::Point pt(cvRound(kp.pt.x), cvRound(kp.pt.y));
        vec1.push_back(pt);
    }
    for (cv::KeyPoint kp : desc2.mfastKeyPoints)
    {
        cv::Point pt(cvRound(kp.pt.x), cvRound(kp.pt.y));
        vec2.push_back(pt);
    }

    mHomography = cv::findHomography(vec1, vec2, cv::RANSAC, 3.0, cv::noArray(), 2000, 0.995);
    if (mHomography.empty())
    {
        std::cerr << "Can't find homography matrix" << std::endl;
        return false;
    }
    return true;
}
// Essential 행렬과 Essential 행렬을 만들 때 사용했던 pts1, pts2, Intrinsic Matrix K 를 넣으면 된다.
bool mvo::CalcMatrix::GetEssentialRt(const cv::InputArray& E, const cv::InputArray& pts1, cv::InputArray pts2, const cv::InputArray& K)
{
    cv::recoverPose(E, pts1, pts2, K, mRotation, mTranslation);
    if (mRotation.empty() || mTranslation.empty())
    {
        std::cerr << "Can't get Essential Rt" << std::endl;
        return false;
    }
    return true;
}
// 호모그래피행렬로 Rt(pose) 얻기
bool mvo::CalcMatrix::GetHomographyRt(const cv::InputArray& H, const cv::InputArray& K)
{
    cv::decomposeHomographyMat(H, K, mRotation, mTranslation, mNormals);
    if (mRotation.empty() || mTranslation.empty())
    {
        std::cerr << "Can't get Homgraphy Rt" << std::endl;
        return false;
    }
    return true;
}

cv::Point3f mvo::DotProduct3D(const cv::Mat& m, cv::Point3f v)
{
    // for(int j=0; j < m.rows(); j++)
    // {
    //     for(int i=0; i < m.cols(); i++)
    //     {
    //         sum += m.at<uchar>(j,i) * v[i]
    //     }
    //     p[j] = sum;
    //     sum = 0;
    // }
    cv::Point3f p;
    
	p.x += m.at<uchar>(0, 0) * v.x; p.x += m.at<uchar>(0, 1) * v.y; p.x += m.at<uchar>(0, 2) * v.z;
    p.y += m.at<uchar>(1, 0) * v.x; p.y += m.at<uchar>(1, 1) * v.y; p.y += m.at<uchar>(1, 2) * v.z;
    p.z += m.at<uchar>(2, 0) * v.x; p.z += m.at<uchar>(2, 1) * v.y; p.z += m.at<uchar>(2, 2) * v.z;

    return p;
}