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
    mVector1.clear();
    mVector2.clear();
}
// 각 image에서 5개 이상의 2D image Points를 각각 pts1, pts2에 같은 사이즈로 넣고 Intrinsic matrix를 K에 넣으면 된다. RANSAC 이용
bool mvo::CalcMatrix::CreateEssentialMatrix(mvo::FeatureDescriptor desc1, mvo::FeatureDescriptor desc2, const cv::InputArray& K)
{
    int n = (desc1.mfastKeyPoints.size() > desc2.mfastKeyPoints.size()) ? desc2.mfastKeyPoints.size() : desc1.mfastKeyPoints.size();
    desc1.mfastKeyPoints.resize(n);
    desc2.mfastKeyPoints.resize(n);
    for (cv::KeyPoint kp : desc1.mfastKeyPoints)
    {
        cv::Point2f pt(cvRound(kp.pt.x), cvRound(kp.pt.y));
        mVector1.push_back(pt);
    }
    for (cv::KeyPoint kp : desc2.mfastKeyPoints)
    {
        cv::Point2f pt(cvRound(kp.pt.x), cvRound(kp.pt.y));
        mVector2.push_back(pt);
    }

    mEssential = cv::findEssentialMat(mVector1, mVector2, K, cv::RANSAC, 0.999, 1.0, cv::noArray());
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

    for (cv::KeyPoint kp : desc1.mfastKeyPoints)
    {
        cv::Point pt(cvRound(kp.pt.x), cvRound(kp.pt.y));
        mVector1.push_back(pt);
    }
    for (cv::KeyPoint kp : desc2.mfastKeyPoints)
    {
        cv::Point pt(cvRound(kp.pt.x), cvRound(kp.pt.y));
        mVector2.push_back(pt);
    }

    mHomography = cv::findHomography(mVector1, mVector2, cv::RANSAC, 3.0, cv::noArray(), 2000, 0.995);
    if (mHomography.empty())
    {
        std::cerr << "Can't find homography matrix" << std::endl;
        return false;
    }
    return true;
}
// Essential 행렬과 Essential 행렬을 만들 때 사용했던 Intrinsic Matrix K 를 넣으면 된다.
bool mvo::CalcMatrix::GetEssentialRt(const cv::InputArray& E, const cv::InputArray& K)
{
    cv::recoverPose(E, mVector1, mVector2, K, mRotation, mTranslation);
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
bool mvo::CalcMatrix::CombineRt()
{
    cv::Mat temp = cv::Mat();
    temp = mRotation.t();
    temp.push_back(mTranslation.t());
    mCombineRt = temp.t();
    if (!(mCombineRt.rows == 3 && mCombineRt.cols == 4))
    {
        std::cerr << "failed Combine Rotation Translation Matrix" << std::endl;
        return false;
    }
    return true;
}

cv::Vec3f mvo::DotProduct3D(const cv::Mat& m, cv::Vec4f v)
{

    int sum = 0;
    cv::Vec3f P;
    for(int j=0; j < m.rows; j++)
    {
        for(int i=0; i < m.cols; i++)
        {
            sum += m.at<uchar>(j,i) * v[i];
        }
        P[j] = sum;
        sum = 0;
    }
    for(int i=0; i<3; i++)
    {
        P[i] = v[i]/v[3];
    }
    
    return P;
}
