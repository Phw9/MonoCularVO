// #include "Triangulation.h"
#include "../include/Triangulation.h"
mvo::Triangulate::Triangulate()
{
    mworldPoints.clear();
}

bool mvo::Triangulate::CalcWorldPoints(const cv::InputArray& pose1,
                                        const cv::InputArray& pose2,
                                        mvo::FeatureDescriptor desc1,
                                        mvo::FeatureDescriptor desc2)
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
    
    cv::triangulatePoints(pose1, pose2, vec1, vec2, mworldPoints);
    
    if(mworldPoints.empty()) return false;

    return true;
}