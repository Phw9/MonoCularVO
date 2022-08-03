// #include "Triangulation.h"
#include "../include/Triangulation.h"
mvo::Triangulate::Triangulate()
{
    mworldPoints.clear();
}

bool mvo::Triangulate::CalcWorldPoints(const cv::Mat& pose1,
                                        const cv::Mat& pose2,
                                        std::vector<cv::Point2f> pts1,
                                        std::vector<cv::Point2f> pts2)
{
    int n = (pts1.size() > pts2.size()) ? pts1.size() : pts2.size();
    pts1.resize(n);
    pts2.resize(n);
    
    cv::triangulatePoints(pose1, pose2, pts1, pts2, mworldPoints);
    
    if(mworldPoints.empty())
    {
        std::cerr << "failed triagulatePoints" << std::endl;
        return false;
    } 

    return true;
}