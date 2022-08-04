// #include "Triangulation.h"
#include "../include/Triangulation.h"


bool mvo::Triangulate::CalcWorldPoints(const cv::Mat& pose1,
                                        const cv::Mat& pose2,
                                        cv::Mat pts1,
                                        cv::Mat pts2)
{
    int n = (pts1.rows < pts2.rows) ? pts1.rows : pts2.rows;
    
    pts1.pop_back(abs(pts1.rows - n));
    pts2.pop_back(abs(pts2.rows - n));
    
    cv::triangulatePoints(pose1, pose2, pts1.t(), pts2.t(), mworldPoints);
    
    if(mworldPoints.empty())
    {
        std::cerr << "failed triagulatePoints" << std::endl;
        return false;
    }

    return true;
}

    bool mvo::Triangulate::ScalingPoints()
    {
        for(int i = 0; i < mworldPoints.cols; i++)
        {
            for(int j = 0; j < mworldPoints.rows; j++)
            {
                mworldPoints.at<float>(j,i) = mworldPoints.at<float>(j,i) / mworldPoints.at<float>(mworldPoints.rows-1,i);
            }
        }
        if(mworldPoints.at<float>(mworldPoints.rows-1,0) != 1.0f) return false;

        mworldPoints.pop_back();
        return true;
    }