// #include "PoseEstimation.h"
#include "../include/PoseEstimation.h"

mvo::PoseEstimation::PoseEstimation()
{
    rvec = cv::Mat();
    tvec = cv::Mat();
}
bool mvo::PoseEstimation::solvePnP(const std::vector<cv::Point3f>& objectPoints,
                    const std::vector<cv::Point2f>& imagePoints,
                    const cv::Mat cameraIntrinsic,
                    cv::OutputArray rvec,
                    cv::OutputArray tvec)
{
    if(!cv::solvePnPRansac(objectPoints, imagePoints, cameraIntrinsic,cv::Mat(), rvec, tvec))
    {
        std::cerr <<"Can't solve PnP" << std::endl;
    }
    return true;
}