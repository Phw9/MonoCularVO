#include "Triangulation.h"

namespace mvo
{
    class PoseEstimation
    {
    public:
    PoseEstimation();

    public:
    bool solvePnP(const std::vector<cv::Point3f>& objectPoints,
                    const std::vector<cv::Point2f>& imagePoints,
                    const cv::Mat cameraIntrinsic,
                    cv::OutputArray rvec,
                    cv::OutputArray tvec);

    public:
    cv::Vec3f rvec;
    cv::Vec3f tvec;

    };
}