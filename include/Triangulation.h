#include "CalcMatrix.h"

namespace mvo
{
    class Triangulate
    {
        public:
        bool CalcWorldPoints(const cv::Mat& pose1,
                            const cv::Mat& pose2,
                            const std::vector<cv::Point2f>& pts1,
                            const std::vector<cv::Point2f>& pts2);
        // bool CalcWorldPoints(const cv::Mat& pose1,
        //                     const cv::Mat& pose2,
        //                     cv::Mat pts1,
        //                     cv::Mat pts2);
        bool ScalingPoints();

        public:
        cv::Mat mworldMapPoints = cv::Mat();
    };
}