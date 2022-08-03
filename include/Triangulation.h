#include "CalcMatrix.h"

namespace mvo
{
    class Triangulate
    {
        public:
        Triangulate();

        public:
        bool CalcWorldPoints(const cv::Mat& pose1,
                            const cv::Mat& pose2,
                            std::vector<cv::Point2f> desc1,
                            std::vector<cv::Point2f> desc2);

        public:
        std::vector<cv::Vec4f> mworldPoints;
    };

}