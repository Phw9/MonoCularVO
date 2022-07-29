#include "CalcMatrix.h"

namespace mvo
{
    class Triangulate
    {
        public:
        Triangulate();

        public:
        bool CalcWorldPoints(const cv::InputArray& pose1,
                            const cv::InputArray& pose2,
                            mvo::FeatureDescriptor desc1,
                            mvo::FeatureDescriptor desc2);

        public:
        std::vector<std::vector<float>> mworldPoints;
    };

}