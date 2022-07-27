#pragma once

#include "FeatureDetection.h"
#include "opencv2/stitching.hpp"

std::vector<cv::detail::CameraParams> cameras;
std::vector<cv::detail::MatchesInfo> pairwise_matches;
std::vector<cv::detail::ImageFeatures> features;

// initialize the above params here

// cv::Ptr<cv::detail::BundleAdjusterBase> adjuster;
// adjuster = cv::makePtr<cv::detail::BundleAdjusterReproj>();
// if (!(*adjuster)(features, pairwise_matches, cameras)) {
//     cout << "Camera parameters adjusting failed." << endl;
//     return -1;
// }