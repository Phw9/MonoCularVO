#pragma once

#include <deque>
#include <fstream>
#include "opencv2/core.hpp"

//Intrinsic Matrix K

float cameraX = 6.018873000000e+02;
float cameraY = 1.831104000000e+02;
float focalLength = 7.070912000000e+02;
float data[] = {focalLength, 0, cameraX,
                0, focalLength, cameraY,
                0, 0, 1};

cv::Mat IntrinsicK(3, 3, CV_32FC1, data);

void FileRead(std::deque<std::string>& v, std::ifstream &fin);
void MakeTextFile(std::ofstream& fout, const int& imageNum);
