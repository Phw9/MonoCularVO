#pragma once

#include <deque>
#include <fstream>

//Intrinsic Matrix K

// float cameraX = 10.f;
// float cameraY = 10.f;
// float focalLength = 1.f;
// float intrinsicMatrix[] = {focalLength, 0, cameraX,
//                             0, focalLength, cameraY,
//                             0, 0, 1
//                           };
// cv::Mat IntrinsicK(3, 3, CV_32F, intrinsicMatrix);

void FileRead(std::deque<std::string>& v, std::ifstream &fin);
void MakeTextFile(std::ofstream& fout, const int& imageNum);
