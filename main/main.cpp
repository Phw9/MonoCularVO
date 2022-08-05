#include <algorithm>
#include <deque>
#include <fstream>
#include <iostream>
#include <math.h>
#include <string>

#include "Init.h"
#include "BundleAdjustment.h"
#include "CalcMatrix.h"
#include "FeatureDetection.h"
#include "KeyData.h"
#include "Triangulation.h"

#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
// #include "gtest/gtest.h"


float cameraX = 6.018873000000e+02;
float cameraY = 1.831104000000e+02;
float focalLength = 7.070912000000e+02;
float data[] = {focalLength, 0, cameraX,
                0, focalLength, cameraY,
                0, 0, 1};

static cv::Mat IntrinsicK(cv::Size(3, 3), CV_32FC1, data);



int main(int argc, char** argv)
{
	std::ofstream rawData ("../main/image.txt", rawData.out | rawData.trunc);
	std::ifstream read ("../main/image.txt", read.in);
	if(!read.is_open())
	{
		std::cerr << "file can't read image" << std::endl;
		return 0;
	}
	std::deque<std::string> readImage;
	argc = 921;
	int imageCurNum = 0 ;
	MakeTextFile(rawData, argc);
	FileRead(readImage, read);

	mvo::FeatureDescriptor desc1;
	mvo::FeatureDescriptor desc2;
	mvo::CalcMatrix calcM;

	//Triangulate Points Use
	cv::Vec4f homoPoints4D;
	std::vector<cv::Vec4f> globalHomoPositions4D;
	int gHP = 0;
	globalHomoPositions4D.push_back({1,1,1,1});
	gHP++;

	mvo::LocalPoints localPoints;
	std::vector<mvo::LocalPoints> globalLocalPoints;
	// int gLP = 0;

	std::vector<cv::Mat> globalFeaturePoints;
	int gFP = 0;

	std::vector<mvo::CalcMatrix> globalPose;
	int gP = 0;

	// Cur Position
	cv::Vec3f worldPosition;
	std::vector<cv::Vec3f> globalWorldPositions;
	int gWP = 0;
	globalWorldPositions.push_back({1.0f, 1.0f, 1.0f});
	gWP++;



	while(true)
	{
		cv::Mat img;
		img = cv::imread(readImage.front(), cv::ImreadModes::IMREAD_UNCHANGED);
		if (img.empty())
		{
			std::cerr << "frame upload failed" << std::endl;
		}
		readImage.pop_front();
		imageCurNum++;

		if (imageCurNum==1)
		{
			if(!desc1.ConerFAST(img))
			{
				std::cerr << "imageCurNum 1" << std::endl;
			}
			else
			{
				std::cout << desc1.mfastKeyPoints.size() << std::endl;
			}
			
		}
		else if(imageCurNum == 2)
		{

		}
		else if(imageCurNum == 3)
		{
			if(!desc2.ConerFAST(img))
			{
				std::cerr << "imageCurNum 3" << std::endl;
			}else
			{
				std::cout << desc2.mfastKeyPoints.size() << std::endl;
			}

			if(!calcM.CreateEssentialMatrix(desc1, desc2, IntrinsicK))
			{
				std::cerr << "imageCurNum 3" << std::endl;
			}
			calcM.GetEssentialRt(calcM.mEssential, IntrinsicK);
			calcM.CombineRt();
			globalPose.push_back(std::move(calcM));
			std::cout << globalPose[gP].mRotation << std::endl;
			std::cout << globalPose[gP].mTranslation << std::endl;
			std::cout << globalPose[gP].mVecMat1.size() << std::endl;
			std::cout << globalPose[gP].mVecMat2.size() << std::endl;
			gP++;

		}
		else if(imageCurNum == 4)
		{
		}
		else if(imageCurNum == 5)
		{
			if(!desc1.ConerFAST(img))
			{
				std::cerr << "imageCurNum 5" << std::endl;
			};
			// for(cv::KeyPoint kp : desc1.mfastKeyPoints)
			// {
			// 	globalDesc[gDesc].mfastKeyPoints.push_back(kp);
			// }
			// gDesc++;
			if(!calcM.CreateEssentialMatrix(desc1, desc2, IntrinsicK))
			{
				std::cerr << "imageCurNum 5" << std::endl;
			}
			calcM.GetEssentialRt(calcM.mEssential, IntrinsicK);
			calcM.CombineRt();
			globalPose.push_back(std::move(calcM));
			std::cout << globalPose[gP].mRotation << std::endl;
			std::cout << globalPose[gP].mTranslation << std::endl;
			std::cout << globalPose[gP].mVecMat1.size() << std::endl;
			std::cout << globalPose[gP].mVecMat2.size() << std::endl;
			std::cout << globalPose[gP-1].mCombineRt << std::endl;
			std::cout << globalPose[gP].mCombineRt << std::endl;
			gP++;
		}
			
		else
		{
			switch(imageCurNum%5)
			{
				case 1:
				{
					mvo::Triangulate tri;
					if(!tri.CalcWorldPoints(globalPose[gP-2].mCombineRt,
											globalPose[gP-1].mCombineRt,
											globalPose[gP-2].mVecMat1, globalPose[gP-1].mVecMat1))
					{
						std::cerr << "Failed to calcuate Triangulate" << std::endl;
					}
					// std::cout <<tri.mworldMapPoints.row(3) << std::endl;
					// std::cout << tri.mworldMapPoints.rows << std::endl;
					// std::cout <<tri.mworldMapPoints << std::endl;
					// std::cout << tri.mworldMapPoints.size() << std::endl;

					if(!tri.ScalingPoints())
					{
						std::cerr << "Failed to scale mwroldPoints" << std::endl;
					}
					// std::cout << tri.mworldMapPoints.size() << std::endl;
					// std::cout << tri.mworldMapPoints.row(tri.mworldMapPoints.rows-1) << std::endl;
					globalFeaturePoints.push_back(std::move(tri.mworldMapPoints));
					gFP++;
					// std::cout << globalFeaturePoints[0].size() << std::endl;


					break;
					
				}
				case 2:
				{
					break;
				}
				case 3:
				{
					break;
				}
				case 4:
				{
					break;
				}
				case 0:
				{
					
				}
			}
		
		} //if

		cv::imshow("img",img);
	
		if(cv::waitKey(0) == 27) break;	//ESC key	
	}
	std::cout << "globalWorldPositions" << globalWorldPositions.size() << std::endl;
	std::cout << "global Pose: " << globalPose.size() <<std::endl;
	std::cout << "globalHomoPositions4D: " << globalHomoPositions4D.size() <<std::endl;
	cv::destroyAllWindows();

	return 0;
}

