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
	int imageCurNum = 0;
	argc = 921;
	
	MakeTextFile(rawData, argc);
	FileRead(readImage, read);

	mvo::FeatureDetect desc1;
	// mvo::FeatureDetect desc2;
	mvo::FeatureTracking tracker1;
	mvo::FeatureTracking tracker2;
	mvo::CalcMatrix calcM;
	mvo::Triangulate tri;

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
	globalWorldPositions.push_back({0, 0, 0});
	gWP++;



	while(true)
	{
		cv::Mat img;
		img = cv::imread(readImage.at(imageCurNum), cv::ImreadModes::IMREAD_UNCHANGED);
		if (img.empty() || imageCurNum == (argc-1))
		{
			std::cerr << "frame upload failed" << std::endl;
		}
		imageCurNum++;

		if (imageCurNum==1)
		{
			if(!desc1.GoodFeatureToTrack(img))
			{
				std::cerr << "imageCurNum 1" << std::endl;
			}
			else
			{
				std::cout << desc1.mfeatures.size() << std::endl;
			}

			// if(!desc1.ConerFAST(img))
			// {
			// 	std::cerr << "imageCurNum 1" << std::endl;
			// }
			// else
			// {
			// 	std::cout << desc1.mfastKeyPoints.size() << std::endl;
			// }

			// GFTT
			
			
		}
		else if(imageCurNum == 2)
		{

		}
		else if(imageCurNum == 3)
		{
			tracker1.OpticalFlowPyrLK(cv::imread(readImage.at(imageCurNum-2), cv::ImreadModes::IMREAD_UNCHANGED), img, desc1.mfeatures);
			std::cout << tracker1.mfeatures.size() << std::endl;
			std::cout << desc1.mfeatures.size() << std::endl;

			if(!calcM.CreateEssentialMatrix(desc1.mfeatures, tracker1.mfeatures, IntrinsicK))
			{
				std::cerr << "imageCurNum 3" << std::endl;
			}
			calcM.GetEssentialRt(calcM.mEssential, IntrinsicK, desc1.mfeatures, tracker1.mfeatures);
			calcM.CombineRt();
			globalPose.push_back(std::move(calcM));
			std::cout << globalPose[gP].mRotation << std::endl;
			std::cout << globalPose[gP].mTranslation << std::endl;
			gP++;

			// if(!desc2.ConerFAST(img))
			// {
			// 	std::cerr << "imageCurNum 3" << std::endl;
			// }else
			// {
			// 	std::cout << desc2.mfastKeyPoints.size() << std::endl;
			// }

			// if(!calcM.CreateEssentialMatrix(desc1, desc2, IntrinsicK))
			// {
			// 	std::cerr << "imageCurNum 3" << std::endl;
			// }
			// calcM.GetEssentialRt(calcM.mEssential, IntrinsicK);
			// calcM.CombineRt();
			// globalPose.push_back(std::move(calcM));
			// std::cout << globalPose[gP].mRotation << std::endl;
			// std::cout << globalPose[gP].mTranslation << std::endl;
			// std::cout << globalPose[gP].mVecMat1.size() << std::endl;
			// std::cout << globalPose[gP].mVecMat2.size() << std::endl;
			// gP++;
			
			

		}
		else if(imageCurNum == 4)
		{
		}
		else if(imageCurNum == 5)
		{

			tracker2.OpticalFlowPyrLK(cv::imread(readImage.at(imageCurNum-2), cv::ImreadModes::IMREAD_UNCHANGED), img, tracker1.mfeatures);
			std::cout << tracker2.mfeatures.size() << std::endl;
			std::cout << tracker1.mfeatures.size() << std::endl;
			if(!calcM.CreateEssentialMatrix(tracker1.mfeatures, tracker2.mfeatures, IntrinsicK))
			{
				std::cerr << "imageCurNum 3" << std::endl;
			}
			calcM.GetEssentialRt(calcM.mEssential, IntrinsicK, tracker1.mfeatures, tracker2.mfeatures);
			calcM.CombineRt();
			globalPose.push_back(std::move(calcM));
			std::cout << globalPose[gP].mRotation << std::endl;
			std::cout << globalPose[gP].mTranslation << std::endl;
			gP++;

			// if(!desc1.ConerFAST(img))
			// {
			// 	std::cerr << "imageCurNum 5" << std::endl;
			// }else
			// {
			// 	std::cout << desc1.mfastKeyPoints.size() << std::endl;
			// }

			// if(!calcM.CreateEssentialMatrix(desc1, desc2, IntrinsicK))
			// {
			// 	std::cerr << "imageCurNum 5" << std::endl;
			// }
			// calcM.GetEssentialRt(calcM.mEssential, IntrinsicK);
			// calcM.CombineRt();
			// globalPose.push_back(std::move(calcM));
			// std::cout << globalPose[gP].mRotation << std::endl;
			// std::cout << globalPose[gP].mTranslation << std::endl;
			// std::cout << globalPose[gP].mVecMat1.size() << std::endl;
			// std::cout << globalPose[gP].mVecMat2.size() << std::endl;
			// std::cout << globalPose[gP-1].mCombineRt << std::endl;
			// std::cout << globalPose[gP].mCombineRt << std::endl;
			// gP++;


		}
			
		else
		{
			switch(imageCurNum%5)
			{
				case 1:
				{
					tracker1.OpticalFlowPyrLK(cv::imread(readImage.at(imageCurNum-2), cv::ImreadModes::IMREAD_UNCHANGED), img, tracker2.mfeatures);
					std::cout << tracker2.mfeatures.size() << std::endl;
					std::cout << tracker1.mfeatures.size() << std::endl;
					if(!calcM.CreateEssentialMatrix(tracker2.mfeatures, tracker1.mfeatures, IntrinsicK))
					{
						std::cerr << "imageCurNum 3" << std::endl;
					}
					calcM.GetEssentialRt(calcM.mEssential, IntrinsicK, tracker2.mfeatures, tracker1.mfeatures);
					calcM.CombineRt();
					globalPose.push_back(std::move(calcM));
					std::cout << globalPose[gP].mRotation << std::endl;
					std::cout << globalPose[gP].mTranslation << std::endl;
					gP++;

					if(!tri.CalcWorldPoints(globalPose[gP-2].mCombineRt, globalPose[gP-1].mCombineRt, tracker2.mfeatures, tracker1.mfeatures))
					{
						std::cerr << "failed to calculate triangulatePoints" << std::endl;
					}
					if(!tri.ScalingPoints())
					{
						std::cerr << "failed to Scale Points" << std::endl;
					}
					globalFeaturePoints[gFP] = tri.mworldMapPoints;
					gFP++;

					std::cout << globalFeaturePoints[gFP-1].size() << std::endl;


					// mvo::Triangulate tri;
					// if(!tri.CalcWorldPoints(globalPose[gP-2].mCombineRt,
					// 						globalPose[gP-1].mCombineRt,
					// 						globalPose[gP-2].mVecMat1, globalPose[gP-1].mVecMat1))
					// {
					// 	std::cerr << "Failed to calcuate Triangulate" << std::endl;
					// }
					// // std::cout <<tri.mworldMapPoints.row(3) << std::endl;
					// // std::cout << tri.mworldMapPoints.rows << std::endl;
					// // std::cout <<tri.mworldMapPoints << std::endl;
					// // std::cout << tri.mworldMapPoints.size() << std::endl;

					// if(!tri.ScalingPoints())
					// {
					// 	std::cerr << "Failed to scale mwroldPoints" << std::endl;
					// }
					// // std::cout << tri.mworldMapPoints.size() << std::endl;
					// // std::cout << tri.mworldMapPoints.row(tri.mworldMapPoints.rows-1) << std::endl;
					// globalFeaturePoints.push_back(std::move(tri.mworldMapPoints));
					// gFP++;
					// // std::cout << globalFeaturePoints[0].size() << std::endl;


					break;
					
				}
				case 2:
				{
					break;
				}
				case 3:
				{
					tracker2.OpticalFlowPyrLK(cv::imread(readImage.at(imageCurNum-2), cv::ImreadModes::IMREAD_UNCHANGED), img, tracker1.mfeatures);
					std::cout << tracker1.mfeatures.size() << std::endl;
					std::cout << tracker2.mfeatures.size() << std::endl;
					if(!calcM.CreateEssentialMatrix(tracker1.mfeatures, tracker2.mfeatures, IntrinsicK))
					{
						std::cerr << "imageCurNum 3" << std::endl;
					}
					calcM.GetEssentialRt(calcM.mEssential, IntrinsicK, tracker1.mfeatures, tracker1.mfeatures);
					calcM.CombineRt();
					globalPose.push_back(std::move(calcM));
					std::cout << globalPose[gP].mRotation << std::endl;
					std::cout << globalPose[gP].mTranslation << std::endl;
					gP++;

					if(!tri.CalcWorldPoints(globalPose[gP-2].mCombineRt, globalPose[gP-1].mCombineRt, tracker1.mfeatures, tracker2.mfeatures))
					{
						std::cerr << "failed to calculate triangulatePoints" << std::endl;
					}
					if(!tri.ScalingPoints())
					{
						std::cerr << "failed to Scale Points" << std::endl;
					}
					globalFeaturePoints[gFP] = tri.mworldMapPoints;
					gFP++;

					std::cout << globalFeaturePoints[gFP-1].size() << std::endl;
					break;
				}
				case 4:
				{
					if(tracker2.mfeatures.size() < 100)
					{
						if(!desc1.GoodFeatureToTrack(img))
						{
							std::cerr << "imageCurNum " << imageCurNum << std::endl;
						}
						else
						{
							tracker2.mfeatures = std::move(desc1.mfeatures);
							std::cout << desc1.mfeatures.size() << std::endl;
							std::cout << tracker2.mfeatures.size() << std::endl;
						}
					}
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

