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


int main()
{
	std::ofstream rawData ("main/image.txt", rawData.out | rawData.trunc);
	std::ifstream read ("main/image.txt", read.in);
	if(!read.is_open())
	{
		std::cerr << "file can't read image" << std::endl;
		return 0;
	}
	std::deque<std::string> readImage;
	int imageNum = 921;
	int imageCurNum = 0 ;
	MakeTextFile(rawData, imageNum);
	FileRead(readImage, read);

	//Triangulate Points
	mvo::HomoVec homoPoints4D;
	std::vector<mvo::HomoVec> globalHomoPoints4D;

	mvo::FeaturePoint keyPoints;
	std::vector<mvo::FeaturePoint> globalKeyPoints;
	int gKP = 1;

	mvo::LocalPoints localPoints;
	std::vector<mvo::LocalPoints> globalLocalPoints;
	int gLP = 0;

	mvo::LocalPoints featurePoints;
	std::vector<mvo::LocalPoints> globalFeaturePoints;
	int gFP = 0;

	// Pose
	mvo::KeyFrame keyFrames;
	std::vector<cv::Mat> globalKeyFrames;
	int gKF = 0;

	cv::Point3f worldPosition;
	std::vector<cv::Point3f> globalWorldPositions;
	globalWorldPositions.push_back({1.0f, 1.0f, 1.0f});
	int gWP = 0;

	mvo::FeatureDescriptor desc1;
	mvo::FeatureDescriptor desc2;

	mvo::CalcMatrix calcM;

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

			// for(cv::KeyPoint kp : desc1.mfastKeyPoints)
			// {
			// 	globalKeyPoints[gKP].mfeaturePoints.push_back(kp);
			// }
			// gKP++;
			
		}
		else if(imageCurNum == 2)
		{

		}
		else if(imageCurNum == 3)
		{
			if(!desc2.ConerFAST(img))
			{
				std::cerr << "imageCurNum 3" << std::endl;
			};
			// for(cv::KeyPoint kp : desc2.mfastKeyPoints)
			// {
			// 	globalKeyPoints[gKP].mfeaturePoints.emplace_back(kp);
			// }
			// gKP++;
			if(!calcM.CreateHomographyMatrix(desc1, desc2))
			{
				std::cerr << "imageCurNum 3" << std::endl;
			}
			globalKeyFrames.push_back(calcM.mHomography);
			std::cout << globalKeyFrames.size() << std::endl;
			std::cout << globalKeyFrames[gKF] << std::endl;
			globalWorldPositions.push_back(mvo::DotProduct3D(globalKeyFrames[gKF++], globalWorldPositions[gWP++]));
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
			// for(cv::KeyPoint kp : desc2.mfastKeyPoints)
			// {
			// 	globalKeyPoints[gKP].mfeaturePoints.emplace_back(kp);
			// }
			// gKP++;
			if(!calcM.CreateHomographyMatrix(desc1, desc2))
			{
				std::cerr << "imageCurNum 5" << std::endl;
			}
			globalKeyFrames.push_back(calcM.mHomography);
			std::cout << globalKeyFrames.size() << std::endl;
			std::cout << globalKeyFrames[gKF] << std::endl;
			globalWorldPositions.push_back(mvo::DotProduct3D(globalKeyFrames[gKF++], globalWorldPositions[gWP++]));
		}
		else
		{
			switch(imageCurNum%5)
			{
				case 1:
				{
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
	std::cout << globalWorldPositions.size() << std::endl;
	cv::destroyAllWindows();

	return 0;
}

