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

#define WINDOWWIDTH 1024
#define WINDOWHEIGHT 768
#define VIEWPOINTF 20.0
#define VIEWPOINTX 0.0
#define VIEWPOINTY -100.0
#define VIEWPOINTZ -0.1



float cameraX = 6.071928000000e+02;
float cameraY = 1.852157000000e+02;
float focalLength = 7.188560000000e+02;
float data[] = {focalLength, 0, cameraX,
                0, focalLength, cameraY,
                0, 0, 1};				

static cv::Mat intrinsicK(cv::Size(3, 3), CV_32FC1, data);

static const int minOfTrackPoints = 100;



int main(int argc, char** argv)
{ 
	std::ofstream rawData ("../main/image.txt", rawData.out | rawData.trunc);
	std::ifstream read ("../main/image.txt", read.in);
	std::ifstream readGTPose ("../image/GTpose.txt", readGTPose.in);
    cv::Mat temp(cv::Size(4,3), CV_64FC1, 0.0);
    
	

	if(!read.is_open())
	{
		std::cerr << "file can't read image" << std::endl;
		return 0;
	}
	std::deque<std::string> readImageName;
	int imageCurNum = 0;
	argc = 4540;
	
	std::vector<cv::Vec3f> GTPose;
	int GTP = 0;
	
	MakeTextFile(rawData, argc);
	FileRead(readImageName, read);
	GTPoseRead(GTPose, readGTPose);



	mvo::FeatureDetect sfm;
	mvo::FeatureDetect fastCorner;
	mvo::FeatureTracking tracker1;
	mvo::FeatureTracking tracker2;
	mvo::CalcMatrix calcM;
	mvo::Triangulate tri;

	//Triangulate Points
	cv::Mat positions3D;
	double initialData[] = {0,0,0,1};
	std::vector<cv::Mat> globalMyPositions;
	std::vector<cv::Vec3f> globalGTPositions;
	int gHP = 0;
	globalMyPositions.emplace_back(cv::Mat(cv::Size(1,4), CV_64FC1, initialData));
	globalGTPositions.emplace_back(cv::Mat(cv::Size(1,4), CV_64FC1, initialData));
	gHP++;

	std::vector<cv::Mat> globalMapPoints;	//(4xN)
	int gMP = 0;

	std::vector<mvo::CalcMatrix> globalPose;
	int gP = 0;


	Viewer::my_visualize pangolinViewer=Viewer::my_visualize(WINDOWWIDTH, WINDOWHEIGHT);
    pangolinViewer.initialize();
    pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(WINDOWWIDTH, WINDOWHEIGHT, VIEWPOINTF, VIEWPOINTF, 512, 389, 0.1, 1000),
    pangolin::ModelViewLookAt(VIEWPOINTX, VIEWPOINTY, VIEWPOINTZ, 0, 0, 0, 0.0, -1.0, 0.0));
    pangolin::View &d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -pangolinViewer.window_ratio)
                                .SetHandler(new pangolin::Handler3D(s_cam));

	while(true)
	{
		cv::Mat img;
		img = cv::imread(readImageName.at(imageCurNum), cv::ImreadModes::IMREAD_UNCHANGED);
		if (img.empty() || imageCurNum == (argc-1))
		{
			std::cerr << "frame upload failed" << std::endl;
		}
		imageCurNum++;

		if (imageCurNum==1)
		{
			if(!sfm.GoodFeatureToTrack(img))
			{
				std::cerr << "imageCurNum 1" << std::endl;
			}
			else
			{
				std::cout << "GFTT size : " << sfm.mfeatures.size() << std::endl;
			}

			cv::cvtColor(img, img, cv::ColorConversionCodes::COLOR_GRAY2BGR);
			img = pangolinViewer.cv_draw_features(img, sfm.mfeatures, sfm.mfeatures);
			
		}
		else if(imageCurNum == 2){}
		else if(imageCurNum == 3)
		{
			tracker1.OpticalFlowPyrLK(cv::imread(readImageName.at(imageCurNum-2), cv::ImreadModes::IMREAD_UNCHANGED), img, sfm.mfeatures);
			std::cout <<"after tracked size: "<< tracker1.mfeatures.size() << std::endl;
			std::cout <<"after tracking size: "<< sfm.mfeatures.size() << std::endl;
			
			cv::cvtColor(img, img, cv::ColorConversionCodes::COLOR_GRAY2BGR);
			img = pangolinViewer.cv_draw_features(img,tracker1.mfeatures, sfm.mfeatures);

			if(!calcM.CreateEssentialMatrix(sfm.mfeatures, tracker1.mfeatures, intrinsicK))
			{
				std::cerr << "imageCurNum 3" << std::endl;
			}
			calcM.GetEssentialRt(calcM.mEssential, intrinsicK, sfm.mfeatures, tracker1.mfeatures);
			calcM.CombineRt();
			globalPose.emplace_back(std::move(calcM));
			gP++;
			std::cout << "globalPose" << gP-1 << ": " << globalPose[gP-1].mCombineRt << std::endl;

			positions3D = mvo::GetPosePosition(globalPose[gP-1].mCombineRt, globalMyPositions[gHP-1]);
			globalMyPositions.emplace_back(std::move(positions3D));
			gHP++;
			for(int i = 0; i < 12; i++)
			{
				globalGTPositions.emplace_back(std::move(GTPose[GTP+i]));
			}
			GTP += 12;

			std::cout << "globalGTPose" << GTP-1 << ": " << GTPose[GTP-1] << std::endl;
			std::cout << "globalGTPositions"<< GTP-1 << "th: " << globalGTPositions[GTP-1]<< std::endl;
		}
		else if(imageCurNum == 4){}
		else if(imageCurNum == 5)
		{
			tracker2.OpticalFlowPyrLK(cv::imread(readImageName.at(imageCurNum-2), cv::ImreadModes::IMREAD_UNCHANGED), img, tracker1.mfeatures);
			std::cout << "after tracked size: "<<tracker2.mfeatures.size() << std::endl;
			std::cout << "after tracking size: "<<tracker1.mfeatures.size() << std::endl;
			
			cv::cvtColor(img, img, cv::ColorConversionCodes::COLOR_GRAY2BGR);
			img = pangolinViewer.cv_draw_features(img,tracker2.mfeatures, tracker1.mfeatures);
			
			if(!calcM.CreateEssentialMatrix(tracker1.mfeatures, tracker2.mfeatures, intrinsicK))
			{
				std::cerr << "imageCurNum 3" << std::endl;
			}
			calcM.GetEssentialRt(calcM.mEssential, intrinsicK, tracker1.mfeatures, tracker2.mfeatures);
			calcM.CombineRt();
			globalPose.emplace_back(std::move(calcM));
			gP++;
			
			std::cout << "globalPose" << gP-1 << ": " << globalPose[gP-1].mCombineRt << std::endl;
			
			positions3D = mvo::GetPosePosition(globalPose[gP-1].mCombineRt, globalMyPositions[gHP-1]);
			globalMyPositions.emplace_back(std::move(positions3D));
			gHP++;
			
			std::cout << "globalMyPositions" << gHP-1 << ": " << globalMyPositions[gHP-1] << std::endl;
			
			for(int i = 0; i < 12; i++)
			{
				globalGTPositions.emplace_back(std::move(GTPose[GTP+i]));
			}
			GTP += 12;
			
			std::cout << "globalGTPose" << GTP-1 << ": " << GTPose[GTP-1] << std::endl;
			std::cout << "globalGTPositions"<< GTP-1 << "th: " << globalGTPositions[GTP-1]<< std::endl;
		}
			
		else
		{
			switch(imageCurNum%5)
			{
				case 1:
				{
					tracker1.OpticalFlowPyrLK(cv::imread(readImageName.at(imageCurNum-2), cv::ImreadModes::IMREAD_UNCHANGED), img, tracker2.mfeatures);
					std::cout <<"after tracked size: "<< tracker2.mfeatures.size() << std::endl;
					std::cout <<"after tracking size: "<< tracker1.mfeatures.size() << std::endl;
					
					cv::cvtColor(img, img, cv::ColorConversionCodes::COLOR_GRAY2BGR);
					img = pangolinViewer.cv_draw_features(img,tracker2.mfeatures, tracker1.mfeatures);
					
					if(!calcM.CreateEssentialMatrix(tracker2.mfeatures, tracker1.mfeatures, intrinsicK))
					{
						std::cerr << "imageCurNum 3" << std::endl;
					}
					calcM.GetEssentialRt(calcM.mEssential, intrinsicK, tracker2.mfeatures, tracker1.mfeatures);
					calcM.CombineRt();
					globalPose.emplace_back(std::move(calcM));
					gP++;

					std::cout << "globalPose" << gP-1 << ": " << globalPose[gP-1].mCombineRt << std::endl;
					
					positions3D = mvo::GetPosePosition(globalPose[gP-1].mCombineRt, globalMyPositions[gHP-1]);
					globalMyPositions.emplace_back(std::move(positions3D));
					gHP++;

					std::cout << "size of globalMyPositions" << gHP-1 << ": " << globalMyPositions[gHP-1].size() << std::endl;

					for(int i = 0; i < 12; i++)
					{
						globalGTPositions.emplace_back(std::move(GTPose[GTP+i]));
					}
					GTP += 12;
					
					std::cout << "globalGTPose" << GTP-1 << ": " << GTPose[GTP-1] << std::endl;
					std::cout << "globalGTPositions"<< GTP-1 << "th: " << globalGTPositions[GTP-1]<< std::endl;

					if(!tri.CalcWorldPoints(globalPose[gP-2].mCombineRt, globalPose[gP-1].mCombineRt, tracker2.mfeatures, tracker1.mfeatures))
					{
						std::cerr << "failed to calculate triangulatePoints" << std::endl;
					}

					std::cout << "after triangulate Point size: " << tri.mworldMapPoints.size() << std::endl;
					// std::cout << "after triangulate Points: " << tri.mworldMapPoints << std::endl;

					if(!tri.ScalingPoints())
					{
						std::cerr << "failed to Scale Points" << std::endl;
					}
					globalMapPoints.emplace_back(tri.mworldMapPoints);
					gMP++;
					
					std::cout << "check to insert global size: " << globalMapPoints.size() << std::endl;
					std::cout << "MapPoints at the Moment by global: " << globalMapPoints[gMP-1].size() << std::endl;
					std::cout << "MapPoints at the Moment by tri.mworldMapPoints: " << tri.mworldMapPoints.size() << std::endl;

					break;
					
				}
				case 2:
				{
					if(tracker1.mfeatures.size() < minOfTrackPoints)
					{
						if(!sfm.GoodFeatureToTrack(img))
						{
							std::cerr << "imageCurNum " << imageCurNum << std::endl;
						}
						else
						{
							tracker1.mfeatures = std::move(sfm.mfeatures);
							std::cout << "new GFTT size: " << sfm.mfeatures.size() << std::endl;
							std::cout << "move to new GFTT size: " << tracker1.mfeatures.size() << std::endl;
						}
					}
					break;
				}
				case 3:
				{
					tracker2.OpticalFlowPyrLK(cv::imread(readImageName.at(imageCurNum-2), cv::ImreadModes::IMREAD_UNCHANGED), img, tracker1.mfeatures);
					std::cout << "after tracked size: " << tracker1.mfeatures.size() << std::endl;
					std::cout << "after tracking size: " << tracker2.mfeatures.size() << std::endl;
					
					cv::cvtColor(img, img, cv::ColorConversionCodes::COLOR_GRAY2BGR);
					img = pangolinViewer.cv_draw_features(img,tracker1.mfeatures, tracker2.mfeatures);
					
					if(!calcM.CreateEssentialMatrix(tracker1.mfeatures, tracker2.mfeatures, intrinsicK))
					{
						std::cerr << "imageCurNum 3" << std::endl;
					}
					calcM.GetEssentialRt(calcM.mEssential, intrinsicK, tracker1.mfeatures, tracker1.mfeatures);
					calcM.CombineRt();
					globalPose.emplace_back(std::move(calcM));
					gP++;

					std::cout << "globalPose" << gP-1 << ": " << globalPose[gP-1].mCombineRt << std::endl;

					positions3D = mvo::GetPosePosition(globalPose[gP-1].mCombineRt, globalMyPositions[gHP-1]);
					globalMyPositions.emplace_back(std::move(positions3D));
					gHP++;

					std::cout << "globalMyPositions" << gHP-1 << ": " << globalMyPositions[gHP-1] << std::endl;

					for(int i = 0; i < 12; i++)
					{
						globalGTPositions.emplace_back(std::move(GTPose[GTP+i]));
					}
					GTP += 12;

					std::cout << "globalGTPose" << GTP-1 << ": " << GTPose[GTP-1] << std::endl;
					std::cout << "globalGTPositions"<< GTP-1 << ": " << globalGTPositions[GTP-1]<< std::endl;

					if(!tri.CalcWorldPoints(globalPose[gP-2].mCombineRt, globalPose[gP-1].mCombineRt, tracker1.mfeatures, tracker2.mfeatures))
					{
						std::cerr << "failed to calculate triangulatePoints" << std::endl;
					}
					if(!tri.ScalingPoints())
					{
						std::cerr << "failed to Scale Points" << std::endl;
					}
					globalMapPoints.emplace_back(tri.mworldMapPoints);
					gMP++;
					
					std::cout << "check to insert global size: " << globalMapPoints.size() << std::endl;
					std::cout << "MapPoints at the Moment by global: " << globalMapPoints[gMP-1].size() << std::endl;
					std::cout << "MapPoints at the Moment by tri.mworldMapPoints: " << tri.mworldMapPoints.size() << std::endl;


					break;

				}
				case 4:
				{
					if(tracker2.mfeatures.size() < minOfTrackPoints)
					{
						if(!sfm.GoodFeatureToTrack(img))
						{
							std::cerr << "imageCurNum " << imageCurNum << std::endl;
						}
						else
						{
							tracker2.mfeatures = std::move(sfm.mfeatures);
							std::cout << "new GFTT size: " << sfm.mfeatures.size() << std::endl;
							std::cout << "move to new GFTT size: " << tracker2.mfeatures.size() << std::endl;
						}
					}
					break;
				}
				case 0:
				{
					
				}
			}
		
		} //if
		// std::cout << globalGTPositions[0] << std::endl;
		// std::cout << globalMyPositions[0] << std::endl;
		// std::cout << globalMapPoints[0] << std::endl;
		// std::cout << tri.[0] << std::endl;
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		d_cam.Activate(s_cam);
		pangolinViewer.draw_point(globalMyPositions, globalGTPositions, globalMapPoints, tri.mworldMapPoints);

    	pangolin::FinishFrame();
		cv::imshow("img",img);
	
		if(cv::waitKey(5) == 27) break;	//ESC key	
	}

	std::cout << "globalMapPoints size: " << globalMapPoints.size() << std::endl;
	std::cout << "globalPose size: " << globalPose.size() <<std::endl;	//367
	std::cout << "globalMyPositions size: " << globalMyPositions.size() <<std::endl;
	std::cout << "GTPose size: " << GTPose.size() << std::endl;
	std::cout << "globalGTPositions size: " << globalGTPositions.size() <<std::endl;
	cv::destroyAllWindows();
	

	return 0;
}