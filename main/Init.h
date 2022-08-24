#pragma once

#include <deque>
#include <fstream>
#include <string>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

#include <pangolin/display/display.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/handler/handler_image.h>
#include <pangolin/image/image_utils.h>
#include <pangolin/display/view.h>
#include <pangolin/handler/handler.h>
#include <pangolin/gl/viewport.h>
#include <pangolin/utils/range.h>

#include <functional>

#include <stdio.h>
#include <iostream>



void FileRead(std::deque<std::string>& v, std::ifstream &fin);
void MakeTextFile(std::ofstream& fout, const int& imageNum);
void GTPoseRead(std::vector<cv::Mat>& m, std::ifstream& fin);

/*
    pangolin :: visualioze GT,RE Trajectory 
    cv :: visualize 2D features points 
    (current Features, Triangulated Features, attached Features)
*/


// namespace Viewer
// {
//     class my_visualize
// 	{
//         private:
//             int window_width;
//             int window_height;

//         public:
//             float window_ratio;

//             //생성자
//             my_visualize(int width,int height)
// 			{
//                 this->window_width=width;
//                 this->window_height=height;
//                 this->window_ratio=(float)width/height;
//             }


//             void initialize()
// 			{
//                 pangolin::CreateWindowAndBind("TrajectoryViewer", window_width, window_height);
//                 glEnable(GL_DEPTH_TEST);
//                 glEnable(GL_BLEND);
//                 glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//             }

//             void active_cam()
// 			{
//                 pangolin::OpenGlRenderState s_cam(
//                     pangolin::ProjectionMatrix(window_width, window_height, 20, 20, 512, 389, 0.1, 1000),
//                     pangolin::ModelViewLookAt(0, -100, -0.1, 0, 0, 0, 0.0, -1.0, 0.0));

//                 pangolin::View &d_cam = pangolin::CreateDisplay()
//                                             .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -window_ratio)
//                                             .SetHandler(new pangolin::Handler3D(s_cam)); 
//                 d_cam.Activate(s_cam);
//             }

//             // pts1, pts2, pts3, pts4
//             void draw_point(std::vector<cv::Point3f>& gtPose, std::vector<cv::Point3f>& pose, std::vector<cv::Point3d>& allOfPoints, cv::Mat fovPoints)
// 			{
//                 glClearColor(1.0f,1.0f,1.0f,1.0f);
//                 if(gtPose.size()==0 || pose.size()==0)
// 				{
//                     return;
//                 }
//                 else
// 				{
//                     glPointSize(3);
//                     glBegin(GL_POINTS);
//                     glColor3f(1.0,0.0,0.0);

//                     for(int i=0;i<gtPose.size();i++)
// 					{
//                         glVertex3f((float)gtPose[i].x,0,(float)gtPose[i].z);
//                     }
//                     glEnd();

//                     glPointSize(3);
//                     glBegin(GL_POINTS);
//                     glColor3f(0.0,0.0,1.0);

//                     for(int i=0;i<pose.size();i++)
// 					{
//                         glVertex3f((float)pose[i].x,0,(float)pose[i].z);
//                     }
//                     glEnd();

//                     glPointSize(1);
//                     glBegin(GL_POINTS);
//                     glColor3f(0.0,0.0,0.0);

//                     for(int i=0;i<allOfPoints.size();i++)
// 					{
//                         glVertex3f((double)allOfPoints[i].x,0,(double)allOfPoints[i].z);
//                     }
//                     glEnd();      

//                     glPointSize(2);
//                     glBegin(GL_POINTS);
//                     glColor3f(1.0,0.0,1.0);

//                     for(int i=0;i<fovPoints.rows;i++)
// 					{
//                         glVertex3f(fovPoints.at<double>(i,0),0,fovPoints.at<double>(i,2));
//                     }
//                     glEnd();            
//                 }
//             }


namespace Viewer
{
    class my_visualize
	{
        private:
            int window_width;
            int window_height;

        public:
            float window_ratio;

            //생성자
            my_visualize(int width,int height)
			{
                this->window_width=width;
                this->window_height=height;
                this->window_ratio=(float)width/height;
            }


            void initialize()
			{
                pangolin::CreateWindowAndBind("TrajectoryViewer", window_width, window_height);
                glEnable(GL_DEPTH_TEST);
                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            }

            void active_cam()
			{
                pangolin::OpenGlRenderState s_cam(
                    pangolin::ProjectionMatrix(window_width, window_height, 20, 20, 512, 389, 0.1, 1000),
                    pangolin::ModelViewLookAt(0, -100, -0.1, 0, 0, 0, 0.0, -1.0, 0.0));

                pangolin::View &d_cam = pangolin::CreateDisplay()
                                            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -window_ratio)
                                            .SetHandler(new pangolin::Handler3D(s_cam)); 
                d_cam.Activate(s_cam);
            }

            // pts1: GT Pose, pts2: Pose, pts3: 3D Points, pts4: FOV of 3D Points
            void draw_point(std::vector<cv::Mat>& gtPose, std::vector<cv::Mat>& pose, std::vector<cv::Mat>& allOfPoints, cv::Mat fovPoints)
			{
                glClearColor(1.0f,1.0f,1.0f,1.0f);
                if(gtPose.size()==0 || pose.size()==0)
				{
                    return;
                }
                else
				{
                    glPointSize(3);
                    glBegin(GL_POINTS);
                    glColor3f(1.0,0.0,0.0);

                    for(int i=0;i<gtPose.size();i++)
					{
                        glVertex3f(gtPose[i].at<float>(0,0),0, gtPose[i].at<float>(2,0));
                    }
                    glEnd();

                    glPointSize(3);
                    glBegin(GL_POINTS);
                    glColor3f(0.0,0.0,1.0);

                    for(int i=0;i<pose.size();i++)
					{
                        glVertex3f(gtPose[i].at<float>(0,0),0, gtPose[i].at<float>(2,0));
                    }
                    glEnd();

                    glPointSize(1);
                    glBegin(GL_POINTS);
                    glColor3f(0.0,0.0,0.0);

                    for(int i = 0; i < allOfPoints.size(); i++)
					{
                        for(int j = 0; j < allOfPoints[i].cols; j++)
                        glVertex3f(allOfPoints[i].at<float>(0,j), 0, allOfPoints[i].at<float>(2,j));
                    }
                    glEnd();      

                    glPointSize(2);
                    glBegin(GL_POINTS);
                    glColor3f(1.0,0.0,1.0);

                    for(int i = 0; i < fovPoints.cols; i++)
					{
                        glVertex3f(fovPoints.at<double>(0,i), 0, fovPoints.at<double>(2,i));
                    }
                    glEnd();            
                }
            }

            // circle is before, rectangle is after
            cv::Mat cv_draw_features(cv::Mat src, std::vector<cv::Point2f>& beforePoints, std::vector<cv::Point2f> afterPoints)
			{
                for (int i = 0; i < beforePoints.size(); i++)
				{
                    //random color
                    int rgb[3];
                    rgb[0]=rand()%256;
                    rgb[1]=rand()%256;
                    rgb[2]=rand()%256;
                    cv::line(src, beforePoints[i], afterPoints[i],cv::Scalar(rgb[0],rgb[1],rgb[2]),1,8,0);
                    cv::circle(src, beforePoints[i], 5, cv::Scalar(rgb[0], rgb[1], rgb[2]), 1, 8, 0); //2d features  
                    // circle(src, afterPoints[i], 6, Scalar(rgb[0], rgb[1], rgb[2]), 1, 8, 0); //3d points
                    cv::rectangle(src, cv::Rect(cv::Point(afterPoints[i].x-5,afterPoints[i].y-5),
                    cv::Point(afterPoints[i].x+5,afterPoints[i].y+5)), cv::Scalar(rgb[0],rgb[1],rgb[2]),1,8,0);//projection 3d points
                }
                return src;    
            }
    };
}