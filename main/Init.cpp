#include "Init.h"

void FileRead(std::deque<std::string>& v, std::ifstream &fin)
{
	std::string line;
	while(true)
	{
		getline(fin, line);
		if(fin.eof()) break;
		v.emplace_back(line);
	}
}
void MakeTextFile(std::ofstream& fout, const int& imageNum)
{
    for(int i = 0; i<imageNum; i++)
	{
		fout << "../image/image_0/";
		fout.width(6);
		fout.fill('0');
		// fout.right;
		fout << i;
		fout << ".png" << std::endl;
	}
}



void GTPoseRead(std::vector<cv::Mat>& m, std::ifstream& fin)
{
	char buf[100];
	int numOfGT = 0;
	while(true)
	{
		cv::Mat temp(cv::Size(4,3), CV_64FC1, 0.0);
		int num = 0;

		for(int j = 0; j< temp.rows; j++)
		{
			for(int i = 0; i< temp.cols; i++)
			{
				if(num == 11)
				{
					fin.getline(buf, 100, '\n');
					double tempd = atof(buf);
					temp.at<double>(j,i) = tempd;
					break;
				}
				fin.getline(buf, 100, ' ');
				double tempd = atof(buf);
				temp.at<double>(j,i) = tempd;
				num++;
			}
		}
		numOfGT++;
		m.emplace_back(std::move(temp));
		if(numOfGT == 4541) break;
		// if(fin.eof()) break;
	}

	std::cout << m[m.size()-1] << std::endl;
}

// /*
//     pangolin :: visualioze GT,RE Trajectory 
//     cv :: visualize 2D features points 
//     (current Features, Triangulated Features, attached Features)
// */


Viewer::my_visualize::my_visualize(int width,int height)
{
    this->window_width=width;
    this->window_height=height;
    this->window_ratio=(float)width/height;
}

void Viewer::my_visualize::initialize()
{
    pangolin::CreateWindowAndBind("TrajectoryViewer", window_width, window_height);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void Viewer::my_visualize::active_cam()
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
void Viewer::my_visualize::draw_point(std::vector<cv::Mat>& gtPose, std::vector<cv::Mat>& pose, std::vector<cv::Mat>& allOfPoints, cv::Mat fovPoints)
{
    glClearColor(1.0f,1.0f,1.0f,1.0f);
    if(gtPose.size()==0 || pose.size()==0)
	{
        return;
    }
    else
	{
        //빨간색(첫번째) : 구한포즈, 파란색(두번째) : 지티포즈, 검은색(세번째): 모든 맵 , 자홍색(네번째) : 현재 키프레임의 맵
        glPointSize(3);
        glBegin(GL_POINTS);
        glColor3f(1.0,0.0,0.0); //

        for(int i=0;i<gtPose.size();i++)
	    {
            glVertex3f(gtPose[i].at<float>(0,0),0.0f, gtPose[i].at<float>(2,0));
        }
        glEnd();

        glPointSize(3);
        glBegin(GL_POINTS);
        glColor3f(0.0,0.0,1.0);

        for(int i=0;i<pose.size();i++)
	    {
                glVertex3f(pose[i].at<float>(0,0),0.0f, pose[i].at<float>(2,0));
        }
        glEnd();

        glPointSize(1);
        glBegin(GL_POINTS);
        glColor3f(0.0,0.0,0.0);

        for(int i = 0; i < allOfPoints.size(); i++)
	    {
           for(int j = 0; j < allOfPoints[i].cols; j++)
           {
            glVertex3f(allOfPoints[i].at<float>(0,j), 0.0f, allOfPoints[i].at<float>(2,j));
           }
        }
        glEnd();      

        glPointSize(2);
        glBegin(GL_POINTS);
        glColor3f(1.0,0.0,1.0);

        for(int i = 0; i < fovPoints.cols; i++)
	    {
            glVertex3f(fovPoints.at<double>(0,i), 0.0f, fovPoints.at<double>(2,i));
        }
        glEnd();            
    }
}

// circle is before, rectangle is after
cv::Mat Viewer::my_visualize::cv_draw_features(cv::Mat src, std::vector<cv::Point2f>& beforePoints, std::vector<cv::Point2f> afterPoints)
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
