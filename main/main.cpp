#include "Init.h"
#include "../include/BundleAdjustment.h"
#include "../include/CalcMatrix.h"
#include "../include/FeatureDetection.h"
#include "../include/KeyData.h"
#include "../include/Triangulation.h"

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
	
	while(true)
	{
		cv::Mat cap;
		cap = cv::imread(readImage.front(), cv::ImreadModes::IMREAD_UNCHANGED);
		if (cap.empty())
		{
			std::cerr << "frame upload failed" << std::endl;
		}
		readImage.pop_front();
		imageCurNum++;
		




		cv::imshow("cap",cap);
	
		if(cv::waitKey(33) == 27) break;	//ESC key	
	}


	cv::destroyAllWindows();

	return 0;
}

