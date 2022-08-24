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



// void GTPoseRead(std::vector<cv::Mat> m, std::ifstream &fin)
// {
// 	char value[100];
// 	cv::Mat temp(cv::Size(4,3), CV_64FC1, 0);
// 	while(true)
// 	{
// 		for(int j = 0; j< temp.rows; j++)
// 		{
// 			for(int i = 0; i< temp.cols; i++)
// 			{
// 				fin.getline(value, 100, ' ');
// 				double(fin);
// 				std::cout << fin << std::endl;
// 				temp.at<double>(j,i) = fin;
// 			}
// 		}
// 		m.emplace_back(temp);
// 		if(fin.eof()) break;
		
// 	}
// }
