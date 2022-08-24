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
