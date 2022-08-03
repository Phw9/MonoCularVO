#pragma once

#include <deque>
#include <fstream>
#include <string>
#include "opencv2/core.hpp"

//Intrinsic Matrix K

void FileRead(std::deque<std::string>& v, std::ifstream &fin);
void MakeTextFile(std::ofstream& fout, const int& imageNum);
