#pragma once

#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/opencv.hpp>
#include <filesystem>

using std::vector;
using std::string;

/*! \brief readMatFromCsvFile reads a 2D float array from a csv file and returns a cv::Mat.
/
//
*/
template <class T>
cv::Mat readMatFromCsvFile(std::string fname, char commentChar = '#')
{
	std::ifstream inputfile(fname);
	std::string current_line;
	// vector allows you to add data without knowing the exact size beforehand
	std::vector<std::vector<T> > all_data;
	// Start reading lines as long as there are lines in the file
	while (std::getline(inputfile, current_line)) {
		// Now inside each line we need to seperate the cols
		std::vector<T> values;
		std::stringstream temp(current_line);
		std::string single_value;
		while (std::getline(temp, single_value, ',')) {
			// remove space and TAB
			single_value.erase(std::remove_if(single_value.begin(), single_value.end(),
				[](char c) {
					return (c == ' ' || c == '\t');
				}),
				single_value.end());
			// if string starts from '#', skip entire line
			if (single_value[0] == commentChar) break;
			// convert the string element to a value
			T value;
			try {
				value = (T)std::stod(single_value);
				values.push_back(value);
			}
			catch (...) {
				// for exception, do nothing
				std::cerr << "# Exception: " << single_value << std::endl;
			}
			// if there is any '#' after the value, skip this line after adding this value
			if (single_value.find(commentChar) != std::string::npos) break;
		}
		// add the row to the complete data vector
		if (values.size() > 0)
			all_data.push_back(values);
	}
	// Now add all the data into a Mat element
	cv::Mat mat;
	if (all_data.size() >= 1 && all_data[0].size() >= 1) {
		if (std::is_same<T, unsigned char>::value)  mat = cv::Mat::zeros((int)all_data.size(), (int)all_data[0].size(), CV_8U);
		if (std::is_same<T, char>::value)           mat = cv::Mat::zeros((int)all_data.size(), (int)all_data[0].size(), CV_8S);
		if (std::is_same<T, unsigned short>::value) mat = cv::Mat::zeros((int)all_data.size(), (int)all_data[0].size(), CV_16U);
		if (std::is_same<T, short>::value)          mat = cv::Mat::zeros((int)all_data.size(), (int)all_data[0].size(), CV_16S);
		if (std::is_same<T, int>::value)            mat = cv::Mat::zeros((int)all_data.size(), (int)all_data[0].size(), CV_32S);
		if (std::is_same<T, float>::value)          mat = cv::Mat::zeros((int)all_data.size(), (int)all_data[0].size(), CV_32F);
		if (std::is_same<T, double>::value)         mat = cv::Mat::zeros((int)all_data.size(), (int)all_data[0].size(), CV_64F);
		// Loop over vectors and add the data
		for (int row = 0; row < (int)all_data.size(); row++) {
			for (int col = 0; col < (int)all_data[0].size(); col++) {
				if (row < all_data.size() && col < all_data[row].size())
					mat.at<T>(row, col) = all_data[row][col];
			}
		}
	}
	return mat;
}

int writeMatToCsvFile(const cv::Mat& m, std::string fname, std::string cfmt = ""); 

/*! \brief Returns all files in the specific path.
 *  This function dir() returns all files in the specific path
 *  This function uses filesystem::directory_iterator in C++ 17
 */
vector<string> dir(string path = "./");
vector<string> dirRegularFiles(string path = "./");
vector<std::filesystem::directory_entry> dirEntries(string path = "./");
vector<std::filesystem::directory_entry> dirRegularFileEntries(string path = "./");
