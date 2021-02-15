#include <iostream>
#include <cstdio>
#include <fstream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "impro_util.h"

using namespace std;

int FuncDrawRotatingTemplate(int argc, char** argv)
{
	// Step 0: Declare variables
	cv::Mat imgTmplt, imgBackg; 
	string fnameTmplt, fnameBackg; 
	istream & _cin = std::cin; 

	// Step 1: Get template (and reference point)
	while (true) {
		printf("# Enter template file name (or \"corner\" for black-white corner:\n");
		fnameTmplt = readStringLineFromIstream(_cin);
		if (fnameTmplt.compare("corner") == 0) {
			printf("# Enter corner size (in pixels, twice of small square):\n");
			int cornerSize = readIntFromIstream(_cin);
			imgTmplt = cv::Mat::zeros(2, 2, CV_8UC3); 
			cv::resize(imgTmplt, imgTmplt, cv::Size(cornerSize, cornerSize)); 
			cv::imshow("Tmplt", imgTmplt);
			cv::waitKey(0);
		}
	}


	// Step 2: Get background 

	// Step 3: Get revolution center, radius (in pixels), and revolution speed (deg per frame)

	// Step 4: Get rotation or not 

	// Step 5: Get noise level 

	// Step 6: Get output file sequence 

	// Step 7: Start loop

	// Step 8:    Generate rotated template (if it rotates)

	// Step 9:    Replace template on background

	// Step 10:   Save image 

	

	return 0;
}
