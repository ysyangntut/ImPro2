#include <iostream>

#include <opencv2/opencv.hpp>

#include "impro_util.h"
#include "ImagePointsPicker.h"

using namespace std;
using namespace cv;

const String keys =
"{help h usage ? |      | print this message   }"
"{image img      |      | background image file }"
"{n npoint       |      | number of points }"
"{outTxtFile txt |      | output filename in txt (with .txt) } "
"{outXmlFile xml |      | output filename in xml (with .xml) } "
"{outMarkedFile marked |      | output filename of marked image (with .JPG) } "
;

int FuncTemplatesPicking(int argc, char** argv)
{
	// Variables declaration
	ImagePointsPicker pp;
	bool ui = false;
	int setImgOk = -1;
	int nPoint = 0;
	string outTxtFile;
	string outXmlFile;
	string outMarkedFile;

	cv::CommandLineParser * pparser = NULL;
	if (argc >= 1) pparser = new cv::CommandLineParser(argc, argv, keys);

	if (argc == 0) {
		ui = true;
	}
	else if (argc == 1 || (*pparser).has("help")) {
		(*pparser).printMessage();
		cout << "\nUsage: \n"
			<< " ImageTmpltsPicking -img=theImage.JPG -n=4 -xml=theImage_PickedTemplates.xml -marked=theImage_marked.JPG \n";
		cout << "Enter i for interaction. Otherwise to quit: ";
		string str_interaction;
		str_interaction = readStringLineFromCin(); 
		if (str_interaction[0] == 'i')
			ui = true;
		else
			return 0;
	}

	// Set background image
	if (argc >= 1 && (*pparser).has("image")) {
		setImgOk = pp.setBackgroundImageByFilename((*pparser).get<string>("image"));
	}
	else {
		string fname_img;
		cout << "# Enter background image file ('g' for ui file dialog): ";
		fname_img = readStringLineFromCin();
		if (fname_img.length() == 1 && fname_img[0] == 'g')
			fname_img = uigetfile();
		setImgOk = pp.setBackgroundImageByFilename(fname_img);
		//		setImgOk = pp.setBackgroundImageByFilename("..\\test_images\\CalibGrid01.JPG");
	}
	if (setImgOk != 0) {
		cout << "# Failed to set background image.\n";
		return -1;
	}

	// number of points to pick
	if (argc >= 1 && (*pparser).has("n")) {
		nPoint = (*pparser).get<int>("n");
	}
	else if (ui) {
		cout << "# Number of points to pick: ";
		nPoint = readIntFromCin();
		if (nPoint <= 0) {
			cout << "# Invalid number of points.\n";
			return -1;
		}
	}
	else {
		nPoint = 1;
	}
//	pp.pickPoints(nPoint);
	pp.pickTemplates(nPoint, 27 /*ESC*/, 13 /*SPACE*/, 32 /*ENTER*/); 

	// Write to files (cv::FileStorage format)
	if (argc >= 1 && (*pparser).has("outTxtFile"))
		outTxtFile = (*pparser).get<string>("outTxtFile");
	else if (ui) {
		cout << "# Enter output text file (.txt) ('g' for ui file dialog): " << endl;
		outTxtFile = readStringLineFromCin();
		if (outTxtFile.length() == 1 && outTxtFile[0] == 'g')
			outTxtFile = uiputfile();
	}
	else
		outTxtFile = string("");

	if (argc >= 1 && (*pparser).has("outXmlFile"))
		outXmlFile = (*pparser).get<string>("outXmlFile");
	else if (ui) {
		cout << "# Enter output XML file (.xml) ('g' for ui file dialog): " << endl;
		outXmlFile = readStringLineFromCin();
		if (outXmlFile.length() == 1 && outXmlFile[0] == 'g')
			outXmlFile = uiputfile();
	}
	else
		outXmlFile = string("");

	if (argc >= 1 && (*pparser).has("outMarkedFile"))
		outMarkedFile = (*pparser).get<string>("outMarkedFile");
	else if (ui) {
		cout << "# Enter output marked JPG file (.JPG) ('g' for ui file dialog): " << endl;
		outMarkedFile = readStringLineFromCin();
		if (outMarkedFile.length() == 1 && outMarkedFile[0] == 'g')
			outMarkedFile = uiputfile();
	}
	else
		outMarkedFile = string("");

	pp.writeTemplateMarkersToImageFile(outMarkedFile);
	pp.writeTemplatesToTxtFile(outTxtFile);
	pp.writeTemplatesToXmlFile(outXmlFile);
	//	cout << pp.pointsIn1xNCvMat() << endl;

	return 0;
}