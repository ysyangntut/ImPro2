#pragma once
#include <string>
#include <vector>
#include "impro_util.h"
using namespace std;
class IoData
{
public:
	IoData();
	virtual ~IoData() {};

	// to/from txt file
	virtual int readFromTxt(string fileTxt) = 0;
	virtual int writeToTxt(string fileTxt)  = 0;

	// to/from xml/yaml file by using OpenCV FileStorage 
	virtual int readFromXml(string fileXml) = 0;
	virtual int writeToXml(string fileXml)  = 0;

	// ask user which format and where to read, then read data
	virtual int readThruUserInteraction() = 0;
	virtual int writeThruUserInteraction() = 0;

	// export m-script (Matlab/Octave) to visualize data
	virtual int writeScriptMat(string fileM) = 0;

	// serialization and deserialization
	virtual vector<unsigned char> serialize();
	virtual int deserialize(const vector<unsigned char> & dat);

	// warnings
	virtual void memoryReallocationClick();

protected:
	int memoryReallocationCount = 0; 
};

