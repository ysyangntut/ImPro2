#include <climits> // for numeric_limits<unsigned int>::max()
#include <string>
#include <iostream>
#include <cstdio>  // for remove
#include <cstdlib> // for FILE

#include "IoData.h"

using namespace std; 

IoData::IoData()
{
}

bool fileExist(const char * name) {
    int err;
	FILE * file;
	err = fopen_s(&file, name, "r");
	if (err == 0) {
		fclose(file);
		return true;
	}
	else {
		return false;
	}
}

vector<unsigned char> IoData::serialize()
{
// write this object to xml file, read it as binary array
	vector<unsigned char> barray; 
	// find an available file name
	unsigned int i; 
	char fname[99]; 
    for (i = 0; i < 99999999; i++) {
        snprintf(fname, 99, "%d.xml", i);
		if (fileExist(fname) == false) break;
	}
	// write to xml file 
	int ok = this->writeToXml(string(fname)); 
	if (ok != 0) return barray; 
	// Reading size of file
	FILE * file;
    int err;
	long int fileSize; 
	err = fopen_s(&file, fname, "r+");
	if (err == 0) {
		fseek(file, 0, SEEK_END);
		fileSize = ftell(file);
		fclose(file);
	}
	else
	{
		return barray; 
	}
	// Reading data to array of unsigned chars
	err = fopen_s(&file, fname, "r+");
	if (err == 0) {
		barray.resize(fileSize);
		size_t bytes_read = fread((void*) barray.data(), sizeof(unsigned char), fileSize, file);
		fclose(file);
		// remove the file
		remove(fname);
	}
	return barray;
}

int IoData::deserialize(const vector<unsigned char>& barray)
{
// assuming the array is a binary form of an xml file (see ::serialize())
// write the array to a binary file (.xml) file and read it

	// find an available file name
	unsigned int i;
	char fname[99];
    for (i = 0; i < 99999999; i++) {
        snprintf(fname, 99, "%d.xml", i);
		if (fileExist(fname) == false) break;
	}
	// write array to binary file (actually an xml file)
	int ret; 
	FILE * file;
    int err;
	err = fopen_s(&file, fname, "w+");
	if (err == 0) {
		size_t bytes_written = fwrite((void*) barray.data(), sizeof(unsigned char), barray.size(), file);
		fclose(file);
		// read data from the xml file
		ret = this->readFromXml(string(fname));
	}
	else {
		ret = -1; 
	}
	return ret;
}

void IoData::memoryReallocationClick()
{
	this->memoryReallocationCount++;
	if (memoryReallocationCount % 10 == 9)
		cerr << "IoData warning: You are re-allocating IoData memory too many times. Try to modify your code or inform the authors.\n";
}

