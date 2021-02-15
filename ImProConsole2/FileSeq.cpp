#define _CRT_SECURE_NO_WARNINGS
#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING 

#include "FileSeq.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <thread>
#include <experimental/filesystem>

#include "impro_util.h"

using namespace std;
namespace fs = std::experimental::filesystem; // In C++11 filesystem is under experimental

// functions for case insensitive comparison
inline bool caseInsCharCompareN(char a, char b) {
	return(toupper(a) == toupper(b));
}

bool caseInsCompare(const string& s1, const string& s2) {
	return((s1.size() == s2.size()) &&
		equal(s1.begin(), s1.end(), s2.begin(), caseInsCharCompareN));
}

FileSeq::FileSeq()
{
	this->theDir = ""; 
}

FileSeq::FileSeq(std::string _theDir)
{
	// set directory
	this->setDir(_theDir);
}

FileSeq::~FileSeq()
{
	// log
    this->log("# FileSeq stopped logging.");
}


// FileSeq::FileSeq(std::string _theDir, std::string filesFormat, int begin, int nFiles)
// {
// 	this->setDir(_theDir);
// 	this->setFilesByFormat(filesFormat, begin, nFiles);
// }

// FileSeq::FileSeq(std::string _theDir, std::string fileExt)
// {
// 	this->setDir(_theDir);
// 	this->setFilesByExt(fileExt);
// }

// FileSeq::FileSeq(std::string fileListFile)
// {
// 	this->setFileListByListFile(fileListFile);
// }


int FileSeq::setDir(std::string dir)
{
	// set log file name 
	// (as this function is supposed to be called in the very beginning)
	std::time_t t = std::time(0);
	std::tm* now = std::localtime(&t);
	char logFilenamec[100];
	snprintf(logFilenamec, 100, "log_FileSeq_%4d%02d%02d%02d%02d%02d.log",
		now->tm_year + 1900, now->tm_mon + 1, now->tm_mday,
		now->tm_hour, now->tm_min, now->tm_sec);
	this->logFilename = std::string(logFilenamec);

    // set this->theDir
    this->theDir = dir;

    // if the dir is empty, return. No need to add a slash.
    if (dir.length() <= 0) return 0;

	// Confirm theDir ends with '/' or '\'. If not, add one.
	if (dir[dir.length() - 1] != '/' && dir[dir.length() - 1] != '\\') {
		// counts '/' and '\\' in the string
		int count_slash = 0, count_bslsh = 0;
		std::string slash = std::string("/");
		std::string bslsh = std::string("\\");
		std::string::size_type pos = 0;
		while ((pos = dir.find(slash, pos)) != std::string::npos) {
			count_slash++;
			pos += slash.length();
		}
		pos = 0;
		while ((pos = dir.find(bslsh, pos)) != std::string::npos) {
			count_bslsh++;
			pos += bslsh.length();
		}
		// add slash or back slash
		if (count_bslsh > count_slash)
			this->theDir += "\\";
		else
			this->theDir += "/";
	}
    this->log("# Set directory to " + this->theDir);

	return 0;
}

int FileSeq::setFilesByFormat(std::string dir, std::string format, int begin, int num_files)
{
	this->setDir(dir);
	return this->setFilesByFormat(format, begin, num_files); 
}
int FileSeq::setFilesByFormat(const std::string format, int begin, int num_files)
{
	char cstr[10000]; // maximum length of each file name string
	this->filenames.clear();
	for (int i = 0; i < num_files; i++) {
		snprintf(cstr, 10000, format.c_str(), i + begin);
		filenames.push_back(std::string(cstr));
	}
    this->log("# Set " + to_string(this->num_files()) + " files by format ");
	this->log("  starting from " + this->filenames[0] + 
		" to " + this->filenames[this->num_files() - 1]);
	return 0;
}

int FileSeq::setFilesByExt(std::string dir, ::string fileExt)
{
	this->setDir(dir); 
	return this->setFilesByExt(fileExt); 
}

int FileSeq::setFilesByExt(std::string fileExt)
{
	int fileExtLen = (int) fileExt.length();

	// clear files
	this->filenames.clear(); 

	// add files into filenames
	for (auto & p : fs::directory_iterator(this->theDir)) {
		string strp;
		strp = p.path().string();
//		cout << strp.find_last_of('\\') << "  ";
//		cout << strp.substr(strp.find_last_of('\\')+1 ) << "...";
//		cout << strp.substr(strp.length() - fileExtLen) << "...";

		if (caseInsCompare(strp.substr(strp.length() - fileExtLen), fileExt))
			this->filenames.push_back(strp.substr(strp.find_last_of('\\') + 1));
//		if (strp.substr(strp.length() - fileExtLen).compare(fileExt.c_str()) == 0)
//			fnames.push_back(strp.substr(strp.find_last_of('\\') + 1));
	}
    this->log("# Set " + to_string(this->num_files()) + " files by file extension "
		+ fileExt);
	if (this->num_files() > 0) // added by vince on 26-Feb-2019
		this->log("  starting from " + this->filenames[0] + 
			" to " + this->filenames[this->num_files() - 1]);
	return 0;
}			
			
int FileSeq::setFilesByListFile(std::string fileListFile)
{
	ifstream flist;
	flist.open(fileListFile);
	if (flist.is_open() == false) {
        this->log("# Error in setFileListByListFile(): Cannot open file-list file: "
			+ fileListFile); 
		return -1;
	}
	// path
	this->setDir(directoryOfFullPathFile(fileListFile)); 
//	int path_seperate = (int) fileListFile.find_last_of('/');
//	if (path_seperate == fileListFile.npos) 
//		path_seperate = (int) fileListFile.find_last_of('\\');
//	if (path_seperate == fileListFile.npos) {
//		this->theDir = "./";
//	}
//	else {
//		this->theDir = fileListFile.substr(0, path_seperate + 1);
//	}
	// read file names
	this->filenames.clear(); 
	while (flist.eof() == false)
	{
		string fname;
//		flist >> fname;
		fname = readStringLineFromIstream(flist); 
		if (fname.empty() == false) {
			this->filenames.push_back(fname);
//			cout << "-->" << fname << "<--\n";
		}
	}
	flist.close();
//	log
    this->log("# Set " + to_string(this->num_files()) + " files by file list "
		+ fileListFile);
	this->log("  starting from " + this->filenames[0] +
		" to " + this->filenames[this->num_files() - 1]);
	return 0;
}


		
int FileSeq::setFilesByStringVec(const std::vector<std::string> & fVec)
{
	// Check
	if (fVec.size() <= 1)
		return -1;
	// set directory
	this->filenames.clear();
	this->setDir(fVec[0]);
	for (int i = 1; i < (int) fVec.size(); i++) 
	{
		if (fVec[i].empty() == false)
			this->filenames.push_back(fVec[i]);
	}
	//	log
    this->log("# Set " + to_string(this->num_files()) + " files by vec of strings ");
	this->log("  Directory: " + this->directory() + "\n"); 
	this->log("  starting from " + this->filenames[0] +
		" to " + this->filenames[this->num_files() - 1]);
	return 0;
}

//! Sets directory and files by console interaction (cout and cin)
/*!
\details
Asks user to choose one of the following ways to define files list.
'm': Input directory, then files one by one.
'c': Input directory, c-style format of file names (containing a %d), starting index, and number of files.
'f': Input file of files list.
'g': Select files by gui file dialog.
\return 0 for success, otherwise if something wrong.
*/
int FileSeq::setDirFilesByConsole(std::istream & ifile) {
	while (true) {
        cout << "# Choose one of the following ways to define files list:\n";
        cout << "#   'o': Input directory, then files one by one.\n";
        cout << "#   'c': Input directory, c-style format of file names(containing a %d), starting index, and number of files.\n";
        cout << "#   'f': Input file of files list.\n";
        cout << "#   'g': Graphical.Select file of files list by gui file dialog.\n";
        cout << "#   'gm': Graphical multi - selection.Select files(multi-selection) by gui file dialog.\n";
		string strbuf; strbuf = readStringFromIstream(ifile);
		if (strbuf.length() == 1 && strbuf[0] == 'o') {
			this->setDirFilesByConsoleOneByOne(ifile);
		}
		else if (strbuf.length() == 1 && strbuf[0] == 'c') {
			this->setDirFilesByConsoleCStyle(ifile);
		}
		else if (strbuf.length() == 1 && strbuf[0] == 'f') {
			this->setDirFilesByConsoleFilesList(ifile);
		}
		else if (strbuf.length() == 1 && strbuf[0] == 'g') {
			this->setDirFilesByConsoleGraphical(ifile);
		}
		else if (strbuf.length() == 2 && strbuf[0] == 'g' && strbuf[1] == 'm') {
			this->setDirFilesByConsoleGraphicalMultiFiles(ifile);
		}
		else {
			continue;
		}
		break;
	}
	return 0; 
}

int FileSeq::setDirFilesByConsoleOneByOne(std::istream & ifile)
{
    cout << "#  Enter directory: ";
	this->setDir(readStringFromIstream(ifile));
    cout << "#  Enter number of files: ";
	int nFiles = readIntFromIstream(ifile);
	this->filenames.clear();	// clear files
	for (int i = 0; i < nFiles; i++)
	{
		this->filenames.push_back(readStringFromIstream(ifile));
	}
    printf("#  %d files are set, from %s to %s under %s\n", (int)this->filenames.size(),
		this->filename(0).c_str(), this->filename((int) this->filenames.size() - 1).c_str(), this->directory().c_str());
	return 0;
}
int FileSeq::setDirFilesByConsoleCStyle(std::istream & ifile)
{
    cout << "#  Enter directory: ";
	this->setDir(readStringFromIstream(ifile));
    cout << "#  Enter c-style file name (with a %d. space is allowed): ";
	string cformat = readStringLineFromIstream(ifile);
    cout << "#  Enter starting index for the %d: ";
	int startIdx = readIntFromIstream(ifile);
    cout << "#  Enter number of files: ";
	int nFiles = readIntFromIstream(ifile);
	this->filenames.clear();	// clear files
	this->setFilesByFormat(cformat, startIdx, nFiles); 
    printf("#  %d files are set, from %s to %s under %s\n", (int)this->filenames.size(),
		this->filename(0).c_str(), this->filename((int)this->filenames.size() - 1).c_str(), this->directory().c_str());
	return 0;
}
int FileSeq::setDirFilesByConsoleFilesList(std::istream & ifile)
{
    cout << "#  Enter full path of files list: ";
	string fullPathFilesList = readStringLineFromIstream(ifile);
	this->setFilesByListFile(fullPathFilesList); 
    printf("#  %d files are set, from %s to %s under %s\n", (int)this->filenames.size(),
		this->filename(0).c_str(), this->filename((int)this->filenames.size() - 1).c_str(), this->directory().c_str());
	return 0;
}
int FileSeq::setDirFilesByConsoleGraphical(std::istream & ifile)
{
    cout << "# Select list file by file dialog: \n";
	string fullPathFilesList = uigetfile(); 
	if (fullPathFilesList.length() <= 0) {
        cout << "# User cancelled.\n";
		return -1;
	}
	this->setFilesByListFile(fullPathFilesList);
    printf("#  %d files are set, from %s to %s under %s\n", (int)this->filenames.size(),
		this->filename(0).c_str(), this->filename((int)this->filenames.size() - 1).c_str(), this->directory().c_str());
	return 0;
}
int FileSeq::setDirFilesByConsoleGraphicalMultiFiles(std::istream & ifile)
{
    cout << "# Select files by file dialog (Not suggested as files could be re-ordered): \n";
	vector<string> files = uigetfiles(); 
	if (files.size() <= 0) {
        cout << "# User cancelled.\n";
		return -1;
	}
	this->setFilesByStringVec(files);
    printf("#   %d files are set, from %s to %s under %s\n", (int) this->filenames.size(),
		this->filename(0).c_str(), this->filename((int)this->filenames.size() - 1).c_str(), this->directory().c_str());
	return 0;
}


int FileSeq::canRead(int idx) const
{

//	std::ifstream file(this->theDir + this->filenames[idx]);
    std::ifstream file(this->fullPathOfFile(idx));
	if (file.is_open() == true)
	{
		file.close();
		return 1;
	}
	return 0;
}

int FileSeq::num_files() const
{
	return (int) this->filenames.size();
}

int FileSeq::findIndexOfFile(std::string filename) const
{
	int nfiles = this->num_files();
	for (int i = 0; i < nfiles; i++)
	{
		if (filename.compare(this->filenames[i]) == 0)
			return i; 
	}
	return -1;
}

std::string FileSeq::filename(int idx) const
{
	if (idx >= 0 && idx < (int) this->filenames.size())
		return this->filenames[idx];
    this->log("# Error: from FileSeq::filename: Wrong idx: " + to_string(idx));
	std::cerr << "Error from FileSeq::filename: "
            << "# Wrong idx : " << idx << "\n";
	return std::string("");
}

std::string FileSeq::directory() const
{
	return this->theDir;
}

std::string FileSeq::fullPathOfFile(int idx) const
{
    // check idx
    if (idx < 0 || idx >= this->num_files()) {
        std::cerr << "# Warning from FileSeq: trying to access a file out of index range.\n";
        std::cerr << "#   Number of files: " << this->num_files() << "\n";
        std::cerr << "#   Trying to access file index " << idx << "\n";
        return std::string("");
    }

    // check string length of file name
    std::string _fname = this->filename(idx);
    if (_fname.length() <= 0) {
        std::cerr << "# Warning from FileSeq: accessing an empty file name.\n";
        std::cerr << "#   Trying to access file index " << idx << "\n";
        return std::string("");
    }

    // check if filename itself is a full path
    bool filenameIsFullPath = false;
    if (_fname[0] == '/' ||
        _fname[0] == '\\' ||
        (_fname.length() > 1 && _fname[1] == ':')  // Even if _fname is like "c:test.jpg", which is relative, we do not add directory() prior to this file name.
       ) {
        filenameIsFullPath = true;
    }

    // if it is full path, we do not add director() prior to the file name
    if (filenameIsFullPath == true) {
        return this->filename(idx);
    }

	return this->directory() + this->filename(idx);
}

int FileSeq::countCanRead() const
{
	int count = 0, nfiles = this->num_files();
	for (int i = 0; i < nfiles; i++)
	{
		if (this->canRead(i))
			count++;
	}
	return count;
}

int FileSeq::waitForFile(int idx, int waitTimeEach, int maxTotalWait) 
{
	// Check arguments
	if (idx >= this->num_files())
	{
        this->log("# Error: from FileSeq::waitForFile: Wrong idx: " + to_string(idx));
		std::cerr << "Error from FileSeq::waitForFile: " 
            << "# Wrong idx: " << idx << ". \n";
		return -1;
	}
	if (waitTimeEach < 1)
	{
        this->log("# Warning from FileSeq::waitForFile: waitTimeEach of a non-positive "
			+ std::string("value means waiting forever.")); 
        std::cerr << "# Warning from FileSeq::waitForFile: "
			<< "waitTimeEach of a non-positive value means waiting forever.\n";
	}

	// 
	int waitCount = 0; 
	int checkLaterFileExisting = 1;
	int checkEndOfFileSeq = 1;
	while (true)
	{
		// try to open the file
		if (this->canRead(idx)) {
			break;
		} else 
		{
			// check if waited too long
			if (maxTotalWait >= 0 && waitCount * waitTimeEach >= maxTotalWait)
			{ // give up waiting
                this->log("# Warning from FileSeq::waitForFile: Gave up waiting file index " +
					to_string(idx) + ": " + this->filename(idx)); 
                std::cerr << "# Warning from FileSeq::waitForFile: "
                    << "# Gave up waiting file index " << idx << ".\n";
				// 1. refresh number of files
				this->filenames.resize((size_t) idx); 
				// 2. generate file list file
				this->generateFileList("fileList.txt");
				return -1;
			}
			// check if eofs file exists (signal of end of files)
			if (checkEndOfFileSeq == 1 && this->checkEndOfFiles()) {
                this->log("# Signal of end-of-files (eofs) exists " +
					std::string("while waiting for file idx ") + to_string(idx) + 
					+ ": " + this->filename(idx) 
					+ ". File list refreshes and written to fileList.txt");
				// 1. refresh number of files
				this->filenames.resize((size_t) idx); 
				// 2. generate file list file
				this->generateFileList("fileList.txt");
				// 3. return the value indicating eofs
				return -2;
			}

			// check if this file is the findLastCanReadFile()
			if (checkLaterFileExisting) {
				int lastCanReadFile = this->findLastCanReadFile();
				if (idx < lastCanReadFile)
				{
                    this->log("# Warning from FileSeq::waitForFile: "
						+ std::string("File ") + to_string(lastCanReadFile) +
						" is found while you are still waiting for file "
						+ to_string(idx) + ": " + this->filename(idx));
                    std::cerr << "# Warning from FileSeq::waitForFile: "
						<< "File " << lastCanReadFile << " is found while you are "
						<< "still waiting for file " << idx << endl;
					// Only warn once. If warned, it does not check anymore.
					checkLaterFileExisting = 0; 
				}
			}
			// wait 
			std::this_thread::sleep_for(std::chrono::milliseconds(waitTimeEach));
			waitCount++;
		}
	}
	return 0;
}

int FileSeq::waitForImageFile(int idx, cv::Mat & img, int imread_flag, 
	int waitTimeEach, int maxTotalWait)
{
	// Check arguments
	if (idx >= this->num_files())
	{
        this->log("# Error: from FileSeq::waitForImageFile: Wrong idx: " + to_string(idx));
        std::cerr << "# Error from FileSeq::waitForImageFile: "
			<< "Wrong idx: " << idx << ". \n";
		return -1;
	}
	if (waitTimeEach < 1)
	{
        this->log("# Warning from FileSeq::waitForImageFile: waitTimeEach of a non-positive "
			+ std::string("value means waiting forever."));
        std::cerr << "# Warning from FileSeq::waitForImageFile: "
			<< "waitTimeEach of a non-positive value means waiting forever.\n";
	}

	// 
	int waitCount = 0;
	int checkLaterFileExisting = 1;
	int checkEndOfFileSeq = 1;
	while (true)
	{
		// try to open the file
		if (this->canRead(idx)) {
			img = cv::imread(this->fullPathOfFile(idx), imread_flag); 
			if (img.cols > 0 && img.rows > 0)
				break;
		}
		else
		{
			// check if waited too long
			if (maxTotalWait >= 0 && waitCount * waitTimeEach >= maxTotalWait)
			{ // give up waiting
                this->log("# Warning from FileSeq::waitForFile: Gave up waiting file index " +
					to_string(idx) + ": " + this->filename(idx));
                std::cerr << "# Warning from FileSeq::waitForFile: "
                    << "# Gave up waiting file index " << idx << ".\n";
				// 1. refresh number of files
				this->filenames.resize((size_t)idx);
				// 2. generate file list file
				this->generateFileList("fileList.txt");
				return -1;
			}
			// check if eofs file exists (signal of end of files)
			if (checkEndOfFileSeq == 1 && this->checkEndOfFiles()) {
                this->log("# Signal of end-of-files (eofs) exists " +
					std::string("while waiting for file idx ") + to_string(idx) +
					+": " + this->filename(idx)
					+ ". File list refreshes and written to fileList.txt");
				// 1. refresh number of files
				this->filenames.resize((size_t)idx);
				// 2. generate file list file
				this->generateFileList("fileList.txt");
				// 3. return the value indicating eofs
				return -2;
			}

			// check if this file is the findLastCanReadFile()
			if (checkLaterFileExisting) {
				int lastCanReadFile = this->findLastCanReadFile();
				if (idx < lastCanReadFile)
				{
                    this->log("# Warning from FileSeq::waitForFile: "
						+ std::string("File ") + to_string(lastCanReadFile) +
						" is found while you are still waiting for file "
						+ to_string(idx) + ": " + this->filename(idx));
                    std::cerr << "# Warning from FileSeq::waitForFile: "
                        << "# File " << lastCanReadFile << " is found while you are "
						<< "still waiting for file " << idx << endl;
					// Only warn once. If warned, it does not check anymore.
					checkLaterFileExisting = 0;
				}
			}
			// wait 
			std::this_thread::sleep_for(std::chrono::milliseconds(waitTimeEach));
			waitCount++;
		}
	}
	return 0;
}


int FileSeq::findLastCanReadFile() const
{
	for (int i = this->num_files() - 1; i >= 0; i--)
	{
		if (this->canRead(i))
			return i;
	}	
	return -1;
}

int FileSeq::checkEndOfFiles(std::string eofs) const
{
	ifstream if_eofs(this->theDir + eofs);
	if (if_eofs.is_open()) {
		if_eofs.close();
		return 1;
	}
	return 0;
}

int FileSeq::generateFileList(std::string fileListFile) 
{
	std::ofstream of(this->theDir + fileListFile);
	for (int i = 0; i < this->num_files(); i++)
	{
		of << this->filenames[i] << endl;
	}
	this->log("Generated a file-list file: " + fileListFile); 
	of.close(); 
	return 0;
}

//! Returns vector of full-path file names
//!
std::vector<std::string> FileSeq::allFullPathFileNames() const
{
    int n = this->num_files();
    // if num_files is zero, returns an empty vector.
    if (n <= 0) return std::vector<std::string>(0);
    std::vector<std::string> vecFilenames(this->num_files());
    for (int i = 0; i < n; i++)
        vecFilenames[i] = this->fullPathOfFile(i);
    return vecFilenames;
}


int FileSeq::log(std::string msg) const 
{
	// Log file is located in the working directory.
	// If the working directory (theDir) is not defined yet, logging will not 
	// be carried out. 
	if (this->theDir.length() <= 1) return -1; 
	ofstream logFile(this->theDir + this->logFilename, std::ios_base::app); 
	if (logFile.is_open()) {
		std::time_t t = std::time(0);
		std::tm* now = std::localtime(&t);
		char logTime[100];
		snprintf(logTime, 100, " (%4d-%02d-%02d %02d:%02d:%02d)",
			now->tm_year + 1900, now->tm_mon + 1, now->tm_mday,
			now->tm_hour, now->tm_min, now->tm_sec);
		logFile << msg << logTime << endl;
		logFile.close(); 
	}
	return 0;
}
