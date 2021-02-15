#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstring>
#include <cstdlib>

#include <opencv2/opencv.hpp> // for cv::imread

using namespace std;

//! FileSeq manages a sequence of files. 
/*!
  FileSeq manages a sequence of files. It is originally designed for 
  instant analysis where files could be generated (by other programs) 
  during the run-time of user's program. 
*/ 


class FileSeq
{
public:

	//! Constructs a FileSeq object.
	/*!
	*/
	FileSeq();

	//! Constructs a FileSeq object which files are supposed to be assigned later.
	/*!
	\details
	Usage example: 
		FileSeq fsq("c:/TestPhotos/");
	\param _theDir the directory of files. Must be given. Files must be 
	               in the same directory. 
				   The directory separator can be either '/' or '\\'.
				   If filePath does not end with '/' or '\\' then this class
				   will automatically add one.
	*/
	FileSeq(std::string _theDir); 

//	//! Constructs a FileSeq object with C-style format of file names. 
//	/*!
//	\details 
//	Usage example:
//		FileSeq fsq("c:/TestPhotos/DSC%05d.JPG", 1, 9999);
//	\param _theDir the directory of files.
//	\param filesFormat C-style format of file names (without directory)
//	\param begin the beginning number in the file names
//	\param num_files number of file names to be generated
//	\see setFilesFormat()
//	*/
//	FileSeq(std::string _theDir, std::string filesFormat, int begin, int num_files);

//	//! Constructs a FileSeq object by giving a path and a file extension.	
//	/*!
//	\details
//	Sets all files in the given path as the file sequence. User gives the 
//	file extension (E.g., ".JPG"). FileSeq adds all files with given 
//	file extension into the FileSeq. It is NOT designed to be used when 
//	some of the files are either being generated or not even generated 
//	(during an experiment). 
//	\param _theDir the directory of files.
//	\param fileExt File extension (e.g., ".JPG"). Only files with this
//	extension will be added.
//	*/
//	FileSeq(std::string _theDir, std::string fileExt);
	
//	//! Constructs a FileSeq object by giving a list of files.
//	/*!
//	User gives the full path of file-list file, and FileSeq sets all files
//	in the file into the FileSeq.
//	\param fileListFile full path of the file-list file. The file contains
//	the file names (without path). This function also sets the working 
//	directory (that files are located) to the directory of fileListFile.
//	*/
//	FileSeq(std::string fileListFile);
		
	~FileSeq();

	//! Sets the directory of the files. 
	/*!
	\details
	Usage example:
	FileSeq fsq; fsq.setDir("c:/TestPhotos/");
	\param theDir the specified directory of the files.
	If dir is empty (length zero), this function then prompts
	question to user and gets directory from GUI or text-based
	screen (if GUI is not available. Not implemented yet).
	\return 0: Successfully set to the specified existing directory.
	\see See also ... (not written yet)
	*/
	int setDir(std::string theDir);

	//! Sets names of the files according to given c-style wildcard format. 
	/*!
	\details Usage example:
	    FileSeq fsq; 
		fsq.setFilesFormat("DSC%05.JPG", 1, 9999); // DSC00001.JPG, DSC00002.JPG, ...
	\param format c-style format (mainly %d based) of the file names.
	\param begin the first number (for the first file)
	\param num_files number of files 
	\return 0: Successfully set.
	        1: Successfully set to the newly created specified directory
	\see setDir()
	*/
	int setFilesByFormat(std::string dir, std::string format, int begin, int num_files);
	int setFilesByFormat(std::string format, int begin, int num_files);

	//! Sets names of the files by giving a  a file extension.	
	/*!
	\details
	Sets all files in the given path as the file sequence. User gives the 
	file extension. FileSeq adds all files with given 
	file extension into the FileSeq. 
	\param theDir The path of files. The directory separator can be either
	'/' or '\\'. If filePath does not end with '/' or '\\' then this class 
	will automatically add one. 
	\param fileExt File extension (e.g., ".JPG"). Only files with this
	extension will be added.
	\return 0: Successfully set.
	*/
	int setFilesByExt(std::string dir, std::string fileExt);
	int setFilesByExt(std::string fileExt);
	
	//! Sets names of the files by giving a file-list file. 
	/*!
	\details
	User gives the full path of the file-list file, and FileSeq sets all files
	in the file into the FileSeq.
	\param fileListFile The full path of the file-list file. The file contains
	the file names (without path). The path of files is assumed the same path
	of the file-list file.
	\return 0: Successfully set.  -1: Failed.
	*/
	int setFilesByListFile(std::string fileListFile);

	//! Sets directory and files by giving vector<string>
	/*!
	\details 
	User gives a vector<string>, which [0] is the directory, followed by
	file names. 
	\param filenamesVec the vector of file names, where [0] is the directory.
	\return 0 for success, otherwise if something wrong.
	*/
	int setFilesByStringVec(const std::vector<std::string> & fVec); 

	//! Sets directory and files by console interaction (cout and cin)
	/*!
	\details 
	Asks user to choose one of the following ways to define files list.
	'o': One by one. Input directory, then files one by one.
	'c': C-style. Input directory, c-style format of file names (containing a %d), starting index, and number of files.
	'f': Files list. Input file of files list.
	'g': Graphical. Select file of files list by gui file dialog.
	'gm': Graphical multi-selection. Select files (multi-selection) by gui file dialog.
	\return 0 for success, otherwise if something wrong.
	*/
	int setDirFilesByConsole(std::istream & ifile = std::cin);
	int setDirFilesByConsoleOneByOne(std::istream & ifile = std::cin);
	int setDirFilesByConsoleCStyle(std::istream & ifile = std::cin);
	int setDirFilesByConsoleFilesList(std::istream & ifile = std::cin);
	int setDirFilesByConsoleGraphical(std::istream & ifile = std::cin);
	int setDirFilesByConsoleGraphicalMultiFiles(std::istream & ifile = std::cin);

	//! Checks if a file (of given index) exists and can be read or not.
	/*!
	\param idx index of the file. idx is 0 based, i.e., first file index is 0.
	\return 1: This file can be read. 0: Cannot be read. Probably the file does
	not exist, or the file is being written by other processes. 
	*/	
	int canRead(int idx) const;

	//! Returns number of files
	/*!
	\return number of files
	*/
	int num_files() const;

	//! Finds the index of a given file name
	/*!
	\param filename the name of the file. 
	\return idx the index of the given file name in the FileSeq.
	        Returns -1 if the filename is not in this object.
	*/
	int findIndexOfFile(std::string filename) const;

	//! Returns file name (without directory) of an index
	/*!
	\param idx index of file name, zero-based index 
	*/
	std::string filename(int idx) const; 

	//! Returns the directory of the files.
	/*!
	\return the directory of the files.
	*/
	std::string directory() const; 

	//! Returns the full path (directory + filename) of the files.
	/*!
	\param idx index of file name
	\return the full path of a file.
	*/
	std::string fullPathOfFile(int idx) const; 

	//! Counts existing files (which can be read) 
	/*! 
	\return number of files which exist and can be read
	*/
	int countCanRead() const;

	//! Waits until a file can be read.
	/*!
	\brief This function waits until the specified file (index idx) can be read.
	\details 
	This function waits until the specified file (index idx) can be read. However, if 
	the eofs file appears (see int checkEndOfFiles()), this function returns.
	This function is not const because if eofs (signal of end-of-files) appears, this 
	function cut off the filenames vectors. 
	\param idx index of the file
	\param waitTimeEach waiting time (in ms) between each check
	\param maxTotalWait maximum waiting time (in ms) before giving up. 
	mxs being < 0 indicates waiting without giving up. 
	\see FileSeq::checkEndOfFiles()
	\return 0: the file can be read. -1: time-out. -2: a signal of end of FileSeq is found. 
	The "end-of-FileSeq" signal indicates that no more files will be added. The number
	of files of the FileSeq needs to be refreshed. The signal is the existance of file
	"end_of_FileSeq" 
	*/
	int waitForFile(int idx, int waitTimeEach = 50, int maxTotalWait = 86400 * 1000);
	
	//! Waits until an image file can be read.
	/*!
	\brief This function waits until the specified image (index idx) can be read.
	\details
	By using opencv cv::imread(), 
	this function waits until the specified image (index idx) can be read. However, if
	the eofs file appears (see int checkEndOfFiles()), this function returns.
	This function is not const because if eofs (signal of end-of-files) appears, this
	function cut off the filenames vectors.
	\param idx index of the file
	\param img the image which is read from the specified image file
	\param imread_flag flag for cv::imread(), by default, cv::IMREAD_COLOR. 
	\param waitTimeEach waiting time (in ms) between each check
	\param maxTotalWait maximum waiting time (in ms) before giving up.
	mxs being < 0 indicates waiting without giving up.
	\see FileSeq::checkEndOfFiles()
	\return 0: the file can be read. -1: time-out. -2: a signal of end of FileSeq is found.
	The "end-of-FileSeq" signal indicates that no more files will be added. The number
	of files of the FileSeq needs to be refreshed. The signal is the existance of file
	"end_of_FileSeq"
	*/
	int waitForImageFile(int idx, cv::Mat & img, int imread_flag = cv::IMREAD_COLOR, 
		int waitTimeEach = 50, int maxTotalWait = 86400 * 1000);

	
	//! Returns the index of the last file.
	/*! For example, if there are files 0, 1, 2, 3, 9, the
	    findLastCanReadFile() returns 9. 
	\return the index of the last existing (can-read) file.
	        If no file can be read, it returns -1.
	*/
	int findLastCanReadFile() const;

	//! Checks if signal of end-of-files exists
	/*! 
	\return 0 if this file does not exist, meaning more file(s) can be generated.
	1 if file exists, meaning no new file will be generated. 
	*/
	int checkEndOfFiles(std::string eofs = std::string("eofs")) const; 

	//! Generates file-list file. 
	/*!
	\details 
	User gives the file name of the file-list file to be generated, this
	function generates the file-list file according to the filenames. 
	\param fileListFile the file name of the file-list file. 
	*/
	int generateFileList(std::string fileListFile);

    //! Returns vector of full-path file names
    //!
    std::vector<std::string> allFullPathFileNames() const;

	//! Writes string to log file
	/*!
	*/
	int log(std::string) const; 

    //! \brief FileSeq::write() function allows cv::FileStorage to output
    //! a FileSeq object.
    /*!
    //! For example:
    //! {
    //!     cv::FileStorage fs("aFile.xml", cv::FileStorage::WRITE);
    //!     FileSeq fsq;
    //!     fs << "MyFiles" << fsq;
    //!     fs.release()
    //! }
    */
    void write(cv::FileStorage& fs) const
    {
        fs << "{";
        fs << "FileSeq_theDir" << this->theDir;
        fs << "FileSeq_filenames" << filenames;
        fs << "FileSeq_logFilename" << logFilename;
        fs << "}";
    }
    void read(const cv::FileNode& node)
    {
        node["FileSeq_theDir"] >> this->theDir;
        node["FileSeq_filenames"] >> this->filenames;
        node["FileSeq_logFilename"] >> this->logFilename;
    }

protected:
	std::string theDir; 
	std::vector<std::string> filenames; 
	std::string logFilename; 
};

static void write(cv::FileStorage& fs, const std::string&, const FileSeq& fsq)
{
    fsq.write(fs);
}

static void read(const cv::FileNode& node, FileSeq& fsq, const FileSeq& default_value = FileSeq())
{
  if(node.empty())
    fsq = default_value;
  else
    fsq.read(node);
}

static std::ostream& operator<<(std::ostream& out, const FileSeq& fsq)
{
  int n = fsq.num_files();
  out << "{ ";
  out << "FileSeq: This FileSeq has " << n << " files.\n";
  out << "  FileSeq directory is at: " << fsq.directory() << "\n";
  for (int i = 0; i < n; i++)
      out << "   FileSeq[" << i << "]: " << fsq.filename(i) << "\n";
  out << "}\n";
  return out;
}

