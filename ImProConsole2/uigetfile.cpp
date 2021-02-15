#include <iostream>
#include <string>
#include <vector>

using std::cout;
using std::cin;
using std::endl;
using std::vector;

#include "uigetfile.h"
#include "impro_util.h"


std::string uigetfile(void)
{
    std::string f;
    cout << "Sorry. uigetfile() is not supported here. Please enter full path of your file: \n";
    f = readStringLineFromIstream(std::cin);
    return f;
}

std::vector<std::string> uigetfiles(void)
{
    std::vector<std::string> fnames;
    std::string f;
    cout << "Sorry. uigetfiles() is not supported here. Please enter full path of your files one by one. Single charater \'/\' means the end:\n";
    while (true)
    {
        f = readStringLineFromIstream(std::cin);
        if (f.length() == 1 && f[0] == '/') break;
        fnames.push_back(f);
    }
    return fnames;

}

std::string uigetdir(void)
{
    std::string f;
    cout << "Sorry. uigetdir() is not supported here. Please enter full path of your directory: \n";
    f = readStringLineFromIstream(std::cin);
    f = appendSlashOrBackslashAfterDirectoryIfNecessary(f);
    return f;
}

std::string uiputfile(void)
{
    std::string f;
    cout << "Sorry. uiputfile() is not supported here. Please enter full path of your file: \n";
    f = readStringLineFromIstream(std::cin);
    return f;
}

