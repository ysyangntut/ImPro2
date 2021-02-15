#include "improStrings.h"
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>

// This function returns A.compare(B), while A is uppercase of a, and
// B is uppercase of b.
int __strcmpi(std::string a, std::string b) {
    std::transform(a.begin(), a.end(), a.begin(), std::toupper);
    std::transform(b.begin(), b.end(), b.begin(), std::toupper);
    return a.compare(b);
}

// This function reads floats from a string.
// Case insensitive "na", "#n/a", "nan", "#nan" converts to nanf("")
// Other string that cannot be parsed will be ignored.
// For example: input text as "0 whatever 1 2 na \n 3 whatever 4 5 whatever whatever", output vecFloat will be
// {0, 1, 2, nanf(""), 3, 4, 5}
void fromStringToVecFloatAcceptNaIgnoreOthers(
        std::string text,
        std::vector<float> & vecFloats,
        int & nFloats, int & nNanfs)
{
    // get text (entire content to a single string)
    std::stringstream ss;
    ss << text;
    vecFloats.clear();
    nFloats = 0;
    nNanfs = 0;
    while (true) {
        float tmp;
        std::string stmp;
        // check EOF of the stream
        if (ss.eof() == true) break;
        // read a string
        ss >> stmp;
        if (stmp.length() <= 0) continue;
        // convert to a float
        try {
            if(__strcmpi(stmp, std::string("NA")) == 0 ||
               __strcmpi(stmp, std::string("#n/a")) == 0 ||
               __strcmpi(stmp, std::string("nan")) == 0 ||
               __strcmpi(stmp, std::string("#nan")) == 0 ||
               __strcmpi(stmp, std::string("#value!")) == 0)
            {
                tmp = std::nanf("");
                nNanfs++;
            } else {
                tmp = std::stof(stmp);
                nFloats++;
            }
            // append to vector (including NA (nanf))
            vecFloats.push_back(tmp);
        }  catch (...) {
            // if neither NA nor a float, ignore it.
            // do nothing
        }
    }
}

// reads double from a string. Case insensitive "na" and "#n/a" converts to nan("")
// Other string that cannot be parsed will be ignored.
// For example: input text as "0 whatever 1 2 na \n 3 whatever 4 5 whatever whatever", output vecDouble will be
// {0, 1, 2, nanf(""), 3, 4, 5}
void fromStringToVecDoubleAcceptNaIgnoreOthers(
        std::string text,
        std::vector<double> & vecDoubles,
        int & nDoubles, int & nNans)
{
    // get text (entire content to a single string)
    std::stringstream ss;
    ss << text;
    vecDoubles.clear();
    nDoubles = 0;
    nNans = 0;
    while (true) {
        double tmp;
        std::string stmp;
        // check EOF of the stream
        if (ss.eof() == true) break;
        // read a string
        ss >> stmp;
        if (stmp.length() <= 0) continue;
        // convert to a double
        try {
            if(__strcmpi(stmp, std::string("NA")) == 0 ||
               __strcmpi(stmp, std::string("#n/a")) == 0 ||
               __strcmpi(stmp, std::string("nan")) == 0 ||
               __strcmpi(stmp, std::string("#nan")) == 0 ||
               __strcmpi(stmp, std::string("#value!")) == 0)
            {
                tmp = std::nan("");
                nNans++;
            } else {
                tmp = std::stod(stmp);
                nDoubles++;
            }
            // append to vector (including NA (nanf))
            vecDoubles.push_back(tmp);
        }  catch (...) {
            // if neither NA nor a double, ignore it.
            // do nothing
        }
    }
}


// This function reads array of arrays of double from a string.
// Case insensitive "na", "#n/a" "nan", "#nan", "#value!" converts to nan("")
// Other string that cannot be parsed will be ignored.
// Arrays are splitted with delimitation.
// For example: input text is "1 2 whatever 3 # 3 4 # na 4 5 # whatever # 6" and the delimitation is '#',
// the output vecVecFloats will be
// vector<vector<float> > { vector<float>{1,2,3}, vector<float>{3,4}, vector<float>{nanf(""), 4, 5},
//                          vector<float>{6} }
// nVecFloats will be vector<int>{3, 2, 2, 1},
// nVecNanfs will be vector<int>{0, 0, 1, 0}
// nFloats will be 8, and nNanfs will be 1.
void fromStringToVecVecFloatAcceptNaIgnoreOthers(
        std::string text,
        char demin,
        std::vector<std::vector<float> > & vecVecFloats,
        std::vector<int> & nVecFloats,
        std::vector<int> & nVecNanfs,
        int &nFloats, int &nNanfs)
{
    nFloats = nNanfs = 0;
    vecVecFloats.clear();
    nVecFloats.clear();
    nVecNanfs.clear();
    // convert entire string into strings
    std::stringstream streamFullText(text);
    std::vector<std::string> strings;
    if (text.length() > 0) {
        std::string thisLine;
        while(std::getline(streamFullText, thisLine, demin))
        {
            int nFloatsThisLine = 0, nNanfThisLine = 0;
            std::vector<float> vecFloats;
            fromStringToVecFloatAcceptNaIgnoreOthers(thisLine, vecFloats, nFloatsThisLine, nNanfThisLine);
            if (vecFloats.size() >= 1) {
                vecVecFloats.push_back(vecFloats);
                nVecFloats.push_back(nFloatsThisLine);
                nVecNanfs.push_back(nNanfThisLine);
                nFloats += nFloatsThisLine;
                nNanfs += nNanfThisLine;
            }
        }
    }
}

std::vector<std::string> splitPath(
  const std::string& str
  , const std::set<char> delimiters)
{
  std::vector<std::string> result;

  char const* pch = str.c_str();
  char const* start = pch;
  for(; *pch; ++pch)
  {
    if (delimiters.find(*pch) != delimiters.end())
    {
      if (start != pch)
      {
        std::string str(start, pch);
        result.push_back(str);
      }
      else
      {
        result.push_back("");
      }
      start = pch + 1;
    }
  }
  result.push_back(start);

  return result;
}

