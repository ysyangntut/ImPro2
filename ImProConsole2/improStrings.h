#ifndef IMPRO_STRINGS_H
#define IMPRO_STRINGS_H

#include <string>
#include <vector>
#include <set>

using std::string;
using std::vector;
using std::set;

// returns A.compare(B), while a is uppercase of A, and
// B is uppercase of B.
int __strcmpi(std::string a, std::string b) ;

// reads floats from a string. Case insensitive "na", "#n/a", "#value!" converts to nanf("")
// Other string that cannot be parsed will be ignored.
// For example: input text as "0 whatever 1 2 na \n 3 whatever 4 5 whatever whatever", output vecFloat will be
// {0, 1, 2, nanf(""), 3, 4, 5}
void fromStringToVecFloatAcceptNaIgnoreOthers(
        std::string text,
        std::vector<float> & vecFloats,
        int & nFloats, int & nNanfs);

// reads double from a string. Case insensitive "na", "#n/a", and "#value!" converts to nan("")
// Other string that cannot be parsed will be ignored.
// For example: input text is "0 whatever 1 2 na \n 3 whatever 4 5 whatever whatever", output vecDouble will be
// {0, 1, 2, nanf(""), 3, 4, 5}
void fromStringToVecDoubleAcceptNaIgnoreOthers(
        std::string text,
        std::vector<double> & vecDoubles,
        int & nDoubles, int & nNans);

// This function reads array of arrays of double from a string. Case insensitive "na", "#n/a", and "#value!"
// converts to nan("")
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
        int & nFloats, int & nNanfs);



/*! \brief Breaks a path name into components.
 *  This function splitPath() breaks a path name into components.
 *  Example 1:
 *    splitPath("c:\\aa\\b1 b2\\fname.txt", set<char>{'\\', '/'}),
 *    returns vector<string>{"c:", "aa", "b1 b2", "fname.txt"}.
 *  Example 2:
 *    splitPath("c:\\aa\\b1 b2\\fname.txt", set<char>{'\\', '/'}).back(),
 *    returns string("fname.txt").
 */
std::vector<std::string> splitPath(
  const std::string& str
  , const std::set<char> delimiters);

#endif // IMPRO_STRINGS_H
