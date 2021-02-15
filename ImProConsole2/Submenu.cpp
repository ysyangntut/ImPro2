#include "Submenu.h"

#include <opencv2/opencv.hpp>

int FuncRet(int argc, char ** argv)
{
	return -1001;
}

Submenu::Submenu(int _argc, char ** _argv, string _preStr)
{
	// initial arguments
	this->argc = _argc; 
	this->argv = _argv;
	// prefix string
	this->preStr = _preStr;
	// add default menu item(s)
	this->addItem("ret", "Quit this menu", FuncRet);
}

int Submenu::addItem(SubmenuItem item)
{
	this->items.push_back(item);
	return 0;
}

int Submenu::addItem(string cmd, string shortStr, int(*f)(int, char **))
{
	this->addItem(SubmenuItem(cmd, shortStr, f)); 
	return 0;
}

int Submenu::printItems()
{
	// Print menu 
	cout << this->preStr << "=====================\n";
	cout << this->preStr << "Command      Description\n";
	cout << this->preStr << ".....................\n";
	for (int i = 0; i < this->items.size(); i++) {
		printf("%12s %s\n", items[i].cmd.c_str(), items[i].description.c_str());
	}
	cout << this->preStr << ".....................\n";
	return 0;
}

int Submenu::run()
{
	int countOfConsecutiveInvalidInput = 0, maxCountOfConsecutiveInvalidInput = 3;
	string strbuf;
	const cv::String parserKeys =
		"{help h usage ? |      | print this message   }"
		"{func           |      | ImPro Console function name}"
		;
	cv::CommandLineParser parser(this->argc, this->argv, parserKeys); 

	while (true) 
	{
		// Print menu 
		this->printItems();

		// Read a string from keyboard (cin). If it starts from '#', read again. 

		if (parser.has("func")) {
			strbuf = parser.get<cv::String>("func"); 
		}
		else {
			strbuf = readStringFromIstream(std::cin);
		}

		// Find a function to run
		int i;
		for (i = 0; i < this->items.size(); i++)
		{
			if (strbuf.compare(this->items[i].cmd) == 0)
			{
				int retVal;
				retVal = this->items[i].func(argc, argv);
				countOfConsecutiveInvalidInput = 0; // refresh count of consecutive invalid input
				// Check function return value
				if (retVal == -1001) return 0;
				break;
			}
		}
		if (i >= this->items.size()) 
			countOfConsecutiveInvalidInput++;
		if (countOfConsecutiveInvalidInput >= maxCountOfConsecutiveInvalidInput) {
			cerr << preStr << "Too many consecutive invalid input. Bye.\n";
			break;
		}
		// if func is assigned, it only runs once. 
		if (parser.has("func")) {
			break;
		}
	} // end of this menu main loop 

	return 0;
}

Submenu::~Submenu()
{
}
