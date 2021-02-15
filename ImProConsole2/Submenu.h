#pragma once

#include <iostream>
#include <cstdio>
#include <string>
#include <vector>

#include "impro_util.h"

using namespace std;

class SubmenuItem
{
public:
	SubmenuItem(string _cmd, string _description, int(*f)(int, char**))
	{
		this->cmd = _cmd; 
		this->description = _description;
		this->func = f; 
	}
	std::string cmd;
	std::string description;
	int(*func)(int argc, char** argv); 
};


class Submenu
{
public:
	Submenu(int argc, char ** argv, string _preStr = string(""));
	int addItem(SubmenuItem item); 
	int addItem(string cmd, string description, int(*f)(int, char**));
	int printItems();
	int run(); 
	~Submenu();
protected:
	vector<SubmenuItem> items; 
	int argc;
	char** argv;
	string preStr; 
};

