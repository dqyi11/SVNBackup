/*
 * CParser.cpp
 *
 *  Created on: Jan 30, 2013
 *      Author: walter
 */

#include "CParser.h"
#include <iostream>
#include <fstream>


CParser::CParser(const char* fileName) {
	// TODO Auto-generated constructor stub
    ifstream in(fileName, ios::in);
    if(in.is_open()){
    	init(in);
    };
    in.close();
}

CParser::~CParser() {
	// TODO Auto-generated destructor stub
}

void CParser::init(istream& in)
{
	string line;
	while(!in.eof())
	{
		line.clear();
		getline(in, line);

		mLines.push_back(line);
	}

}

void CParser::doParsing()
{
	vector<string> elementSet;
	vector<string>::iterator it;
	for(it=mLines.begin();it!=mLines.end();it++)
	{
		elementSet.clear();
		elementSet = parseLine(*it);
		initData(elementSet);
	}
}

vector<string> CParser::parseLine(string & line)
{
	vector<string> elementSet;
	int strLen = line.length();

	//cout << "CParser::parseLine " << strLen << " " << line << endl;
	int subStart = 0, subEnd = 0;
	string element;

	if(0==strLen)
	{
		cout << "ERROR : strlen " << strLen << endl;
		return elementSet;
	}

	//cout << "S " << subStart << " E " << subEnd << " L " << strLen << endl;

	while(subStart<=strLen)
	{
		//cout << " IN WHILE " << endl;
		subEnd=line.find(",", subStart);

		if(subEnd==string::npos)
		{
			subEnd = strLen;
		}

		element.clear();
		element = line.substr(subStart, subEnd-subStart);

		elementSet.push_back(element);

		subStart = subEnd + 1;

		//cout << "A " << subStart << " B " << subEnd << " C " << strLen << endl;
	}

	return elementSet;
}

bool CParser::initData(vector<string> elementSet)
{
	cout << "CParser::initData " << endl;
}
