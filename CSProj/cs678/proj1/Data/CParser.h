/*
 * CParser.h
 *
 *  Created on: Jan 30, 2013
 *      Author: walter
 */

#ifndef CPARSER_H_
#define CPARSER_H_

#include <string>
#include <vector>

using namespace std;

class CParser {
public:
	CParser(const char* fileName);
	virtual ~CParser();

	void init(istream& in);

	vector<string> parseLine(string & line);

	virtual bool initData(vector<string> elementSet);

	void doParsing();

	vector<string> mLines;
};

#endif /* CPARSER_H_ */
