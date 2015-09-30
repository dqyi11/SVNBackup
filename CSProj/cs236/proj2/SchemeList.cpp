/*
 * SchemeList.cpp
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#include "SchemeList.h"
#include "Parser.h"
#include "Token.h"
#include <iostream>
#include <sstream>

using namespace std;

SchemeList::SchemeList() {
	// TODO Auto-generated constructor stub
	mSchemes.clear();
}

SchemeList::~SchemeList() {
	// TODO Auto-generated destructor stub
	mSchemes.clear();
}

std::string SchemeList::toString()
{
	std::string output = "";
	stringstream strStream;

	strStream.clear();
	strStream << "Schemes";
	strStream << "(";
	strStream << mSchemes.size();
	strStream << "):" << endl;
	strStream >> output;
	strStream.clear();

	output += "\n";

	vector<Scheme>::iterator it;
	for(it=mSchemes.begin();it!=mSchemes.end();it++)
	{
		output += "  ";
		output += (*it).toString();
	}

	return output;
}

bool SchemeList::parseTokens(Parser * parser)
{
	bool ret = false;

	if(NULL==parser)
	{
		return false;
	}

	Token * pToken = parser->getCurrentToken();

	while(ID==pToken->getTokenType())
	{
		Scheme scheme;
		ret = scheme.parseTokens(parser);

		if(false == ret)
		{
			return false;
		}

		mSchemes.push_back(scheme);

		pToken = parser->getCurrentToken();

		if(FACTS==pToken->getTokenType())
		{
			return true;
		}
	}

	return false;
}
