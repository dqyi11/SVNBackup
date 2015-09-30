/*
 * QueryList.cpp
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#include "QueryList.h"
#include <iostream>
#include <sstream>
#include "Parser.h"

using namespace std;

QueryList::QueryList() {
	// TODO Auto-generated constructor stub
	mQueries.clear();
}

QueryList::~QueryList() {
	// TODO Auto-generated destructor stub
	mQueries.clear();
}

std::string QueryList::toString()
{
	std::string output = "";
	stringstream strStream;

	strStream.clear();
	strStream << "Queries";
	strStream << "(";
	strStream << mQueries.size();
	strStream << "):" << endl;
	strStream >> output;
	strStream.clear();

	output += "\n";

	vector<Query>::iterator it;
	for(it=mQueries.begin();it!=mQueries.end();it++)
	{
		output += "  ";
		output += (*it).toString();
	}

	return output;
}

bool QueryList::parseTokens(Parser * parser)
{
	bool ret = false;

	if(NULL==parser)
	{
		return false;
	}

	Token * pToken = parser->getCurrentToken();

	while(ID==pToken->getTokenType())
	{
		Query query;
		ret = query.parseTokens(parser);

		if(false == ret)
		{
			return false;
		}

		mQueries.push_back(query);

		pToken = parser->getCurrentToken();

		if(EOF==pToken->getTokenType())
		{
			return true;
		}
	}

	return false;
}
