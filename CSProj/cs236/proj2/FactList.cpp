/*
 * FactList.cpp
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#include "FactList.h"
#include <iostream>
#include "Parser.h"
#include "sstream"

using namespace std;

FactList::FactList() {
	// TODO Auto-generated constructor stub
	mFacts.clear();
}

FactList::~FactList() {
	// TODO Auto-generated destructor stub
	mFacts.clear();
}

std::string FactList::toString()
{
	std::stringstream strStream;
	std::string output = "";

	strStream.clear();
	strStream << "Facts";
	strStream << "(";
	strStream << mFacts.size();
	strStream << "):" << endl;
	strStream >> output;
	strStream.clear();

	output += "\n";

	vector<Fact>::iterator it;
	for(it=mFacts.begin();it!=mFacts.end();it++)
	{
		output += "  ";
		output += (*it).toString();
	}

	return output;
}

bool FactList::parseTokens(Parser * parser)
{
	bool ret = false;
	if(NULL==parser)
	{
		return false;
	}

	Token * pToken = parser->getCurrentToken();

	while(ID==pToken->getTokenType())
	{
		Fact fact;
		ret = fact.parseTokens(parser);

		if(false == ret)
		{
			// cout << " FACT = false " << endl;
			return false;
		}

		mFacts.push_back(fact);

		pToken = parser->getCurrentToken();


	}

	if(RULES==pToken->getTokenType())
	{
		// return true means allow empty
		return true;
	}

	return true;
}
