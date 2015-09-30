/*
 * RuleList.cpp
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#include "RuleList.h"
#include "Parser.h"
#include "Token.h"
#include <iostream>
#include <sstream>

using namespace std;

RuleList::RuleList() {
	// TODO Auto-generated constructor stub
	mRules.clear();
}

RuleList::~RuleList() {
	// TODO Auto-generated destructor stub
	mRules.clear();
}

std::string RuleList::toString()
{
	std::string output = "";
	stringstream strStream;

	strStream.clear();
	strStream << "Rules";
	strStream << "(";
	strStream << mRules.size();
	strStream << "):" << endl;
	strStream >> output;
	strStream.clear();

	output += "\n";

	vector<Rule>::iterator it;
	for(it=mRules.begin();it!=mRules.end();it++)
	{
		output += "  ";
		output += (*it).toString();
	}

	return output;
}

bool RuleList::parseTokens(Parser * parser)
{
	bool ret = false;

	if(NULL==parser)
	{
		return false;
	}

	Token * pToken = parser->getCurrentToken();

	while(ID==pToken->getTokenType())
	{
		Rule rule;
		ret = rule.parseTokens(parser);

		if(false == ret)
		{
			return false;
		}

		mRules.push_back(rule);

		pToken = parser->getCurrentToken();

	}

	// QUERYLIST allows to be empty
	if(QUERIES==pToken->getTokenType())
	{
		return true;
	}

	return false;
}
