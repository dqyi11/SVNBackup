/*
 * Rule.cpp
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#include "Rule.h"
#include "Parser.h"
#include <iostream>

using namespace std;

Rule::Rule() {
	// TODO Auto-generated constructor stub
	mPredicates.clear();
}

Rule::~Rule() {
	// TODO Auto-generated destructor stub
	mPredicates.clear();
}

std::string Rule::toString()
{
	std::string output = "";
	output = mHead.toString();
	output += ":-";
	std::vector<Predicate>::iterator it;
	for(it=mPredicates.begin();it!=mPredicates.end();it++)
	{
		output += (*it).toString();
		if(it==mPredicates.end()-1)
		{
			output += ".";
		}
		else
		{
			output += ",";
		}
	}
	output += "\n";

	return output;

}

bool Rule::parseTokens(Parser * parser)
{
	bool ret = false;

	if(NULL==parser)
	{
		return false;
	}

	Token * pToken = parser->getCurrentToken();
	if(ID != pToken->getTokenType())
	{
		return false;
	}

	mHead.parseTokens(parser);

	//parser->goToNextToken();
	pToken = parser->getCurrentToken();

	if(COLON_DASH != pToken->getTokenType())
	{
		return false;
	}

	parser->goToNextToken();
	pToken = parser->getCurrentToken();


	while(ID==pToken->getTokenType())
	{
		Predicate predicate;
		ret = predicate.parseTokens(parser);

		if(false == ret)
		{
			return false;
		}

		mPredicates.push_back(predicate);

		//parser->goToNextToken();
		pToken = parser->getCurrentToken();

		if(PERIOD==pToken->getTokenType())
		{
			parser->goToNextToken();
			return true;
		}
		else if(COMMA ==pToken->getTokenType())
		{
			parser->goToNextToken();
			pToken = parser->getCurrentToken();
		}
	}

	return false;
}
