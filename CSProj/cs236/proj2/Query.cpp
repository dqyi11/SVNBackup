/*
 * Query.cpp
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#include "Query.h"
#include "Parser.h"
#include <iostream>

using namespace std;

Query::Query() {
	// TODO Auto-generated constructor stub

}

Query::~Query() {
	// TODO Auto-generated destructor stub
}

std::string Query::toString()
{
	std::string output = "";
	output = mPredicate.toString();
	output += "?\n";
	return output;
}

bool Query::parseTokens(Parser * parser)
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

	ret = mPredicate.parseTokens(parser);

	if(false==ret)
	{
		return false;
	}

	pToken = parser->getCurrentToken();

	if(Q_MARK !=pToken->getTokenType())
	{
		return false;
	}

	parser->goToNextToken();
	//pToken = parser->getCurrentToken();

	return true;
}
