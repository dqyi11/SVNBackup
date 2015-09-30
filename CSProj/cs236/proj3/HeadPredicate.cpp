/*
 * Predicate.cpp
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#include "HeadPredicate.h"
#include "Parser.h"
#include <iostream>

using namespace std;

HeadPredicate::HeadPredicate() {
	// TODO Auto-generated constructor stub
	mTokens.clear();
}

HeadPredicate::~HeadPredicate() {
	// TODO Auto-generated destructor stub
	mTokens.clear();
}

std::string HeadPredicate::toString()
{
	std::string output = "";
	std::vector<Token>::iterator it;
	for(it=mTokens.begin();it!=mTokens.end();it++)
	{
		output += (*it).getTokensValue();
	}

	return output;
}

bool HeadPredicate::parseTokens(Parser * parser)
{
	int startIndex = 0, endIndex = 0;

	if(NULL==parser)
	{
		return false;
	}

	Token * pToken = parser->getCurrentToken();

	if(ID != pToken->getTokenType())
	{
		return false;
	}

	startIndex = parser->getCurrentIndex();

	parser->goToNextToken();
	pToken = parser->getCurrentToken();

	if(LEFT_PAREN != pToken->getTokenType())
	{
		return false;
	}

	parser->goToNextToken();
	pToken = parser->getCurrentToken();

	if(ID!=pToken->getTokenType())
	{
		return false;
	}

	while(ID == pToken->getTokenType())
	{
		parser->getDomain()->addToken(*pToken);
		parser->goToNextToken();
		pToken = parser->getCurrentToken();

		if(COMMA==pToken->getTokenType())
		{
			parser->goToNextToken();
			pToken = parser->getCurrentToken();
		}
	}


	if(RIGHT_PAREN != pToken->getTokenType())
	{
		return false;
	}

	endIndex = parser->getCurrentIndex();

	mTokens = parser->getTokens(startIndex, endIndex);

	parser->goToNextToken();

	return true;
}
