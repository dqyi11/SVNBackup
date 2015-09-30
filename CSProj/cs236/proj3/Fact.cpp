/*
 * Fact.cpp
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#include "Fact.h"
#include "Parser.h"
#include <iostream>

using namespace std;

Fact::Fact() {
	// TODO Auto-generated constructor stub
	mTokens.clear();
}

Fact::~Fact() {
	// TODO Auto-generated destructor stub
	mTokens.clear();
}

std::string Fact::toString()
{
	std::string output = "";
	std::vector<Token>::iterator it;
	for(it=mTokens.begin();it!=mTokens.end();it++)
	{
		output += (*it).getTokensValue();
	}
	output += "\n";

	return output;
}

bool Fact::parseTokens(Parser * parser)
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

	if(STRING!=pToken->getTokenType())
	{
		// empty constant list
		return false;
	}

	while(STRING == pToken->getTokenType())
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

	parser->goToNextToken();
	pToken = parser->getCurrentToken();
	if(PERIOD != pToken->getTokenType())
	{
		return false;
	}

	endIndex = parser->getCurrentIndex();

	mTokens = parser->getTokens(startIndex, endIndex);

	parser->goToNextToken();

	return true;
}
