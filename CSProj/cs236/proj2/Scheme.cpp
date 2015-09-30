/*
 * Scheme.cpp
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#include "Scheme.h"
#include "Parser.h"
#include <iostream>

using namespace std;

Scheme::Scheme() {
	// TODO Auto-generated constructor stub
	mTokens.clear();
}

Scheme::~Scheme() {
	// TODO Auto-generated destructor stub
	mTokens.clear();
}

std::string Scheme::toString()
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

bool Scheme::parseTokens(Parser * parser)
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

	/* in case ID LIST is empty */
	if(ID!=pToken->getTokenType())
	{
		return false;
	}

	while(ID == pToken->getTokenType())
	{
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
