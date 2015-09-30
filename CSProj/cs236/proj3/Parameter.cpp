/*
 * Parameter.cpp
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#include "Parameter.h"
#include "Expression.h"
#include "Parser.h"
#include <iostream>

using namespace std;

Parameter::Parameter() {
	// TODO Auto-generated constructor stub
	mTokens.clear();
}

Parameter::~Parameter() {
	// TODO Auto-generated destructor stub
	mTokens.clear();
}

std::string Parameter::toString()
{
	std::string output = "";
	std::vector<Token>::iterator it;
	for(it=mTokens.begin();it!=mTokens.end();it++)
	{
		output += (*it).getTokensValue();
	}
	//output += "\n";

	return output;
}

bool Parameter::parseTokens(Parser * parser)
{
	bool ret = false;
	int startIndex = 0, endIndex = 0;

	// cout << "Parameter::parseTokens" << endl;

	if(NULL==parser)
	{
		return false;
	}

	Token * pToken = parser->getCurrentToken();
	startIndex = parser->getCurrentIndex();

	if(ID == pToken->getTokenType())
	{
		// cout << " ID found " << endl;
	}
	else if(STRING == pToken->getTokenType())
	{
		// cout << " STRING found " << endl;
		parser->getDomain()->addToken(*pToken);

	}
	else if(LEFT_PAREN == pToken->getTokenType())
	{
		// cout << " EXPRESSION found " << endl;
		Expression expression;
		ret = expression.parseTokens(parser);
		if(false == ret)
		{
			return false;
		}

		/*
		pToken = parser->getCurrentToken();

		if(RIGHT_PAREN!= pToken->getTokenType())
		{
			return false;
		}
		parser->goToNextToken();
		*/
	}

    endIndex = parser->getCurrentIndex();

    mTokens = parser->getTokens(startIndex, endIndex);

    parser->goToNextToken();

    // cout << " Parameter return TURE " << endl;
    return true;
}
