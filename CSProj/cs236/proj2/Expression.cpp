/*
 * Expression.cpp
 *
 *  Created on: Feb 3, 2013
 *      Author: walter
 */

#include "Expression.h"
#include "Parser.h"
#include <iostream>

using namespace std;

Expression::Expression() {
	// TODO Auto-generated constructor stub
    mTokens.clear();
}

Expression::~Expression() {
	// TODO Auto-generated destructor stub
	mTokens.clear();
}

std::string Expression::toString()
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

bool Expression::parseTokens(Parser * parser)
{
	bool ret = false;
	int startIndex = 0, endIndex = 0;

	// cout << "Expression::parseTokens" << endl;

	if(NULL==parser)
	{
		return false;
	}

	Token * pToken = parser->getCurrentToken();

	startIndex = parser->getCurrentIndex();

	if(LEFT_PAREN != pToken->getTokenType())
	{
		return false;
	}

	parser->goToNextToken();
	pToken = parser->getCurrentToken();

    Parameter lhs;
    ret = lhs.parseTokens(parser);

    if(false==ret)
    {
    	return false;
    }

    pToken = parser->getCurrentToken();

    //cout << "lhs parameter OK: " << pToken->getTokensValue() << endl;

    if(MULTIPLY != pToken->getTokenType()
    		&&
    	ADD != pToken->getTokenType())
    {
    	return false;
    }

    parser->goToNextToken();
    pToken = parser->getCurrentToken();

    Parameter rhs;
    ret = rhs.parseTokens(parser);

    if(false==ret)
    {
    	return false;
    }

    pToken = parser->getCurrentToken();

    if(RIGHT_PAREN != pToken->getTokenType())
    {
    	return false;
    }

    endIndex = parser->getCurrentIndex();

    mTokens = parser->getTokens(startIndex, endIndex);

    //cout << "Expression::toString = " << toString() << endl;

    //parser->goToNextToken();

    return true;
}
