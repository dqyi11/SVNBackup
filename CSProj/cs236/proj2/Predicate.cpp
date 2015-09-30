/*
 * Predicate.cpp
 *
 *  Created on: Feb 3, 2013
 *      Author: walter
 */

#include "Predicate.h"
#include "Parser.h"
#include <iostream>

using namespace std;

Predicate::Predicate() {
	// TODO Auto-generated constructor stub
	mParameters.clear();
}

Predicate::~Predicate() {
	// TODO Auto-generated destructor stub
	mParameters.clear();
}

std::string Predicate::toString()
{
	std::string output = "";
	output = mId.getTokensValue();
	output += "(";
	std::vector<Parameter>::iterator it;
	for(it=mParameters.begin();it!=mParameters.end();it++)
	{
		output += (*it).toString();
		if(it!=mParameters.end()-1)
		{
			output += ",";
		}
	}
	output += ")";
	//output += "\n";

	return output;
}

bool Predicate::parseTokens(Parser * parser)
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

	mId = *pToken;

    parser->goToNextToken();
    pToken = parser->getCurrentToken();
    if(LEFT_PAREN != pToken->getTokenType())
    {
    	return false;
    }

    parser->goToNextToken();
    pToken = parser->getCurrentToken();

	while(ID==pToken->getTokenType()
			||
		  STRING==pToken->getTokenType()
		    ||
		  LEFT_PAREN == pToken->getTokenType()
		 )
	{
		Parameter parameter;
		ret = parameter.parseTokens(parser);

		if(false == ret)
		{
			return false;
		}

		mParameters.push_back(parameter);

		pToken = parser->getCurrentToken();

		//cout << "current pos: " << pToken->getTokensValue() << endl;

		if(COMMA == pToken->getTokenType()
				||
		    RIGHT_PAREN == pToken->getTokenType())
		{
			parser->goToNextToken();
			pToken = parser->getCurrentToken();
		}
		else
		{
			return false;
		}
	}

	if(COMMA == pToken->getTokenType()
			||
	   Q_MARK == pToken->getTokenType()
			||
	   PERIOD == pToken->getTokenType())
	{
		//parser->goToNextToken();
		return true;
	}

	return false;

}
