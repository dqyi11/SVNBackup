/*
 * DatalogProgram.cpp
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#include "DatalogProgram.h"
#include <iostream>
#include "Parser.h"

using namespace std;

DatalogProgram::DatalogProgram() {
	// TODO Auto-generated constructor stub
	mpSchemeList = new SchemeList();
	mpFactList = new FactList();
	mpRuleList = new RuleList();
	mpQueryList = new QueryList();

	mpDomain = new Domain();

}

DatalogProgram::~DatalogProgram() {
	// TODO Auto-generated destructor stub
	if(mpDomain)
	{
		delete mpDomain;
		mpDomain = NULL;
	}

	if(mpSchemeList)
	{
		delete mpSchemeList;
		mpSchemeList = NULL;
	}
	if(mpFactList)
	{
		delete mpFactList;
		mpFactList = NULL;
	}
	if(mpRuleList)
	{
		delete mpRuleList;
		mpRuleList = NULL;
	}
	if(mpQueryList)
	{
		delete mpQueryList;
		mpQueryList = NULL;
	}
}

std::string DatalogProgram::toString()
{
	std::string output = "";
	output += mpSchemeList->toString();
	output += mpFactList->toString();
	output += mpRuleList->toString();
	output += mpQueryList->toString();
	output += mpDomain->toString();

	return output;
}

bool DatalogProgram::parseTokens(Parser * parser)
{
	if(mpSchemeList)
	{
		Token * pToken = parser->getCurrentToken();
		if(SCHEMES != pToken->getTokenType())
		{
			return false;
		}

		parser->goToNextToken();
		pToken = parser->getCurrentToken();

		if(COLON != pToken->getTokenType())
		{
			return false;
		}

		parser->goToNextToken();

		if(false == mpSchemeList->parseTokens(parser))
		{
			//cout << "SCHEME LIST = false" << endl;
			return false;
		}

	}

	//cout << mpSchemeList->toString();

	if(mpFactList)
	{
		Token * pToken = parser->getCurrentToken();
		if(FACTS != pToken->getTokenType())
		{
			return false;
		}

		parser->goToNextToken();
		pToken = parser->getCurrentToken();

		if(COLON != pToken->getTokenType())
		{
			return false;
		}

		parser->goToNextToken();

		if(false == mpFactList->parseTokens(parser))
		{
			return false;
		}
	}

	//cout << mpFactList->toString();

	if(mpRuleList)
	{
		Token * pToken = parser->getCurrentToken();
		if(RULES != pToken->getTokenType())
		{
			return false;
		}

		parser->goToNextToken();
		pToken = parser->getCurrentToken();

		if(COLON != pToken->getTokenType())
		{
			return false;
		}

		parser->goToNextToken();


		if(false == mpRuleList->parseTokens(parser))
		{
			return false;
		}
	}

	//cout << mpRuleList->toString();

	if(mpQueryList)
	{
		Token * pToken = parser->getCurrentToken();
		if(QUERIES != pToken->getTokenType())
		{
			return false;
		}

		parser->goToNextToken();
		pToken = parser->getCurrentToken();

		if(COLON != pToken->getTokenType())
		{
			return false;
		}

		parser->goToNextToken();


		if(false == mpQueryList->parseTokens(parser))
		{
			return false;
		}
	}

	//cout << mpQueryList->toString();

	return true;
}

