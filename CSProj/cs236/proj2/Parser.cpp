/*
 * Parser.cpp
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#include "Parser.h"
#include <iostream>

using namespace std;

Parser::Parser(const char* fileName)
{
	// TODO Auto-generated constructor stub
	mpLexer = new Lex(fileName);
	mpTokens = mpLexer->getTokens();
	mTokensNum = mpTokens->size();
	mCurrentTokenIndex = 0;

	mpDatalogProgram = new DatalogProgram();
}

Parser::~Parser() {
	// TODO Auto-generated destructor stub
	if(mpDatalogProgram)
	{
		delete mpDatalogProgram;
		mpDatalogProgram = NULL;
	}

	if(mpLexer)
	{
		delete mpLexer;
		mpLexer = NULL;
	}
}

void Parser::printTokens()
{
	cout << "printTokens():" << endl;
	if(mpTokens)
	{
		for(int i=0;i<mTokensNum;i++)
		{
			cout << (*mpTokens)[i]->toString();
		}
	}
}

std::string Parser::parseTokens()
{
	std::string output = "";
	if(NULL==mpDatalogProgram)
	{
		return "ERROR\n";
	}

	if(true==mpDatalogProgram->parseTokens(this))
	{
		output = "Success!\n";
		output += mpDatalogProgram->toString();
	}
	else
	{
		output = "Failure!\n";
		output += "  ";
		output += getCurrentToken()->toString();
	}

	return output;
}

Token * Parser::getCurrentToken()
{
	if(mCurrentTokenIndex < 0 || mCurrentTokenIndex >= mTokensNum)
	{
		return NULL;
	}

	while(COMMENT==(*mpTokens)[mCurrentTokenIndex]->getTokenType()
			&&
			mCurrentTokenIndex >= 0
			&&
			mCurrentTokenIndex < mTokensNum)
	{
		mCurrentTokenIndex++;
	}

	return (*mpTokens)[mCurrentTokenIndex];
}

bool Parser::goToNextToken()
{
	if(mCurrentTokenIndex < mTokensNum-1)
	{
		mCurrentTokenIndex++;
		while(COMMENT==(*mpTokens)[mCurrentTokenIndex]->getTokenType()
				&&
				mCurrentTokenIndex >= 0
				&&
				mCurrentTokenIndex < mTokensNum)
		{
			mCurrentTokenIndex++;
		}

		// cout << "goToNextToken : " << (*mpTokens)[mCurrentTokenIndex]->toString();
		return true;
	}

	return false;
}

Domain * Parser::getDomain()
{
	if(mpDatalogProgram)
	{
		return mpDatalogProgram->getDomain();
	}

	return NULL;
}

std::vector<Token> Parser::getTokens(int start, int end)
{
	std::vector<Token> tokens;
	tokens.clear();

	if(start<0 || start>end || end>mTokensNum-1)
	{
		return tokens;
	}

	for(int i=start;i<=end;i++)
	{
		if(COMMENT!=(*mpTokens)[i]->getTokenType())
		{
 		tokens.push_back(*((*mpTokens)[i]));
		}
	}

	return tokens;
}
