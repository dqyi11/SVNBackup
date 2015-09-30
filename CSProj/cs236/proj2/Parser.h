/*
 * Parser.h
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#ifndef PARSER_H_
#define PARSER_H_

#include "Lex.h"
#include "DatalogProgram.h"
#include "Domain.h"

class DatalogProgram;

class Parser {
public:
	Parser(const char* fileName);
	virtual ~Parser();

	void printTokens();

	std::string parseTokens();

	Token * getCurrentToken();

	bool goToNextToken();

	Domain * getDomain();

	int getCurrentIndex() { return mCurrentTokenIndex; }
	int getTokenNum() { return mTokensNum; };

	std::vector<Token> getTokens(int start, int end);

private:
	Lex * mpLexer;

	std::vector<Token*>* mpTokens;
	int mTokensNum;
	int mCurrentTokenIndex;

	DatalogProgram * mpDatalogProgram;

};

#endif /* PARSER_H_ */
