/*
 * Parser.h
 *
 *  Created on: Feb 2, 2013
 *      Author: kyle
 */

#ifndef PARSER_H_
#define PARSER_H_

#include "Token.h"
#include "DatalogProgram.h"

class Parser {
public:
	Parser(const char* fileName);
	~Parser();

	void parseFile();
	void parseToken(Token* token);
	Token* getErrorToken();
	DatalogProgram* getProgram();
	std::string toString();

private:
	DatalogProgram* datalogProgram;
	Lex* lex;
};

#endif /* PARSER_H_ */
