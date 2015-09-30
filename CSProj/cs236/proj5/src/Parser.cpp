/*
 * Parser.cpp
 *
 *  Created on: Feb 2, 2013
 *      Author: kyle
 */

#include "Parser.h"
#include "Lex.h"
#include <iostream>


using namespace std;

Parser::Parser(const char* fileName) :lex(NULL) {
	lex = new Lex(fileName);
    lex->clearComments(); //who likes 'em anyway?
    datalogProgram = new DatalogProgram(lex);
}
Parser::~Parser(){
	delete lex;
	delete datalogProgram;
}

DatalogProgram* Parser::getProgram() { return datalogProgram; }

Token* Parser::getErrorToken() {	return lex->getErrorToken(); }

string Parser::toString() {
	return datalogProgram->toString();
}
