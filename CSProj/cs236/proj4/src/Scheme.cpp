/*
 * Scheme.cpp
 *
 *  Created on: Feb 2, 2013
 *      Author: kyle
 */

#include "Scheme.h"

//namespace std {

Scheme::Scheme(Lex* lex) {
	if(lex->assertCurrentToken(ID)){
		identifier = lex->getCurrentToken()->getTokensValue();
		lex->advance();
	}
	if(lex->assertCurrentToken(LEFT_PAREN))
		lex->advance();
	lex->assertCurrentToken(ID);
	while(lex->getCurrentToken()->getTokenType() == ID){
		identifierList.push_back(lex->getCurrentToken()->getTokensValue());
		lex->advance();
		if(lex->getCurrentToken()->getTokenType() == COMMA)
			lex->advance();
		else
			lex->assertCurrentToken(RIGHT_PAREN);
	}
	lex->assertCurrentToken(RIGHT_PAREN);
	lex->advance();
}

Scheme::~Scheme() {
	// TODO Auto-generated destructor stub
}

string Scheme::toString() {
	stringstream rep;
	rep << identifier << "(";
	for(vector<string>::iterator it = identifierList.begin(); it != identifierList.end(); ++it){
		rep << *it;
		if(it != identifierList.end() -1 )
			rep << ",";
	}
	rep << ")";
	return rep.str();
}

//} /* namespace std */
