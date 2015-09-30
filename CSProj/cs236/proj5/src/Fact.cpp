/*
 * Fact.cpp
 *
 *  Created on: Feb 5, 2013
 *      Author: kyle
 */

#include "Fact.h"

Fact::Fact(Lex* lex, Domain* domain) {
	if(lex->assertCurrentToken(ID)){
		identifier = lex->getCurrentToken()->getTokensValue();
		lex->advance();
	}
	if(lex->assertCurrentToken(LEFT_PAREN))
		lex->advance();
	lex->assertCurrentToken(STRING);
	while(lex->getCurrentToken()->getTokenType() == STRING){
		identifierList.push_back(lex->getCurrentToken()->getTokensValue());
		domain->addElement(lex->getCurrentToken()->getTokensValue());
		lex->advance();
		if(lex->getCurrentToken()->getTokenType() == COMMA)
			lex->advance();
		else
			lex->assertCurrentToken(RIGHT_PAREN);
	}
	lex->assertCurrentToken(RIGHT_PAREN);
	lex->advance();
	lex->assertCurrentToken(PERIOD);
	lex->advance();
}

Fact::~Fact() {
	// TODO Auto-generated destructor stub
}

string Fact::toString() {
	stringstream rep;
	rep << identifier << "(";
	for(vector<string>::iterator it = identifierList.begin(); it != identifierList.end(); ++it){
		rep << *it;
		if(it != identifierList.end() -1 )
			rep << ",";
	}
	rep << ").";
	return rep.str();
}
