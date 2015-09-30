/*
 * Rule.cpp
 *
 *  Created on: Feb 5, 2013
 *      Author: kyle
 */

#include "Rule.h"

//namespace std {

Rule::Rule(Lex* lex, Domain* domain) {
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
	lex->assertCurrentToken(COLON_DASH);
	lex->advance();

	lex->assertCurrentToken(ID);
	while(lex->getCurrentToken()->getTokenType() == ID){
		predicateList.push_back(new Predicate(lex, domain));
		if(lex->getCurrentToken()->getTokenType() == COMMA)
			lex->advance();
		else
			lex->assertCurrentToken(PERIOD);
	}
	lex->advance();
}

Rule::~Rule() {
	// TODO Auto-generated destructor stub
}

string Rule::toString() {
	stringstream rep;
	rep << identifier << "(";
	for(vector<string>::iterator it = identifierList.begin(); it != identifierList.end(); ++it){
		rep << *it;
		if(it != identifierList.end() -1 )
			rep << ",";
	}
	rep << ") :- ";
	for(vector<Predicate*>::iterator it = predicateList.begin(); it != predicateList.end(); ++it){
		rep << (*it)->toString();
		if(it != predicateList.end() -1 )
			rep << ",";
	}
	rep << ".";
	return rep.str();
}

//} /* namespace std */
