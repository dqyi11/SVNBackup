/*
 * Predicate.cpp
 *
 *  Created on: Feb 5, 2013
 *      Author: kyle
 */

#include "Predicate.h"

using namespace std;

Predicate::Predicate(Lex* lex, Domain* domain) {
	if(lex->assertCurrentToken(ID)){
		identifier = lex->getCurrentToken()->getTokensValue();
		lex->advance();
	}
	if(lex->assertCurrentToken(LEFT_PAREN))
		lex->advance();
	while(lex->getCurrentToken()->getTokenType() == ID || lex->getCurrentToken()->getTokenType() == STRING
			|| lex->getCurrentToken()->getTokenType() == LEFT_PAREN){
		parameterList.push_back(new Parameter(lex, domain));
		if(lex->getCurrentToken()->getTokenType() == COMMA)
			lex->advance();
		else
			lex->assertCurrentToken(RIGHT_PAREN);
	}
	lex->advance();
}

string Predicate::toString(){
	stringstream rep;
	rep << identifier << "(";
	for(vector<Parameter*>::iterator it = parameterList.begin(); it != parameterList.end(); ++it){
		rep << (*it)->toString();
		if(it != parameterList.end() -1 )
			rep << ",";
	}
	rep << ")";
	return rep.str();

}
