/*
 * Parameter.cpp
 *
 *  Created on: Feb 6, 2013
 *      Author: kyle
 */

#include "Parameter.h"

namespace std {

Parameter::Parameter(Lex* lex, Domain* domain): paraString(""), paraIdentifier(""), paraExpression(NULL) {
	if(lex->getCurrentToken()->getTokenType() == STRING){
		paraString = lex->getCurrentToken()->getTokensValue();
		domain->addElement(paraString);
		lex->advance();
	}else if(lex->getCurrentToken()->getTokenType() == ID){
		paraIdentifier = lex->getCurrentToken()->getTokensValue();
		lex->advance();
	}else if(lex->getCurrentToken()->getTokenType() == LEFT_PAREN){
		paraExpression = new Expression(lex, domain);
	}
	else
		lex->markErrorToken();
}

string Parameter::toString() {
	if(paraString != "")
		return paraString;
	else if(paraIdentifier != "")
		return paraIdentifier;
	else
		return paraExpression->toString();
}


} /* namespace std */

