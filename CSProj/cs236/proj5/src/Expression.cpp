/*
 * Expression.cpp
 *
 *  Created on: Feb 6, 2013
 *      Author: kyle
 */

#include "Expression.h"

namespace std {

Expression::Expression(Lex* lex, Domain* domain) {
	lex->assertCurrentToken(LEFT_PAREN);
	lex->advance();
	parameterOne = new Parameter(lex, domain);
	if(lex->getCurrentToken()->getTokenType() == ADD || lex->getCurrentToken()->getTokenType() == MULTIPLY ){
		op = lex->getCurrentToken()->getTokensValue()[0];
		lex->advance();
	}else
		lex->markErrorToken();
	parameterTwo = new Parameter(lex, domain);
	lex->assertCurrentToken(RIGHT_PAREN);
	lex->advance();
}

string Expression::toString() {
	stringstream rep;
	rep << "(" << parameterOne->toString() << op << parameterTwo->toString() << ")";
	return rep.str();
}

} /* namespace std */
