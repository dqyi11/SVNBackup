/*
 * Expression.h
 *
 *  Created on: Feb 6, 2013
 *      Author: kyle
 */

#ifndef EXPRESSION_H_
#define EXPRESSION_H_

#include "Lex.h"
#include "Domain.h"
#include "Parameter.h"
#include <sstream>
#include <string>

namespace std {

class Parameter;

class Expression {
public:
	Expression(Lex* lex, Domain* domain);
	string toString();
private:
	Parameter* parameterOne;
	char op;
	Parameter* parameterTwo;
};

} /* namespace std */
#endif /* EXPRESSION_H_ */
