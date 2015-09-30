/*
 * Parameter.h
 *
 *  Created on: Feb 6, 2013
 *      Author: kyle
 */

#ifndef PARAMETER_H_
#define PARAMETER_H_

#include "Lex.h"
#include "Domain.h"
#include "Expression.h"

namespace std {

class Expression;

class Parameter {
public:
	Parameter(Lex* lex, Domain* domain);
	string toString();

	string getParaString() { return paraString; };
	string getParaIdentifier() { return paraIdentifier; };
private:
	string paraString;
	string paraIdentifier;
	Expression* paraExpression;
};

} /* namespace std */
#endif /* PARAMETER_H_ */
