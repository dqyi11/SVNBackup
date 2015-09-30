/*
 * Expression.h
 *
 *  Created on: Feb 3, 2013
 *      Author: walter
 */

#ifndef EXPRESSION_H_
#define EXPRESSION_H_

#include "DatalogElement.h"
#include "Token.h"
#include <vector>

class Parser;
class Parameter;

class Expression: public DatalogElement {
public:
	Expression();
	virtual ~Expression();

	virtual std::string toString();

	bool parseTokens(Parser * parser);

private:
	std::vector<Token> mTokens;
};

#endif /* EXPRESSION_H_ */
