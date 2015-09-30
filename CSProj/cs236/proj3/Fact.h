/*
 * Fact.h
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#ifndef FACT_H_
#define FACT_H_

#include "DatalogElement.h"
#include "Token.h"
#include <vector>

class Parser;

class Fact: public DatalogElement {
public:
	Fact();
	virtual ~Fact();

	virtual std::string toString();

	bool parseTokens(Parser * parser);

private:
	std::vector<Token> mTokens;
};

#endif /* FACT_H_ */
