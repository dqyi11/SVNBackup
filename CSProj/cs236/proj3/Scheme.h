/*
 * Scheme.h
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#ifndef SCHEME_H_
#define SCHEME_H_

#include "DatalogElement.h"
#include "Token.h"
#include <vector>

class Parser;

class Scheme: public DatalogElement {
public:
	Scheme();
	virtual ~Scheme();

	virtual std::string toString();

	bool parseTokens(Parser * parser);

private:
	std::vector<Token> mTokens;
};

#endif /* SCHEME_H_ */
