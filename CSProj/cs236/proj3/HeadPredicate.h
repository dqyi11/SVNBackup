/*
 * Predicate.h
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#ifndef HEADPREDICATE_H_
#define HEADPREDICATE_H_

#include "DatalogElement.h"
#include "Token.h"
#include <vector>

class Parser;

class HeadPredicate: public DatalogElement {
public:
	HeadPredicate();
	virtual ~HeadPredicate();

	virtual std::string toString();

	bool parseTokens(Parser * parser);

private:
	std::vector<Token> mTokens;
};

#endif /* HEADPREDICATE_H_ */
