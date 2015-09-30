/*
 * Predicate.h
 *
 *  Created on: Feb 3, 2013
 *      Author: walter
 */

#ifndef PREDICATE_H_
#define PREDICATE_H_

#include "DatalogElement.h"
#include "Token.h"
#include "Parameter.h"
#include <vector>

class Parser;

class Predicate: public DatalogElement {
public:
	Predicate();
	virtual ~Predicate();

	virtual std::string toString();

	bool parseTokens(Parser * parser);

private:
	Token mId;
	std::vector<Parameter> mParameters;
};

#endif /* PREDICATE_H_ */
