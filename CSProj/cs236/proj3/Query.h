/*
 * Query.h
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#ifndef QUERY_H_
#define QUERY_H_

#include "DatalogElement.h"
#include "Predicate.h"

class Parser;

class Query: public DatalogElement {
public:
	Query();
	virtual ~Query();

	virtual std::string toString();

	bool parseTokens(Parser * parser);

private:
	Predicate mPredicate;
};

#endif /* QUERY_H_ */
