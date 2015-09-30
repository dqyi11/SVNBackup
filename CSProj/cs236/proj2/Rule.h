/*
 * Rule.h
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#ifndef RULE_H_
#define RULE_H_

#include "DatalogElement.h"
#include "HeadPredicate.h"
#include "Predicate.h"
#include <vector>

using namespace std;

class Parser;

class Rule: public DatalogElement {
public:
	Rule();
	virtual ~Rule();

	virtual std::string toString();

	bool parseTokens(Parser * parser);

private:
	HeadPredicate mHead;
	vector<Predicate> mPredicates;

};

#endif /* RULE_H_ */
