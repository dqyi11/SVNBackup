/*
 * RuleList.h
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#ifndef RULELIST_H_
#define RULELIST_H_

#include "DatalogElement.h"
#include "Rule.h"
#include <vector>

class Parser;

using namespace std;

class RuleList: public DatalogElement {
public:
	RuleList();
	virtual ~RuleList();

	virtual std::string toString();

	bool parseTokens(Parser * parser);

private:
	vector<Rule> mRules;
};

#endif /* RULELIST_H_ */
