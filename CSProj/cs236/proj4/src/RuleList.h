/*
 * RuleList.h
 *
 *  Created on: Feb 5, 2013
 *      Author: kyle
 */

#ifndef RULELIST_H_
#define RULELIST_H_

#include "Lex.h"
#include "Domain.h"
#include "TokenType.h"
#include "Rule.h"

//namespace std {

class RuleList {
	friend class Database;
public:
	RuleList(Lex* lex, Domain* domain);
	~RuleList();
	string toString();
protected:
	vector<Rule*> rules;
};

//} /* namespace std */
#endif /* RULELIST_H_ */
