/*
 * Rule.h
 *
 *  Created on: Feb 5, 2013
 *      Author: kyle
 */

#ifndef RULE_H_
#define RULE_H_

#include <vector>
#include <string>
#include <sstream>
#include "Lex.h"
#include "Domain.h"
#include "Predicate.h"

//namespace std {

class Rule {
	friend class Database;
public:
	Rule(Lex* lex, Domain* domain);
	~Rule();
	string toString();
protected:
	string identifier;
	vector<string> identifierList;
	vector<Predicate*> predicateList;
};

//} /* namespace std */
#endif /* RULE_H_ */
