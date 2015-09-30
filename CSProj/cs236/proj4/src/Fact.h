/*
 * Fact.h
 *
 *  Created on: Feb 5, 2013
 *      Author: kyle
 */

#ifndef FACT_H_
#define FACT_H_

#include <vector>
#include <string>
#include <sstream>
#include "Lex.h"
#include "Domain.h"

class Fact {
	friend class Relation;
	friend class Database;
public:
	Fact(Lex* lex, Domain* domain);
	~Fact();
	string toString();
protected:
	string identifier;
	vector<string> identifierList;
};

#endif /* FACT_H_ */
