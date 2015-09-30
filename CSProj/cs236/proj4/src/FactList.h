/*
 * FactList.h
 *
 *  Created on: Feb 2, 2013
 *      Author: kyle
 */

#ifndef FACTLIST_H_
#define FACTLIST_H_

#include "Lex.h"
#include "Domain.h"
#include "TokenType.h"
#include "Fact.h"

// namespace std {

class FactList {
	friend class Relation;
	friend class Database;
public:
	FactList(Lex* lex, Domain* domain);
	virtual ~FactList();
	string toString();
protected:
	vector<Fact*> facts;
};

// } /* namespace std */
#endif /* factLIST_H_ */
