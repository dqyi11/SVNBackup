/*
 * QueryList.h
 *
 *  Created on: Feb 5, 2013
 *      Author: kyle
 */

#ifndef QUERYLIST_H_
#define QUERYLIST_H_

#include "Lex.h"
#include "Domain.h"
#include "TokenType.h"
#include "Scheme.h"
#include "Predicate.h"

namespace std {

class QueryList {
public:
	QueryList(Lex* lex, Domain* domain);
	string toString();
//private:
	vector<Predicate*> queries;
};

} /* namespace std */
#endif /* QUERYLIST_H_ */
