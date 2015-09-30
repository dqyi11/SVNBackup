/*
 * QueryList.h
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#ifndef QUERYLIST_H_
#define QUERYLIST_H_

#include "DatalogElement.h"
#include "Query.h"
#include <vector>

class Parser;

using namespace std;

class QueryList: public DatalogElement {
public:
	QueryList();
	virtual ~QueryList();

	virtual std::string toString();

	bool parseTokens(Parser * parser);

private:
	vector<Query> mQueries;

};

#endif /* QUERYLIST_H_ */
