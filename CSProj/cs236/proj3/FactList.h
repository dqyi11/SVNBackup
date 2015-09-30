/*
 * FactList.h
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#ifndef FACTLIST_H_
#define FACTLIST_H_

#include "DatalogElement.h"
#include "Fact.h"
#include <vector>

class Parser;

using namespace std;

class FactList: public DatalogElement {
public:
	FactList();
	virtual ~FactList();

	virtual std::string toString();

	bool parseTokens(Parser * parser);

private:
	vector<Fact> mFacts;
};

#endif /* FACTLIST_H_ */
