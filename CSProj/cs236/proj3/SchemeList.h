/*
 * SchemeList.h
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#ifndef SCHEMELIST_H_
#define SCHEMELIST_H_

#include "DatalogElement.h"
#include "Scheme.h"
#include <vector>

class Parser;

using namespace std;

class SchemeList: public DatalogElement {
public:
	SchemeList();
	virtual ~SchemeList();

	virtual std::string toString();

	bool parseTokens(Parser * parser);

private:
	vector<Scheme> mSchemes;
};

#endif /* SCHEMELIST_H_ */
