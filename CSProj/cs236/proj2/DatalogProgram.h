/*
 * DatalogProgram.h
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#ifndef DATALOGPROGRAM_H_
#define DATALOGPROGRAM_H_

#include "DatalogElement.h"
#include "SchemeList.h"
#include "FactList.h"
#include "RuleList.h"
#include "QueryList.h"
#include "Domain.h"

class Parser;

class DatalogProgram: public DatalogElement {
public:
	DatalogProgram();
	virtual ~DatalogProgram();

	virtual std::string toString();

	bool parseTokens(Parser * parser);

	Domain * getDomain() { return mpDomain; };

private:
	SchemeList * mpSchemeList;
	FactList   * mpFactList;
	RuleList   * mpRuleList;
	QueryList  * mpQueryList;

	Domain     * mpDomain;
};

#endif /* DATALOGPROGRAM_H_ */
