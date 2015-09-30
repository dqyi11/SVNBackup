/*
 * DatalogProgram.h
 *
 *  Created on: Feb 2, 2013
 *      Author: kyle
 */

#ifndef DATALOGPROGRAM_H_
#define DATALOGPROGRAM_H_
#include "Lex.h"
#include "SchemeList.h"
#include "FactList.h"
#include "RuleList.h"
#include "QueryList.h"
#include "Domain.h"
#include <sstream>

class DatalogProgram {
	friend class Database;
public:
	DatalogProgram(Lex* lex);
	~DatalogProgram();
	string toString();

protected:
	Domain* domain;
	SchemeList* schemeList;
	FactList* factList;
	RuleList* ruleList;
	QueryList* queryList;
};

#endif /* DATALOGPROGRAM_H_ */
