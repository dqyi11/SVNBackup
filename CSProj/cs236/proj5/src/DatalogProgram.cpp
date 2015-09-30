/*
 * DatalogProgram.cpp
 *
 *  Created on: Feb 2, 2013
 *      Author: kyle
 */

#include "DatalogProgram.h"

DatalogProgram::DatalogProgram(Lex* lex) {
	domain = new Domain();
	schemeList = new SchemeList(lex);
	factList = new FactList(lex, domain);
	ruleList = new RuleList(lex, domain);
	queryList = new QueryList(lex, domain);

	lex->assertCurrentToken(EOF);
}

DatalogProgram::~DatalogProgram() {
	delete schemeList;
}

string DatalogProgram::toString(){
	stringstream rep;
	rep << schemeList->toString();
	rep << factList->toString();
	rep << ruleList->toString();
	rep << queryList->toString();
	rep << domain->toString();

	return rep.str();
}
