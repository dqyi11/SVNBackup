/*
 * Database.h
 *
 *  Created on: Feb 19, 2013
 *      Author: walter
 */

#ifndef DATABASE_H_
#define DATABASE_H_

#include <vector>
#include <string>
#include <sstream>
#include "Relation.h"
#include "DatalogProgram.h"
#include "Parser.h"
#include "Scheme.h"
#include "RelationRule.h"

class DatalogProgram;

class Database {
public:
	Database(DatalogProgram * program);
	virtual ~Database();

	void addRelations(std::vector<Scheme*> schemes);
	void addTuples(vector<Fact*> facts);
	void addRules(vector<Rule*> rules);
	void processRules();
	Relation * getRelation(std::string name);
	std::string toString();

	bool importQuery(Predicate * query);
	bool importQueryList();
	bool importRule(Rule * rule);

	Relation * findRelation(std::string name);

private:
	DatalogProgram * mpDatalogProgram;
	std::vector<Relation*> mRelations;
	std::vector<RelationRule> mRules;
};

#endif /* DATABASE_H_ */
