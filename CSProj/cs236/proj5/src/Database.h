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
#include "Graph.h"

class DatalogProgram;

class Database {
public:
	Database(DatalogProgram * program);
	virtual ~Database();

	void addRelations(std::vector<Scheme*> schemes);
	void addTuples(vector<Fact*> facts);
	void addRules(vector<Rule*> rules);
	Relation * getRelation(std::string name);
	RelationRule * getRelationRule(std::string name);
	bool processRulesOnce(vector<RelationRule> rules);
	void processRules(vector<RelationRule> rules, bool LFP);
	std::string toString();

	bool importQuery(Predicate * query);
	bool importQueryList();
	bool importRule(Rule * rule);

	Relation * findRelation(std::string name);

	//Graph interactions
	void initRuleGraph();
	vector<RelationRule> getRulesFromTree(Graph tree);

private:
	DatalogProgram * mpDatalogProgram;
	std::vector<Relation*> mRelations;
	std::vector<RelationRule> mRules;

	Graph * mpRuleGraph;
};

#endif /* DATABASE_H_ */
