/*
 * Database.cpp
 *
 *  Created on: Feb 19, 2013
 *      Author: walter
 */

#include "Database.h"
#include "Query.h"

using namespace std;

Database::Database(DatalogProgram * program)
{
	mpDatalogProgram = program;
	mRelations.clear();
	mpRuleGraph = new Graph();

	if(program)
	{
		addRelations(mpDatalogProgram->schemeList->schemes);
		addTuples(mpDatalogProgram->factList->facts);
		addRules(mpDatalogProgram->ruleList->rules);

		initRuleGraph();
		importQueryList();
	}
}

Database::~Database() {
	mRules.clear();

	if(mpRuleGraph)
	{
		delete mpRuleGraph;
		mpRuleGraph = NULL;
	}

	vector<Relation*>::iterator it;
	Relation * pRelation = NULL;
	for(it=mRelations.begin();it!=mRelations.end();it++)
	{
		pRelation = (*it);
		delete pRelation;
		pRelation = NULL;
	}
	mRelations.clear();
}

void Database::addRelations(vector<Scheme*> schemes){
	vector<Scheme*>::iterator it;
	for(it=schemes.begin(); it != schemes.end();++it){
		mRelations.push_back(new Relation(*it));
	}
}

void Database::addTuples(vector<Fact*> facts){
	vector<Fact*>::iterator it;
	for(it=facts.begin(); it != facts.end();++it){
		Relation* matchedRelation = getRelation((*it)->identifier);
		matchedRelation->importFromFact((*it));
	}
}

void Database::addRules(vector<Rule*> rules)
{
	vector<Rule*>::iterator it;
	for(it=rules.begin(); it != rules.end();++it){
		importRule(*it);
	}
}


Relation * Database::getRelation(string name)
{
	vector<Relation*>::iterator it;
	for(it=mRelations.begin();it!=mRelations.end();it++)
	{
		if((*it)->getName().compare(name)==0)
		{
			return (*it);
		}
	}
	return NULL;
}

string Database::toString() {
	stringstream rep;
	vector<Relation*>::iterator it;
	for(it=mRelations.begin();it!=mRelations.end();it++)
	{
		rep << (*it)->toString() << endl;
	}
	return rep.str();
}

bool Database::importQuery(Predicate * queryPredicate)
{
	if(NULL==queryPredicate)
	{
		return false;
	}

	string queryName = queryPredicate->getName();

	RelationRule * pRule = getRelationRule(queryName);
	if(pRule)
	{
		Graph dfs = mpRuleGraph->getGraphByDFS(queryName);

		vector<RelationRule> rulesToApply = getRulesFromTree(dfs);
		bool isCyclic = dfs.isCyclic();

		processRules(rulesToApply, isCyclic);
	}

	Relation * pRelation = findRelation(queryName);

	Query query = Query(queryPredicate);
	query.run(pRelation);
	cout << query.toString();
	return true;
}

vector<RelationRule> Database::getRulesFromTree(Graph tree){
	vector<RelationRule> rules;

	vector<Node*>::iterator it;
	for(it=tree.mNodeList.begin();it!=tree.mNodeList.end();++it) {
		string name = (*it)->getName();
		vector<RelationRule>::iterator it;

		for(it=mRules.begin(); it!=mRules.end(); ++it)
			if((*it).mpRelation->getName().compare(name)==0)
				rules.push_back(*it);
	}

	return rules;
}

bool Database::importQueryList()
{
	vector<Predicate*>::iterator it;
	for(it=mpDatalogProgram->queryList->queries.begin();
	 it!=mpDatalogProgram->queryList->queries.end();
	 it++
	)
	{
		importQuery(*it);
	}

	return true;
}

bool Database::importRule(Rule * rule)
{
	if(NULL==rule)
	{
		return false;
	}

	Relation * pRelation = findRelation(rule->identifier);
	RelationRule relationRule(pRelation);

	for(vector<string>::iterator it=rule->identifierList.begin();it!=rule->identifierList.end();it++)
	{
		Attribute attr;
		attr.mName = (*it);
		relationRule.addAttribute(attr);
	}

	for(vector<Predicate*>::iterator it=rule->predicateList.begin();it!=rule->predicateList.end();it++)
	{
		Query query(*it);
		relationRule.addQuery(query);
	}

	mRules.push_back(relationRule);
	return true;
}

Relation * Database::findRelation(std::string name)
{
	Relation * pRelation = NULL;

	vector<Relation*>::iterator it;
	for(it=mRelations.begin();it!=mRelations.end();it++)
	{
		if((*it)->getName().compare(name)==0)
		{
			pRelation = (*it);
			return pRelation;
		}
	}

	return pRelation;
}

void Database::processRules(vector<RelationRule> rules, bool LFP) {
	if(LFP) {
		int cycles = 0;
		//fixed point algorithm
		bool changes = true;
		while(changes){
			changes = processRulesOnce(rules);
			cycles++;
		}
	}
	else
		processRulesOnce(rules);
}

bool Database::processRulesOnce(vector<RelationRule> rules) {
	vector<RelationRule>::iterator it;
	bool changes = false;
	for(it=rules.begin(); it!=rules.end(); ++it){
		Relation source = *(it->mpRelation);
		int size = source.size();
		Relation tempRel = it->addTuples(this);

		source = source.unionSet(tempRel);

		*(it->mpRelation) = source;

		if( source.size() != size )
			changes = true;
	}

	return changes;
}

void Database::initRuleGraph()
{
	// init nodes
	vector<RelationRule>::iterator it;
	for(it=mRules.begin(); it!=mRules.end(); ++it){
		mpRuleGraph->addNode((*it).mpRelation->getName());
	}

	// find dependency
	for(it=mRules.begin(); it!=mRules.end(); ++it){
		vector<Query>::iterator itQ;
		for(itQ=(*it).mQueryList.begin();itQ!=(*it).mQueryList.end();itQ++)
		{
			Node* dependNode = mpRuleGraph->getNode((*itQ).getName());
			if(dependNode)
		    {
				mpRuleGraph->connect((*it).mpRelation->getName(),
						dependNode->getName());
			}
		}
	}
}

RelationRule * Database::getRelationRule(std::string name)
{
	vector<RelationRule>::iterator it;
	for(it=mRules.begin(); it!=mRules.end(); ++it)
	{
		if((*it).mpRelation->getName().compare(name)==0)
		{
			return &(*it);
		}
	}
	return NULL;
}

int main(int argc, char* argv[]) {
	Parser* parser = new Parser(argv[1]);
	Database * database = new Database(parser->getProgram());
    return 0;
}
