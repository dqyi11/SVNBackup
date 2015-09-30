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

	if(program)
	{
		addRelations(mpDatalogProgram->schemeList->schemes);
		addTuples(mpDatalogProgram->factList->facts);
		addRules(mpDatalogProgram->ruleList->rules);
		processRules();
		importQueryList();
		/*
		Relation *test = getRelation("test");
		Relation *test2 = getRelation("test2");
		Relation *test3 = new Relation(NULL);
		//Relation test3 = test->unionSet(*test2);
		Relation test4 = test3->naturalJoin(*test2);
		cout << "natural join: " << endl << test4.toString() << endl;
		cout << "test: " << endl << test->toString() << endl;
		cout << "test2: " << endl << test2->toString() << endl;
		//cout << "union: " << endl << test3.toString() << endl;
		 */
	}
}

Database::~Database() {
	mRules.clear();
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

	/*
	vector<RelationRule>::iterator it2;
	for(it2=mRules.begin();it2!=mRules.end();it2++)
	{
		(*it2).toString();
	}
	*/
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
	/*
	cout << "importQuery:: ";
	cout << queryPredicate->toString() << endl;
	*/
	Relation * pRelation = findRelation(queryPredicate->getName());
	Query query = Query(queryPredicate);
	query.run(pRelation);
	cout << query.toString();
	return true;
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

void Database::processRules() {
	//fixed point algorithm
	int passes = 0;
	bool finished = false;
	while(!finished){
		vector<RelationRule>::iterator it;
		finished = true;
		for(it=mRules.begin(); it!=mRules.end(); ++it){
			Relation source = *(it->mpRelation);
			int size = source.size();
			Relation tempRel = it->addTuples(this);

			source = source.unionSet(tempRel);

			*(it->mpRelation) = source;

			if( source.size() != size )
				finished = false;

		}
		passes++;
	}

	cout << "Schemes populated after " << passes << " passes through the Rules." << endl;
}

int main(int argc, char* argv[]) {

	Parser* parser = new Parser(argv[1]);
	Database * database = new Database(parser->getProgram());
    return 0;
}
