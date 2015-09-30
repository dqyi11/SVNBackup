/*
 * RelationRule.cpp
 *
 *  Created on: Mar 20, 2013
 *      Author: walter
 */

#include "RelationRule.h"
#include "Database.h"

RelationRule::RelationRule(Relation * relation) {
	// TODO Auto-generated constructor stub
	mpRelation = relation;
}

RelationRule::~RelationRule() {
	// TODO Auto-generated destructor stub
	mAttributes.clear();
	mQueryList.clear();
}

void RelationRule::addQuery(Query query)
{
	mQueryList.push_back(query);
}

void RelationRule::addAttribute(Attribute attr)
{
	mAttributes.push_back(attr);
}

Relation RelationRule::addTuples(Database * database){
	vector<Query>::iterator it;
	Relation results;

	for(it=mQueryList.begin(); it!=mQueryList.end(); ++it)
	{
		string queryName = it->pQuery->getName();
		Relation * temp = database->findRelation(queryName);
		Relation * tempRel = it->run(temp);

		results = results.naturalJoin(*tempRel);
	}

	results = results.project(mAttributes);
	vector<Attribute> relationAttrs = mpRelation->getAttributes();
	for(int i=0; i<mAttributes.size(); i++)
		results = results.rename(mAttributes[i], relationAttrs[i].mName);

	return results;
}

void RelationRule::toString()
{
	cout << "Relation: ";
	if(mpRelation)
	{
		cout << mpRelation->getName()<<" = ";
	}

	for(vector<Attribute>::iterator it=mAttributes.begin();it!=mAttributes.end();it++)
	{
		cout << (*it).mName << " ";
	}
	cout << endl;

	for(vector<Query>::iterator it=mQueryList.begin();it!=mQueryList.end();it++)
	{
		cout << "Query:" << (*it).pQuery->toString() << endl;
	}
}

void RelationRule::updateRelation(Relation & relation)
{

}
