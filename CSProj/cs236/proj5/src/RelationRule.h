/*
 * RelationRule.h
 *
 *  Created on: Mar 20, 2013
 *      Author: walter
 */

#ifndef RELATIONRULE_H_
#define RELATIONRULE_H_

#include <vector>
#include "Relation.h"
#include "Query.h"

using namespace std;

class Database;

class RelationRule {
public:
	RelationRule(Relation * relation);
	virtual ~RelationRule();

	void addQuery(Query query);
	void addAttribute(Attribute attr);
	Relation addTuples(Database * database);

	void updateRelation(Relation & relation);

	void toString();

	Relation * mpRelation;
	vector<Query> mQueryList;
	vector<Attribute> mAttributes;
};

#endif /* RELATIONRULE_H_ */
