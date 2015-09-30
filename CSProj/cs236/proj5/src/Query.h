/*
 * Query.h
 *
 *  Created on: Mar 13, 2013
 *      Author: Kyle Corbitt
 */

#ifndef QUERY_H_
#define QUERY_H_

#include "Predicate.h"
#include "Schema.h"
#include "Relation.h"

enum QueryParamType {
	CONSTANT, VARIABLE,
};

typedef struct QueryParam {
	QueryParamType mType;
	string         mValue;
};

class Query {
	friend class RelationRule;
public:
	Query(Predicate * query);
	~Query();
	std::string toString();
	vector<QueryParam> getParam();
	Relation * run(Relation* relation);

	std::string getName();

protected:
	Predicate * pQuery;
	Relation * results;
};

#endif /* QUERY_H_ */
