/*
 * Relation.h
 *
 *  Created on: Feb 19, 2013
 *      Author: walter
 */

#ifndef RELATION_H_
#define RELATION_H_

#include <string>
#include <vector>

#include "Schema.h"
#include "Tuple.h"
#include "Fact.h"
#include "FactList.h"
#include "Scheme.h"

class Relation {
	friend class Query;
public:
	Relation(Scheme * scheme);
	Relation(Relation * relation);
	Relation() {};
	virtual ~Relation();

	Relation(const Relation& relation);

	std::string getName() { return mName; };

	bool importFromFactList(FactList * factList);
	bool importFromFact(Fact * fact);

	bool hasAttribute(std::string attrName);
	Attribute * getAttribute(std::string attrName);
	int size();

	Relation select(Attribute attr, string val);
	Relation select(Attribute attr1, Attribute attr2);
	Relation rename(Attribute attr, string newName);
	Relation project(vector<Attribute> attrs);
	Relation projectOut(Attribute attr);

	Relation crossProduct(Relation relation);
	Relation naturalJoin(Relation relation);
	Relation unionSet(Relation relation);

	void sortTuple();
	bool lessTuple(Tuple& a,  Tuple& b);

	bool addTuple(Tuple tuple);
	void clearTuples();

	std::string toString();
	std::vector<Attribute> getAttributes() { return mSchema.getAttributes(); };
protected:
	std::vector<Tuple> mTupleSet;

private:
	std::string mName;
	Schema mSchema;
};

#endif /* RELATION_H_ */
