/*
 * Relation.cpp
 *
 *  Created on: Feb 19, 2013
 *      Author: walter
 */

#include "Relation.h"
#include <sstream>
#include <algorithm>

using namespace std;

const string primeSuffix = "-PRIME";

Relation::Relation(Scheme * scheme)
{
	if(scheme)
	{
		mName = scheme->identifier;

		vector<string>::iterator it;
		for(it=scheme->identifierList.begin();it!=scheme->identifierList.end();it++)
		{
			Attribute attr;
			attr.mName = (*it);
			mSchema.mAttributes.push_back(attr);
		}

	}
}

Relation::Relation(Relation * relation)
{
	if(relation)
	{
		mName = relation->mName;
		mSchema = relation->mSchema;
		mTupleSet = relation->mTupleSet;
	}
}

Relation::~Relation()
{
	// TODO Auto-generated destructor stub
	mTupleSet.clear();
}

bool Relation::importFromFactList(FactList * factList)
{
	if(NULL==factList)
	{
		return false;
	}

	vector<Fact*>::iterator it;
	for(it=factList->facts.begin();it!=factList->facts.end();it++)
	{
		importFromFact(*it);
	}

	return true;
}

bool Relation::importFromFact(Fact * fact)
{
	if(NULL==fact)
	{
		return false;
	}

	if(fact->identifier.compare(mName)!=0)
	{
		return false;
	}
    Tuple * tuple = new Tuple();
    int size = fact->identifierList.size();

    for(int i=0;i<size;i++)
    {
    	tuple->mTuples.insert(
    			pair<string, string>
    			(mSchema.mAttributes[i].mName, fact->identifierList[i]));
    }
    addTuple(*tuple);
    return true;
}

bool Relation::hasAttribute(string attrName)
{
	return mSchema.hasAttribute(attrName);

}

Attribute * Relation::getAttribute(std::string attrName)
{
	return mSchema.getAttribute(attrName);
}

int Relation::size(){
	return mTupleSet.size();
}

string Relation::toString(){
	stringstream rep;
	rep << mName << "(" << mSchema.toString() << ")" << endl;
	for(vector<Tuple>::iterator it = mTupleSet.begin(); it != mTupleSet.end(); ++it){
		rep << "  " << it->toString() << endl;
	}
	return rep.str();
}

Relation::Relation(const Relation& relation)
{
	this->mName = relation.mName;
	this->mSchema = relation.mSchema;
	this->mTupleSet = relation.mTupleSet;
}

Relation Relation::select(Attribute attr, string val)
{
	Relation newRelation = (*this);
	newRelation.clearTuples();
	for(vector<Tuple>::iterator it = mTupleSet.begin(); it != mTupleSet.end(); ++it)
		if(it->mTuples[attr.mName] == val)
			newRelation.addTuple(*it);

	return newRelation;
}

Relation Relation::select(Attribute attr1, Attribute attr2){
	Relation newRelation = (*this);
	newRelation.clearTuples();
	for(vector<Tuple>::iterator it = mTupleSet.begin(); it != mTupleSet.end(); ++it)
		if(it->mTuples[attr1.mName] == it->mTuples[attr2.mName])
			newRelation.addTuple(*it);

	newRelation = newRelation.projectOut(attr2);
	return newRelation;
}

Relation Relation::rename(Attribute attr, string newName)
{
	Relation newRelation = (*this);
	newRelation.clearTuples();

	Attribute * attrSel = newRelation.getAttribute(attr.mName);

	attrSel->mName = newName;

	std::vector<Tuple>::iterator itTS;
	for(itTS=this->mTupleSet.begin();
			itTS!=this->mTupleSet.end();itTS++)
	{
		Tuple * tuple = new Tuple();

		std::map<std::string, std::string>::iterator itT;
		for(itT=(*itTS).mTuples.begin();itT!=(*itTS).mTuples.end();
				itT++)
		{
			if((*itT).first.compare(attr.mName)==0)
			{
				tuple->mTuples.insert(
						pair<string, string>(newName, (*itT).second));
			}
			else
			{
				tuple->mTuples.insert(
						pair<string, string>((*itT).first, (*itT).second));
			}
		}

		newRelation.mTupleSet.push_back(*tuple);
	}

	/*
	cout << "Relation::rename " << attr.mName << " to " << newName << endl;
	cout << newRelation.toString() << endl;
	*/

	return newRelation;
}

Relation Relation::project(vector<Attribute> attrs)
{
	int size = attrs.size();
	/*
	cout << "Relation::project ";
	for(int i=0;i<size;i++)
	{
		cout << " " << attrs[i].mName << " ";
	}
	cout << endl;
	*/

	Relation newRelation = (*this);
	newRelation.clearTuples();

	newRelation.mSchema.mAttributes.clear();
	for(int i=0;i<size;i++)
	{
		newRelation.mSchema.mAttributes.push_back(attrs[i]);
	}

	std::vector<Tuple>::iterator itTS;
	for(itTS=this->mTupleSet.begin();
			itTS!=this->mTupleSet.end();itTS++)
	{
		Tuple * tuple = new Tuple();

		std::map<std::string, std::string>::iterator itT;
		for(itT=(*itTS).mTuples.begin();itT!=(*itTS).mTuples.end();
				itT++)
		{
			for(int i=0;i<size;i++)
			{
				if((*itT).first.compare(attrs[i].mName)==0)
				{
					tuple->mTuples.insert(
							pair<string, string>((*itT).first, (*itT).second));
				}
			}
		}
		newRelation.addTuple(*tuple);
	}

	//cout << newRelation.toString() << endl;

	return newRelation;
}

Relation Relation::projectOut(Attribute attr){
	vector<Attribute> attrs = getAttributes();
	vector<Attribute> newAttrs = vector<Attribute>();
	for(vector<Attribute>::iterator it=attrs.begin(); it!=attrs.end(); ++it)
		if(it->mName != attr.mName)
			newAttrs.push_back(*it);
	return project(newAttrs);
}

Relation Relation::crossProduct(Relation relation){

	Relation newRelation = (*this);
	//check precondition
	vector<Attribute>::iterator it;
	vector<Attribute> attrs = mSchema.getAttributes();
	for(it=attrs.begin(); it!=attrs.end(); ++it)
		if(relation.mSchema.hasAttribute(it->mName))
			return newRelation;

	newRelation.clearTuples();
	attrs = relation.getAttributes();
	for(it=attrs.begin(); it!=attrs.end(); ++it)
		newRelation.mSchema.mAttributes.push_back(*it);

	vector<Tuple>::iterator it1;
	for(it1=mTupleSet.begin(); it1!=mTupleSet.end(); ++it1){
		vector<Tuple>::iterator it2;
		for(it2=relation.mTupleSet.begin(); it2!=relation.mTupleSet.end(); ++it2){
			Tuple newTuple;
			map<string, string>::iterator it3;
			for(it3=it1->mTuples.begin(); it3!=it1->mTuples.end(); ++it3){
				newTuple.mTuples.insert(*it3);
			}
			for(it3=it2->mTuples.begin(); it3!=it2->mTuples.end(); ++it3){
				newTuple.mTuples.insert(*it3);
			}
			newRelation.addTuple(newTuple);
		}
	}
	return newRelation;
}

Relation Relation::naturalJoin(Relation relation)
{
	vector<Attribute> overlapAttrs1;
	vector<Attribute> overlapAttrs2;
	vector<Attribute> unionAttrs = this->mSchema.getAttributes();
 	vector<Attribute> attrs1 = this->mSchema.getAttributes();
	vector<Attribute> attrs2 = relation.mSchema.getAttributes();
	if(attrs1.size() == 0)
		return Relation(relation);

	vector<Attribute>::iterator it1, it2;
	for(it2=attrs2.begin();it2!=attrs2.end();it2++)
	{
		bool notOverlapAttr = true;
		for(it1=attrs1.begin();it1!=attrs1.end();it1++)
		{
			if((*it1).mName.compare((*it2).mName)==0)
			{
				overlapAttrs1.push_back(*it1);
				overlapAttrs2.push_back(*it2);
				notOverlapAttr = false;
			}
		}
		if(notOverlapAttr)
		{
			unionAttrs.push_back(*it2);
		}
	}

	//out << "check " << overlapAttrs1.size() << " " ;
	//cout << overlapAttrs2.size() << " " << unionAttrs.size() << endl;

	for(int i=0;i<(int)overlapAttrs2.size();i++)
	{
		relation = relation.rename(overlapAttrs2[i], overlapAttrs2[i].mName+primeSuffix);
		overlapAttrs2[i].mName += primeSuffix;
	}

	Relation newRelation = this->crossProduct(relation);

	if(overlapAttrs2.size()>0)
	{
		for(int i=0;i<(int)overlapAttrs2.size();i++)
		{
			newRelation = newRelation.select(overlapAttrs1[i],overlapAttrs2[i]);
		}
	}

	newRelation = newRelation.project(unionAttrs);

    return newRelation;
}

Relation Relation::unionSet(Relation relation){
	Relation newRelation = (*this);

	//check precondition
	if(relation.mSchema.getAttributes().size() != mSchema.getAttributes().size())
		return newRelation;
	vector<Attribute>::iterator it;
	vector<Attribute> attrs = mSchema.getAttributes();
	for(it=attrs.begin(); it!=attrs.end(); ++it)
		if(!relation.mSchema.hasAttribute(it->mName))
			return newRelation;


	vector<Tuple>::iterator it2;
	for(it2=relation.mTupleSet.begin(); it2!=relation.mTupleSet.end(); ++it2)
		newRelation.addTuple(*it2);

	return newRelation;
}

bool Relation::addTuple(Tuple tuple) {
	std::vector<Tuple>::iterator itTS;
	for(itTS=this->mTupleSet.begin();
			itTS!=this->mTupleSet.end();itTS++)
	{
		if((*itTS).compare(tuple)==true)
		{
			return false;
		}
	}
	mTupleSet.push_back(tuple);
	return true;
}

void Relation::clearTuples() {
	mTupleSet.clear();
}

void Relation::sortTuple()
{
	vector<Tuple> newTupleSet;
	newTupleSet.clear();

	for(vector<Tuple>::iterator itA=mTupleSet.begin();
			itA!=mTupleSet.end();itA++)
	{
		if(newTupleSet.size()==0)
		{
			newTupleSet.push_back(*itA);

		}
		else
		{
			bool insert = false;
			for(vector<Tuple>::iterator itB=newTupleSet.begin();
					itB!=newTupleSet.end();itB++)
			{
				if(lessTuple(*itA,*itB))
				{

					newTupleSet.insert(itB,*itA);
					insert = true;
					break;
				}

			}

			if(insert==false)
			{
				newTupleSet.push_back(*itA);
			}
		}

	}

	mTupleSet = newTupleSet;
}

bool Relation::lessTuple (Tuple& a,  Tuple& b)
{
	int aSize = a.mTuples.size();
	int bSize = b.mTuples.size();

	if(aSize < bSize)
	{
		return true;
	}
	vector<Attribute> attrs = getAttributes();

	for(vector<Attribute>::iterator it=attrs.begin();it!=attrs.end();it++)
	{
		string valA = a.mTuples[it->mName];
		string valB = b.mTuples[it->mName];

		int ret = valA.compare(valB);
		if(ret<0)
		{
			return true;
		}
		else if(ret>0)
		{
			return false;
		}
	}
	return false;
}
