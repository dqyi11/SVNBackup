/*
 * Query.cpp
 *
 *  Created on: Mar 13, 2013
 *      Author: Kyle Corbitt
 */

#include "Query.h"
#include <algorithm>
#include <iostream>

using namespace std;

Query::Query(Predicate * queryPredicate):pQuery(NULL),results(NULL) {
	pQuery = queryPredicate;
}

Query::~Query()
{
	if(pQuery)
	{
		pQuery = NULL;
	}

	if(results)
	{
		delete results;
		results = NULL;
	}

}

Relation * Query::run(Relation* relation){

	if(results)
	{
		delete results;
		results = NULL;
	}

	results = new Relation(*relation);

	vector<QueryParam> params = getParam();
	vector<Attribute> attrs = results->getAttributes();

	if(params.size() != attrs.size())
	{
		cout << " number unmatch: params " << params.size();
		cout << " + attrs " << attrs.size() << endl;
		return results;
	}

	int num = params.size();
	for(int i=0;i<num;i++)
	{
		QueryParam queryParam = params[i];
		Attribute attr = attrs[i];


		//cout << (int)queryParam.mType << " + " << queryParam.mValue << endl;
		//cout << "Operation .... " << endl;
		//cout << results->toString() << endl;


		if(VARIABLE == queryParam.mType)
		{
			if(queryParam.mValue.compare(attr.mName)!=0)
			{
				Attribute * matchAttr = NULL;
				if(results->hasAttribute(queryParam.mValue))
				{
					matchAttr = results->getAttribute(queryParam.mValue);
					*results = results->select(*matchAttr, attr);

					vector<Attribute> attrs = results->getAttributes();
					vector<Attribute> newAttrs;
					int attrSize = attrs.size();
					for(int i=0;i<attrSize;i++)
					{
						vector<Attribute>::iterator it;
						for(it=newAttrs.begin();it!=newAttrs.end();it++)
						{
							if((*it).mName.compare(attrs[i].mName)!=0)
							{
								newAttrs.push_back(attrs[i]);
							}
						}
					}
					*results = results->project(attrs);
				}
				else
				{
					if(attr.mName.compare(queryParam.mValue)!=0)
					{
						*results = results->rename(attr, queryParam.mValue);
					}
				}

			}

		}
		else
		{
			*results = results->select(attr, queryParam.mValue);

			vector<Attribute> attrs;
			attrs = results->getAttributes();
			vector<Attribute>::iterator it;
			for(it=attrs.begin();it!=attrs.end();it++)
			{
				if((*it).mName.compare(attr.mName)==0)
				{
					attrs.erase(it);
					break;
				}
			}
			*results = results->project(attrs);
		}
	}

	results->sortTuple();
	return results;
}

vector<QueryParam> Query::getParam()
{
	vector<QueryParam> params;

	if(pQuery)
	{
		vector<Parameter*> paramList = pQuery->getParams();
		vector<Parameter*>::iterator it;
		for(it=paramList.begin();
				it!=paramList.end();
				it++)
		{
			if(NULL==(*it))
			{
				cout << " meet NULL  " << endl;
			}
			QueryParam param;
			if((*it)->getParaIdentifier().size()!=0)
			{
				param.mType = VARIABLE;
				param.mValue = (*it)->getParaIdentifier();
			}
			else if((*it)->getParaString().size()!=0)
			{
				param.mType = CONSTANT;
				param.mValue = (*it)->getParaString();
			}
			params.push_back(param);
		}
	}
	return params;
}

std::string Query::toString() {
	stringstream rep;
	vector<QueryParam> params = getParam();
	vector<string> variables = vector<string>();
	rep << pQuery->getName() << "(";
	for(vector<QueryParam>::iterator it=params.begin(); it!=params.end(); ++it){
		rep << it->mValue;
		if(it != params.end()-1)
			rep << ",";
		if(it->mType == VARIABLE && find(variables.begin(), variables.end(),it->mValue)==variables.end())
			variables.push_back(it->mValue);
	}
	rep << ")? ";
	vector<Tuple> tuples = results->mTupleSet;
	if(tuples.size() == 0){
		rep << "No" << endl;
	}else{
		rep << "Yes("<<tuples.size()<<")"<<endl;
		if(variables.size()>0)
		{
			for(vector<Tuple>::iterator itT=tuples.begin(); itT!=tuples.end(); ++itT){
				rep << "  ";
				for(vector<string>::iterator it=variables.begin(); it!=variables.end(); ++it){
					rep << *it << "=" << itT->mTuples[*it];
					if(it!=variables.end()-1)
						rep << ", ";
				}
				rep << endl;
			}
		}
	}
	return rep.str();
}

string Query::getName()
{
	if(NULL==pQuery)
	{
		return "";
	}

	return pQuery->getName();
}
