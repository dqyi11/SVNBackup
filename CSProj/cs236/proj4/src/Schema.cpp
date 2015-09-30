/*
 * Schema.cpp
 *
 *  Created on: Feb 19, 2013
 *      Author: walter
 */

#include "Schema.h"
#include <sstream>

using namespace std;

Schema::Schema()
{
	// TODO Auto-generated constructor stub
	mAttributes.clear();
}

Schema::~Schema() {
	// TODO Auto-generated destructor stub
	mAttributes.clear();
}

string Schema::toString() {
	stringstream rep;
	for(vector<Attribute>::iterator it = mAttributes.begin(); it != mAttributes.end(); ++it){
		rep << (*it).mName;
		if(it != mAttributes.end() -1 )
			rep << ",";
	}
	return rep.str();
}

bool Schema::hasAttribute(std::string attrName)
{
	if(NULL==getAttribute(attrName))
	{
		return false;
	}

	return true;
}

Attribute * Schema::getAttribute(std::string attrName)
{
	for(vector<Attribute>::iterator it = mAttributes.begin(); it != mAttributes.end(); ++it)
	{
		if((*it).mName.compare(attrName)==0)
		{
			return &(*it);
		}
	}

	return NULL;
}
