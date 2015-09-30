/*
 * Schema.h
 *
 *  Created on: Feb 19, 2013
 *      Author: walter
 */

#ifndef SCHEMA_H_
#define SCHEMA_H_

#include <string>
#include <vector>

typedef struct Attribute {
	std::string mName;
};

class Schema {
	friend class Relation;
public:
	Schema();
	virtual ~Schema();
	std::string toString();

	std::vector<Attribute> getAttributes() { return mAttributes; };

	bool hasAttribute(std::string attrName);
	Attribute * getAttribute(std::string attrName);

protected:
	std::vector<Attribute> mAttributes;
};

#endif /* SCHEMA_H_ */
