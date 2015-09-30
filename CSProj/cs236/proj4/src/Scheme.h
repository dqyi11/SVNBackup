/*
 * Scheme.h
 *
 *  Created on: Feb 2, 2013
 *      Author: kyle
 */

#ifndef SCHEME_H_
#define SCHEME_H_

#include <vector>
#include <sstream>
#include "Lex.h"


class Scheme {
	friend class Relation;
public:
	Scheme(Lex* lex);
	~Scheme();
	std::string toString();
private:
	std::string identifier;
	std::vector<std::string> identifierList;
};

#endif /* SCHEME_H_ */
