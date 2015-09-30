/*
 * Predicate.h
 *
 *  Created on: Feb 5, 2013
 *      Author: kyle
 */

#ifndef PREDICATE_H_
#define PREDICATE_H_

#include <vector>
#include <sstream>
#include "Lex.h"
#include "Domain.h"
#include "Parameter.h"

class Predicate {
public:
	Predicate(Lex* lex, std::Domain* domain);
	~Predicate();
	std::string toString();

	std::string getName() { return identifier; };
	std::vector<Parameter*> getParams() { return parameterList; };

private:
	std::string identifier;
	std::vector<Parameter*> parameterList; //TODO: generalize this to expression
};

#endif /* PREDICATE_H_ */
