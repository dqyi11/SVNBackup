/*
 * Parameter.h
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#ifndef PARAMETER_H_
#define PARAMETER_H_

#include "DatalogElement.h"
#include "Token.h"
#include <vector>

class Parser;

class Parameter: public DatalogElement {
public:
	Parameter();
	virtual ~Parameter();

	virtual std::string toString();

	bool parseTokens(Parser * parser);

private:
	std::vector<Token> mTokens;
};

#endif /* PARAMETER_H_ */
