/*
 * Domain.h
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#ifndef DOMAIN_H_
#define DOMAIN_H_

#include "DatalogElement.h"
#include "Token.h"
#include <list>

using namespace std;

class Domain: public DatalogElement {
public:
	Domain();
	virtual ~Domain();

	virtual std::string toString();

	bool hasToken(Token token);
	void addToken(Token token);

private:
	list<Token> mDomainTokens;



};

#endif /* DOMAIN_H_ */
