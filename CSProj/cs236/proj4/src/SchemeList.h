/*
 * SchemeList.h
 *
 *  Created on: Feb 2, 2013
 *      Author: kyle
 */

#ifndef SCHEMELIST_H_
#define SCHEMELIST_H_

#include "Lex.h"
#include "TokenType.h"
#include "Scheme.h"

class SchemeList {
	friend class Database;
public:
	SchemeList(Lex* lex);
	virtual ~SchemeList();
	string toString();
protected:
	vector<Scheme*> schemes;
};

#endif /* SCHEMELIST_H_ */
