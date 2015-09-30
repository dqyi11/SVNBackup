/*
 * CFlag.h
 *
 *  Created on: Sep 28, 2012
 *      Author: fausto
 */

#ifndef CFLAG_H_
#define CFLAG_H_

#include <string>
#include "CBaseObject.h"

using namespace std;

class CFlag : public CBaseObject
{
public:
	CFlag();
	virtual ~CFlag();

	virtual void print();

	string color;
	string PTcolor;
	double x;
	double y;
};

#endif /* CFLAG_H_ */
