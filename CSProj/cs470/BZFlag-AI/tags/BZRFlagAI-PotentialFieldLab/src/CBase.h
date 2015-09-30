/*
 * CBase.h
 *
 *  Created on: Sep 28, 2012
 *      Author: fausto
 */

#ifndef CBASE_H_
#define CBASE_H_
#include <string>
#include <vector>
#include "CBaseObject.h"

using namespace std;

class CBase : public CBaseObject
{
	friend class CWorldController;

public:
	CBase(string color);
	virtual ~CBase();

	virtual void print();

	string mycolor;
	vector <double> xvals;
	vector <double> yvals;

};

#endif /* CBASE_H_ */
