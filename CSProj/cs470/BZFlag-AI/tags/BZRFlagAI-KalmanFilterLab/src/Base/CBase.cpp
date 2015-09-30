/*
 * CBase.cpp
 *
 *  Created on: Sep 28, 2012
 *      Author: fausto
 */

#include "CBase.h"
#include <iostream>

CBase::CBase(string color) {
	// TODO Auto-generated constructor stub
	mycolor = color;

}

CBase::~CBase() {
	// TODO Auto-generated destructor stub
}

void CBase::print()
{
	vector<double>::iterator it;

	cout<<"The base state is : " << "[COLOR]" << mycolor << endl;
	cout<<"X pos is : ";
	for (it = xvals.begin(); it != xvals.end(); it ++)
	{
		cout << (*it) << " ";
	}

	cout<<"Y pos is : ";
	for (it = yvals.begin(); it != yvals.end(); it ++)
	{
		cout << (*it) << " ";
	}
	cout << endl;


}
