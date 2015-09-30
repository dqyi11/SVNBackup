/*
 * CFlag.cpp
 *
 *  Created on: Sep 28, 2012
 *      Author: fausto
 */

#include "CFlag.h"
#include <iostream>

CFlag::CFlag() {
	// TODO Auto-generated constructor stub
	color = "";
	PTcolor = "";
	x = 0.0;
	y = 0.0;

}

CFlag::~CFlag() {
	// TODO Auto-generated destructor stub
}

void CFlag::print()
{

	cout << "The Flag state is : ";
	cout << "[COLOR]" << color;
	cout << "[PTCOLOR]" << PTcolor;
	cout << "[X]" << x;
	cout << "[Y]" << y << endl;
}
