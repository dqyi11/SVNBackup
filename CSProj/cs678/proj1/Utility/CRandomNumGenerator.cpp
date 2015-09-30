/*
 * CRandomNumGenerator.cpp
 *
 *  Created on: Feb 4, 2013
 *      Author: walter
 */

#include "CRandomNumGenerator.h"
#include <cstdlib>

CRandomNumGenerator::CRandomNumGenerator() {
	// TODO Auto-generated constructor stub
	mMax = 0;
	mMin = 0;
}

CRandomNumGenerator::~CRandomNumGenerator() {
	// TODO Auto-generated destructor stub

}

int CRandomNumGenerator::getRndNum()
{
	return (rand() % mMax + mMin);
}
