/*
 * CDataArrayVisualizer.cpp
 *
 *  Created on: Jan 23, 2013
 *      Author: walter
 */

#include "CDataArrayVisualizer.h"
#include <string.h>
#include "gnuplot_i.hpp"


#define MAX_LEN 512

CDataArrayVisualizer::CDataArrayVisualizer() {
	// TODO Auto-generated constructor stub
	mDataValue.clear();
}

CDataArrayVisualizer::~CDataArrayVisualizer() {
	// TODO Auto-generated destructor stub
	mDataValue.clear();
}

void CDataArrayVisualizer::sortData()
{
	mDataValue.sort();

	cout << "OUTPUT SCORE:" << endl;
	list<double>::iterator itD;
	for(itD=mDataValue.begin();itD!=mDataValue.end();itD++)
	{
		cout << (*itD) << endl;
	}
	cout << "END OUTPUT SCORE" << endl;
}

void CDataArrayVisualizer::plot()
{
	Gnuplot mDiscretizedMap;
	char temp[MAX_LEN];

	mDiscretizedMap << "set term png size 5000,500 crop\n";
	mDiscretizedMap << "set term png font ',12' linewidth 1\n";
    mDiscretizedMap << "set output 'data.png' \n";

    int number_count = 1;

    list<double>::iterator it;
    for(it=mDataValue.begin();it!=mDataValue.end();it++)
    {
    	double numTemp = (*it);
    	memset(temp,0x0,MAX_LEN);
		sprintf(temp, "set object %d circle at %f,%d size 0.2 \n", number_count, numTemp, number_count);
    	mDiscretizedMap << temp;
    	number_count++;
    }

    memset(temp,0x0,MAX_LEN);
	sprintf(temp, "plot %d \n", number_count);

    mDiscretizedMap << temp;

}
